/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2020 Inviwo Foundation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************************/

#include <inviwo/mergetreemaps/processors/contourtreesequencetolandscape.h>
#include <modules/base/algorithm/dataminmax.h>
#include <inviwo/mergetreemaps/mergetreemaputils.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo ContourTreeSequenceToLandscape::processorInfo_{
    "org.inviwo.ContourTreeSequenceToLandscape",  // Class identifier
    "Contour Tree Sequence To Landscape",         // Display name
    "Merge Tree Maps",                            // Category
    CodeState::Experimental,                      // Code state
    Tags::None,                                   // Tags
};
const ProcessorInfo ContourTreeSequenceToLandscape::getProcessorInfo() const {
    return processorInfo_;
}

ContourTreeSequenceToLandscape::ContourTreeSequenceToLandscape()
    : PoolProcessor()
    , treeType_("treeType", "Tree Type", "", InvalidationLevel::Valid)
    , mapOriginalScalars_("mapOriginalScalars", "Map Original/Additional Scalars", false)
    , useUnaugmentedTreeOnly_("unaugmentedOnly", "Only Use Unaugmented Tree", false)
    , position_("position", "Position", false)
    , dataDims_("dataDims", "Data Dimensions", 2, 1, 3)
    , orderMode_("orderMode", "Order Mode")
    , timer_("timer", "Eval Time (s)", 0.f, 0.f, std::numeric_limits<float>::max(), 0.001f,
             InvalidationLevel::Valid, PropertySemantics::Text) {

    addPort(inport_);
    addPort(originalScalars_);

    addPort(decisionsInport_);
    decisionsInport_.setOptional(true);
    addPort(temporalTreeInport_);
    temporalTreeInport_.setOptional(true);
    temporalTreeInport_.onConnect([&]() { this->invalidate(InvalidationLevel::InvalidResources); });
    temporalTreeInport_.onDisconnect(
        [&]() { this->invalidate(InvalidationLevel::InvalidResources); });

    addPort(outport_);

    addProperty(treeType_);
    treeType_.setReadOnly(true);
    inport_.onChange([this]() {
        treeType_.set([this]() -> std::string {
            if (inport_.hasData() && inport_.getData()->size() >= 1) {
                return toString(inport_.getData()->at(0)->type);
            } else {
                return "";
            }
        }());
    });

    addProperties(mapOriginalScalars_, useUnaugmentedTreeOnly_, position_, dataDims_, orderMode_,
                  timer_);

    orderMode_.addOption("none", "None", mtmutils::SuperArcOrderMode::None);
    orderMode_.addOption("byId", "By Id", mtmutils::SuperArcOrderMode::ById);
    orderMode_.addOption("byIdReverse", "By Reverse Id", mtmutils::SuperArcOrderMode::ByIdReverse);
    orderMode_.addOption("bySize", "Be Size Below", mtmutils::SuperArcOrderMode::BySize);
    orderMode_.addOption("byDecision", "By Left/Right Decision",
                         mtmutils::SuperArcOrderMode::ByDecision);
    orderMode_.addOption("byTemporal", "By Temporal Tree",
                         mtmutils::SuperArcOrderMode::ByTemporalTree);
    orderMode_.setSelectedValue(mtmutils::SuperArcOrderMode::ById);
    orderMode_.setCurrentStateAsDefault();
    orderMode_.onChange([this]() {
        decisionsInport_.setOptional(orderMode_.get() != mtmutils::SuperArcOrderMode::ByDecision);
        temporalTreeInport_.setOptional(orderMode_.get() !=
                                        mtmutils::SuperArcOrderMode::ByTemporalTree);
    });

    dataDims_.visibilityDependsOn(position_, [](const auto& p) { return p.get(); });

    timer_.setReadOnly(true);
}

void ContourTreeSequenceToLandscape::process() {
    if (!inport_.getData() || inport_.getData()->empty()) return;
    if (!originalScalars_.getData() || originalScalars_.getData()->empty()) return;
    if (inport_.getData()->size() != originalScalars_.getData()->size()) {
        LogProcessorError("Number of contour trees and scalar fields do not match.");
        return;
    }
    if (orderMode_.get() == mtmutils::SuperArcOrderMode::ByDecision && !decisionsInport_.getData())
        return;

    performanceTimer_.Reset();

    const auto treesData = inport_.getData();
    // auto tree = treesData->at(0)->getTree();
    auto type = treesData->at(0)->type;
    if (!(type == topology::TreeType::Join || type == topology::TreeType::Split)) {
        LogProcessorError("Treetype" << toString(type) << " not supported ");
        return;
    }

    vertexOrderMapPerTimeStep = std::vector<VertexOrderMap>(treesData->size());

    if (temporalTreeInport_.isConnected() && temporalTreeInport_.getData()) {

        auto temporalTree = temporalTreeInport_.getData();
        const auto contourTrees = *treesData.get();

        // Get order map
        if (temporalTree->order.empty()) {
            LogProcessorWarn("No order on leaves is given.");
        }

        // indexmap is initialized in the function
        bool success = mtmutils::getVertexOrderPerTimeStepFromTemporal(
            *temporalTree.get(), contourTrees, vertexOrderMapPerTimeStep);
        if (!success) {
            LogProcessorError(
                "Creating node index map between contour and temporal tree was not successful.");
            return;
        }
    }

    if (decisionsInport_.isConnected() && decisionsInport_.getData()) {
        auto decisionsPerStep = decisionsInport_.getData();
        if (decisionsPerStep->size() < treesData->size()) {
            LogProcessorError(
                "Array of decisions per time step does not contain enough time steps.");
            return;
        }
    }

    using Result = std::shared_ptr<DataFrame>;

    const auto computeLandscapeJob =
        [](const topology::ContourTreeData& data,
           const topology::TriangulationData& originalScalars, const size_t timeStep,
           const bool mapOriginalScalars, const bool useUnaugmentedTreeOnly,
           const bool includePositions, const size_t dataDims,
           const mtmutils::SuperArcOrderMode& orderMode, const VertexOrderMap& orderMap,
           const std::vector<int>& decisions) {
            return [data, originalScalars, timeStep, mapOriginalScalars, useUnaugmentedTreeOnly,
                    includePositions, dataDims, orderMode, orderMap,
                    decisions](pool::Stop, pool::Progress progress) -> Result {
                return dispatching::dispatch<Result, dispatching::filter::Float1s>(
                    data.triangulation->getScalarValues()->getDataFormat()->getId(),
                    mtmutils::ComputeLandscape{}, progress, data, originalScalars, timeStep,
                    mapOriginalScalars, useUnaugmentedTreeOnly, includePositions, dataDims,
                    orderMode, orderMap, decisions);
            };
        };

    std::vector<std::function<Result(pool::Stop, pool::Progress progress)>> jobs;
    for (size_t i = 0; i < treesData.get()->size(); i++) {
        std::vector<int> decisions;
        auto currentTree = *treesData.get()->at(i).get();
        if (decisionsInport_.isConnected() && decisionsInport_.getData()) {
            auto decisionsPerStep = decisionsInport_.getData();
            decisions = decisionsInport_.getData()->at(i);
            if (decisions.size() != currentTree.getTree()->getNumberOfSuperArcs()) {
                LogProcessorError("Array with decisions does not contain enough super arcs.");
                return;
            }
        }
        // TODO: Extract treeorderMap per timestep to avoid copying these
        jobs.push_back(computeLandscapeJob(currentTree, *originalScalars_.getData()->at(i).get(), i,
                                           mapOriginalScalars_.get(), useUnaugmentedTreeOnly_.get(),
                                           position_.get(), dataDims_.get(), orderMode_.get(),
                                           vertexOrderMapPerTimeStep[i], decisions));
    }

    outport_.setData(nullptr);
    dispatchMany(jobs, [this](std::vector<Result> result) {
        dfs_ = result;
        outport_.setData(std::make_shared<DataFrameSequence>(dfs_));
        newResults();
        timer_.set(performanceTimer_.ElapsedTimeAndReset());
    });
}

}  // namespace inviwo
