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

#include <inviwo/mergetreemaps/processors/contourtreetolandscape.h>
#include <modules/base/algorithm/dataminmax.h>
#include <modules/temporaltreemaps/datastructures/treeorder.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo ContourTreeToLandscape::processorInfo_{
    "org.inviwo.ContourTreeToLandscape",  // Class identifier
    "Contour Tree To Landscape",          // Display name
    "Merge Tree Maps",                    // Category
    CodeState::Experimental,              // Code state
    Tags::None,                           // Tags
};
const ProcessorInfo ContourTreeToLandscape::getProcessorInfo() const { return processorInfo_; }

ContourTreeToLandscape::ContourTreeToLandscape()
    : PoolProcessor()
    , treeType_("treeType", "Tree Type", "", InvalidationLevel::Valid)
    , mapOriginalScalars_("mapOriginalScalars", "Map Original/Additional Scalars", false)
    , useUnaugmentedTreeOnly_("unaugmentedOnly", "Only Use Unaugmented Tree", false)
    , position_("position", "Position", false)
    , dataDims_("dataDims", "Data Dimensions", 2, 1, 3)
    , orderMode_("orderMode", "Order Mode")
    , timestep_("timestep", "Time Step", 0)
    , timestepSequenceIndex_("timestepSequenceIndex", "Time Step Sequence Index", 1)
    , levelSet_("exportLevelSet", "Export Super/Sub-Level Set", false)
    , isoValue_("isoValue", "Iso value", 0.f, -100.f, 100.f) {

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
            if (inport_.hasData()) {
                return toString(inport_.getData()->type);
            } else {
                return "";
            }
        }());
    });

    addProperties(mapOriginalScalars_, useUnaugmentedTreeOnly_, position_, dataDims_, orderMode_,
                  timestep_, timestepSequenceIndex_, levelSet_, isoValue_);
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
    isoValue_.visibilityDependsOn(levelSet_, [](const auto& p) { return p.get(); });

    timestep_.onChange([&]() { timestepSequenceIndex_.set(timestep_.get() + 1); });
    timestepSequenceIndex_.onChange([&]() { timestep_.set(timestepSequenceIndex_.get() + 1); });
    timestepSequenceIndex_.setReadOnly(true);
    timestepSequenceIndex_.setSemantics(PropertySemantics::Text);
}

void ContourTreeToLandscape::process() {
    if (!inport_.getData() || !originalScalars_.getData()) return;
    auto tree = inport_.getData()->getTree();
    auto type = inport_.getData()->type;
    if (!(type == topology::TreeType::Join || type == topology::TreeType::Split)) {
        LogProcessorError("Treetype" << toString(type) << " not supported.");
        return;
    }

    /*
    Rough idea for countour tree:
    Collect root candidates
    Root Candidate would be the an endpoint of an arc
    where there is just a singe level set component on that arc and at the endpoint
    Use the root candidate that is most in the "middle" of the dataset
    Add direction to traversal
    There are also direction changes possible in the case of siblings
    (e.g. in the case of the only root candidate being global min/max,
    siblings need to be traversed too)
    Up/Down are the same as in the Split Tree
    */

    if (temporalTreeInport_.isConnected() && temporalTreeInport_.getData()) {

        auto temporalTree = temporalTreeInport_.getData();
        // Get order map
        if (temporalTree->order.empty()) {
            LogProcessorWarn("No order on leaves is given.");
        }

        ContourTreeSequence contourTrees(1);
        contourTrees[0] = std::make_shared<topology::ContourTreeData>(*inport_.getData());
        // indexmap is initialized in the function
        std::vector<VertexOrderMap> vertexOrderMaps(1);
        bool success = mtmutils::getVertexOrderPerTimeStepFromTemporal(
            *temporalTree.get(), contourTrees, vertexOrderMaps, timestep_.get());
        vertexOrderMap_ = vertexOrderMaps[0];
        if (!success) {
            LogProcessorError(
                "Creating node index map between contour and temporal tree was not successful.");
            return;
        }
    }

    std::vector<int> decisions;

    if (decisionsInport_.isConnected() && decisionsInport_.getData()) {
        auto decisionsPerStep = decisionsInport_.getData();
        if (decisionsPerStep->size() < timestep_.get()) {
            LogProcessorError(
                "Array of decisions per time step does not contain enough time steps.");
            return;
        }
        decisions = decisionsInport_.getData()->at(timestep_.get());
        if (decisions.size() != tree->getNumberOfSuperArcs()) {
            LogProcessorError("Array with decisions does not contain enough super arcs.");
            return;
        }
    }

    using Result = std::shared_ptr<DataFrame>;

    const auto computeLandscape =
        [data = inport_.getData(), originalScalars = originalScalars_.getData(),
         timeStep = timestep_.get(), mapOriginalScalars = mapOriginalScalars_.get(),
         useUnaugmentedTreeOnly = useUnaugmentedTreeOnly_.get(), dataDims = dataDims_.get(),
         includePositions = position_.get(), orderMode = orderMode_.get(),
         vertexOrderMap = vertexOrderMap_, &decisions](pool::Stop stop, pool::Progress progress) {
            return dispatching::dispatch<Result, dispatching::filter::Float1s>(
                data->triangulation->getScalarValues()->getDataFormat()->getId(),
                mtmutils::ComputeLandscape{}, progress, *data.get(), *originalScalars.get(),
                timeStep, mapOriginalScalars, useUnaugmentedTreeOnly, includePositions, dataDims,
                orderMode, vertexOrderMap, decisions);
        };

    outport_.setData(nullptr);
    dispatchOne(computeLandscape, [this, &type](Result result) {
        if (levelSet_) {
            auto column = result->getColumn(4);
            if (column->getHeader() == "Scalar Value") {
                size_t numRows = result->getNumberOfRows();
                std::vector<int> isInLevelSet(result->getNumberOfRows());
                for (size_t i = 0; i < numRows; i++) {
                    if (type == topology::TreeType::Join) {
                        isInLevelSet[i] = column->getAsDouble(i) >= isoValue_.get();
                    } else {
                        isInLevelSet[i] = column->getAsDouble(i) <= isoValue_.get();
                    }
                }
                result->addColumnFromBuffer("Is in Level Set",
                                            util::makeBuffer<int>(std::move(isInLevelSet)));
            }
        }
        outport_.setData(std::move(result));
        newResults();
    });
}

}  // namespace inviwo
