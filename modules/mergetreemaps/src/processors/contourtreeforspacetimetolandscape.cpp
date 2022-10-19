/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2021 Inviwo Foundation
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

#include <inviwo/mergetreemaps/processors/contourtreeforspacetimetolandscape.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo ContourTreeForSpaceTimeToLandscape::processorInfo_{
    "org.inviwo.ContourTreeForSpaceTimeToLandscape",  // Class identifier
    "Contour Tree For Space Time To Landscape",       // Display name
    "Merge Tree Maps",                                // Category
    CodeState::Experimental,                          // Code state
    Tags::None,                                       // Tags
};
const ProcessorInfo ContourTreeForSpaceTimeToLandscape::getProcessorInfo() const {
    return processorInfo_;
}

ContourTreeForSpaceTimeToLandscape::ContourTreeForSpaceTimeToLandscape()
    : PoolProcessor()
    , treeType_("treeType", "Tree Type", "", InvalidationLevel::Valid)
    , position_("position", "Position", false)
    , dataDims_("dataDims", "Data Dimensions", 2, 1, 3)
    , timer_("timer", "Eval Time (s)", 0.f, 0.f, std::numeric_limits<float>::max(), 0.001f,
             InvalidationLevel::Valid, PropertySemantics::Text) {

    addPort(inport_);
    addPort(originalScalars_);
    addPort(outport_);
    addProperties(treeType_, position_, dataDims_, timer_);
    timer_.setReadOnly(true);

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

    dataDims_.visibilityDependsOn(position_, [](const auto& p) { return p.get(); });
}

void ContourTreeForSpaceTimeToLandscape::process() {

    if (!inport_.getData()) return;
    if (!originalScalars_.getData() || originalScalars_.getData()->empty()) return;

    performanceTimer_.Reset();

    const auto tree = inport_.getData();
    auto type = tree->type;
    if (!(type == topology::TreeType::Join || type == topology::TreeType::Split)) {
        LogProcessorError("Treetype" << toString(type) << " not supported ");
        return;
    }

    const topology::ContourTreeData& data = *tree.get();
    const TriangulationSequence& originalScalars = *originalScalars_.getData();

    auto numVertices = tree->getTree()->getNumberOfVertices();
    auto numVerticesPerTimeStep = originalScalars[0]->getPoints().size();

    if (numVertices != numVerticesPerTimeStep * originalScalars.size()) {
        LogProcessorError("Space-Time Data and per time step data does not match in size.");
        return;
    }

    using Result = std::shared_ptr<DataFrameSequence>;

    const auto computeLandscapeJob = [data, originalScalars, includePositions = position_.get(),
                                      dataDims = dataDims_.get()](
                                         pool::Stop, pool::Progress progress) -> Result {
        return dispatching::dispatch<Result, dispatching::filter::Float1s>(
            data.triangulation->getScalarValues()->getDataFormat()->getId(),
            mtmutils::ComputeLandscapeSpaceTime{}, progress, data, originalScalars.size(),
            includePositions, dataDims);
    };

    outport_.setData(nullptr);
    dispatchOne(computeLandscapeJob, [this](Result result) {
        outport_.setData(result);
        newResults();
        timer_.set(performanceTimer_.ElapsedTimeAndReset());
    });
}  // namespace inviwo

}  // namespace inviwo
