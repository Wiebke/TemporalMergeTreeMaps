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

#include <inviwo/mergetreemaps/processors/triangulationtovolumeforsequence.h>
#include <modules/base/algorithm/dataminmax.h>
#include <inviwo/topologytoolkit/utils/ttkutils.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo TriangulationToVolumeForSequence::processorInfo_{
    "org.inviwo.TriangulationToVolumeForSequence",  // Class identifier
    "Triangulation To Volume For Sequence",         // Display name
    "Topology",                                     // Category
    CodeState::Experimental,                        // Code state
    Tags::None,                                     // Tags
};
const ProcessorInfo TriangulationToVolumeForSequence::getProcessorInfo() const {
    return processorInfo_;
}

TriangulationToVolumeForSequence::TriangulationToVolumeForSequence()
    : Processor()
    , inport_("inport")
    , outport_("outport")
    , nearestNeighborInterpolation_("nearestNeighborInterpolation", "Use NN interpolation", false)
    , useCustomDataRange_("useCustomRange", "Use Custom Range", false)
    , customDataRange_("customDataRange", "Custom Data Range", 0.0, 1.0,
                       std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max(),
                       0.01, 0.0, InvalidationLevel::InvalidOutput, PropertySemantics::Text)
    , dataRange_("dataRange", "Output Range", 0.0, 1.0, std::numeric_limits<double>::lowest(),
                 std::numeric_limits<double>::max(), 0.01, 0.0, InvalidationLevel::Valid,
                 PropertySemantics::Text)
    , timer_("timer", "Eval Time (s)", 0.f, 0.f, std::numeric_limits<float>::max(), 0.001f,
             InvalidationLevel::Valid, PropertySemantics::Text) {

    addPort(inport_);
    addPort(outport_);
    addProperty(nearestNeighborInterpolation_);

    addProperties(useCustomDataRange_, customDataRange_, dataRange_);
    dataRange_.setReadOnly(true);
    useCustomDataRange_.onChange(
        [&]() { customDataRange_.setReadOnly(!useCustomDataRange_.get()); });
    customDataRange_.setReadOnly(!useCustomDataRange_.get());

    addProperty(timer_);
    timer_.setReadOnly(true);
}

void TriangulationToVolumeForSequence::process() {

    performanceTimer_.Reset();
    auto seq = std::make_shared<VolumeSequence>();

    auto triangulations = inport_.getData();

    vec2 overallDataRange{std::numeric_limits<double>::max(),
                          std::numeric_limits<double>::lowest()};

    // ToDo: Should this be a PoolProcessor as well?
    for (size_t i = 0; i < triangulations->size(); i++) {
        auto triangulation = inport_.getData()->at(i);
        auto data = topology::ttkTriangulationToVolume(*triangulation.get());
        if (nearestNeighborInterpolation_) data->setInterpolation(InterpolationType::Nearest);
        auto dataRange = data->dataMap_.dataRange;
        auto currentRange = util::volumeMinMax(data.get());
        if (currentRange.first.x < overallDataRange.x) overallDataRange.x = currentRange.first.x;
        if (currentRange.second.x > overallDataRange.y) overallDataRange.y = currentRange.second.x;
        if (useCustomDataRange_.get()) {
            data->dataMap_.dataRange = customDataRange_.get();
            data->dataMap_.valueRange = customDataRange_.get();
        }
        seq->push_back(data);
    }
    dataRange_.set(overallDataRange);

    outport_.setData(seq);
    timer_.set(performanceTimer_.ElapsedTimeAndReset());
}

}  // namespace inviwo
