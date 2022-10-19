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

#include <inviwo/mergetreemaps/processors/volumetotriangulationforsequence.h>
#include <inviwo/topologytoolkit/utils/ttkutils.h>

#include <inviwo/core/datastructures/buffer/bufferram.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo VolumeToTriangulationForSequence::processorInfo_{
    "org.inviwo.VolumeToTriangulationForSequence",  // Class identifier
    "Volume To Triangulation For Sequence",         // Display name
    "Topology",                                     // Category
    CodeState::Experimental,                        // Code state
    Tags::None,                                     // Tags
};
const ProcessorInfo VolumeToTriangulationForSequence::getProcessorInfo() const {
    return processorInfo_;
}

VolumeToTriangulationForSequence::VolumeToTriangulationForSequence()
    : Processor()
    , inport_("volumes")
    , outport_("outport")
    , usePBC_{"pbc", "Periodic Boundary Conditions", false}
    , channel_("channel", "Channel")
    , timer_("timer", "Eval Time (s)", 0.f, 0.f, std::numeric_limits<float>::max(), 0.001f,
             InvalidationLevel::Valid, PropertySemantics::Text) {

    addPort(inport_);
    addPort(outport_);

    channel_.addOption("Channel 1", "Channel 1", 0);
    channel_.setSerializationMode(PropertySerializationMode::All);
    channel_.setCurrentStateAsDefault();

    addProperties(usePBC_, channel_);

    inport_.onChange([this]() {
        if (inport_.hasData()) {
            // We will assume the channels are the same for every volume
            if (inport_.getData()->size() > 0) {
                size_t channels = inport_.getData()->at(0)->getDataFormat()->getComponents();

                if (channels == channel_.size()) return;

                std::vector<OptionPropertyIntOption> channelOptions;
                for (size_t i = 0; i < channels; i++) {
                    channelOptions.emplace_back("Channel " + toString(i + 1),
                                                "Channel " + toString(i + 1), static_cast<int>(i));
                }
                channel_.replaceOptions(channelOptions);
                channel_.setCurrentStateAsDefault();
            }
        }
    });

    addProperty(timer_);
    timer_.setReadOnly(true);
}

void VolumeToTriangulationForSequence::process() {

    performanceTimer_.Reset();
    auto seq = std::make_shared<TriangulationSequence>();

    auto volumes = inport_.getData();

    // ToDo: Should this be a PoolProcessor as well?
    for (size_t i = 0; i < volumes->size(); i++) {
        auto vol = inport_.getData()->at(i);
        auto data = std::make_shared<topology::TriangulationData>(
            topology::volumeToTTKTriangulation(*vol.get(), static_cast<size_t>(channel_.get())));

        data->getTriangulation().setPeriodicBoundaryConditions(*usePBC_);
        seq->push_back(data);
    }

    outport_.setData(seq);
    timer_.set(performanceTimer_.ElapsedTimeAndReset());
}

}  // namespace inviwo
