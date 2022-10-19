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

#include <modules/tools/volumerepeatertosequence.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo VolumeRepeaterToSequence::processorInfo_{
    "org.inviwo.VolumeRepeaterToSequence",  // Class identifier
    "Volume Repeater To Sequence",          // Display name
    "Volume Operation",                     // Category
    CodeState::Experimental,                // Code state
    Tags::None,                             // Tags
};
const ProcessorInfo VolumeRepeaterToSequence::getProcessorInfo() const { return processorInfo_; }

VolumeRepeaterToSequence::VolumeRepeaterToSequence()
    : Processor()
    , inport_("inport")
    , outport_("outport")
    , numElements_("numElements", "numElements", 32, 1, 1000) {

    addPort(inport_);
    addPort(outport_);
    addProperty(numElements_);
}

void VolumeRepeaterToSequence::process() {

    if (inport_.hasData()) {
        auto input = inport_.getData();
        // Copy input
        auto vol = std::make_shared<Volume>(*input);
        auto sequence = std::make_shared<VolumeSequence>(numElements_, vol);
        outport_.setData(sequence);
    }
}

}  // namespace inviwo
