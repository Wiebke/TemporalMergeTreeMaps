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

#include <modules/tools/volumefromslicesequence.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo VolumeFromSliceSequence::processorInfo_{
    "org.inviwo.VolumeFromSliceSequence",  // Class identifier
    "Volume From Slice Sequence",          // Display name
    "Volume Operation",                    // Category
    CodeState::Experimental,               // Code state
    Tags::None,                            // Tags
};
const ProcessorInfo VolumeFromSliceSequence::getProcessorInfo() const { return processorInfo_; }

VolumeFromSliceSequence::VolumeFromSliceSequence()
    : Processor(), volumesIn_("volsIn"), volumeOut_("volOut") {

    addPort(volumesIn_);
    addPort(volumeOut_);
}

void VolumeFromSliceSequence::process() {
    auto vols = volumesIn_.getData();

    if (!vols) {
        return;
    }

    auto numVolumes = vols->size();

    // test for consistency of dimensions of input volumes

    auto vol0 = vols->at(0);
    auto resDims = vol0->getDimensions();

    for (auto i = 1; i < numVolumes; i++) {

        auto vol = vols->at(i);
        auto dims = vol->getDimensions();

        if (dims.x != resDims.x || dims.y != resDims.y) {
            LogWarn("Wrong Dimensions in XY!");
            return;
        } else {
            resDims.z += dims.z;
        }
    }

    std::shared_ptr<Volume> volume_ = std::make_shared<Volume>(resDims, DataFloat32::get());

    auto drange = vec2(std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest());
    auto vrange = vec2(std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest());

    size_t index = 0;
    auto data = static_cast<float*>(volume_->getEditableRepresentation<VolumeRAM>()->getData());

    for (size_t i = 0; i < numVolumes; i++) {

        auto vol = vols->at(i);
        auto dims = vol->getDimensions();

        const VolumeRAM* vr = vol->getRepresentation<VolumeRAM>();

        drange[0] = std::min(drange[0], static_cast<float>(vol->dataMap_.dataRange[0]));
        drange[1] = std::max(drange[1], static_cast<float>(vol->dataMap_.dataRange[1]));
        vrange[0] = std::min(vrange[0], static_cast<float>(vol->dataMap_.valueRange[0]));
        vrange[1] = std::max(vrange[1], static_cast<float>(vol->dataMap_.valueRange[1]));

        size3_t idx;

        for (idx.z = 0; idx.z < dims.z; idx.z++) {
            for (idx.y = 0; idx.y < dims.y; idx.y++) {
                for (idx.x = 0; idx.x < dims.x; idx.x++) {
                    data[index++] = float(vr->getAsDouble(idx));
                }
            }
        }
    }

    // Adapt basis and offset
    vec3 volOffset = vol0->getOffset();
    mat3 volBasis = vol0->getBasis();

    mat3 newBasis = volBasis;
    newBasis[2] *= resDims.z;

    volume_->setBasis(newBasis);
    volume_->setOffset(volOffset);

    volume_->dataMap_.dataRange = drange;
    volume_->dataMap_.valueRange = vrange;

    volume_->setInterpolation(vol0->getInterpolation());
    volume_->setSwizzleMask(vol0->getSwizzleMask());

    volumeOut_.setData(volume_);
}

}  // namespace inviwo
