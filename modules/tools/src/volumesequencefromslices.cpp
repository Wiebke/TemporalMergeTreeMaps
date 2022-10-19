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

#include <modules/tools/volumesequencefromslices.h>

namespace inviwo {

namespace kth {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo VolumeSequenceFromSlices::processorInfo_{
    "org.inviwo.VolumeSequenceFromSlices",  // Class identifier
    "Volume Sequence From Slices",          // Display name
    "Volume Operation",                     // Category
    CodeState::Experimental,                // Code state
    Tags::None,                             // Tags
};
const ProcessorInfo VolumeSequenceFromSlices::getProcessorInfo() const { return processorInfo_; }

VolumeSequenceFromSlices::VolumeSequenceFromSlices()
    : Processor()
    , inport_("volumeInport")
    , outport_("sequenceOutport")
    , propSliceAlongAxis("sliceAxis", "Slice along axis") {

    addPort(inport_);
    addPort(outport_);

    // Additional property for the slicing axis
    propSliceAlongAxis.addOption("x", "y-z plane (X axis)", CartesianCoordinateAxis::X);
    propSliceAlongAxis.addOption("y", "z-x plane (Y axis)", CartesianCoordinateAxis::Y);
    propSliceAlongAxis.addOption("z", "x-y plane (Z axis)", CartesianCoordinateAxis::Z);
    propSliceAlongAxis.set(CartesianCoordinateAxis::Z);
    propSliceAlongAxis.setCurrentStateAsDefault();
    addProperty(propSliceAlongAxis);
}

void VolumeSequenceFromSlices::process() {

    if (!inport_.hasData()) return;

    size_t numSlices;
    const size3_t dims{inport_.getData()->getDimensions()};
    size3_t dimsSlice;
    size3_t offsetInc;

    switch (propSliceAlongAxis) {
        case CartesianCoordinateAxis::X:
            numSlices = dims.x;
            dimsSlice = size3_t(1, dims.y, dims.z);
            offsetInc = size3_t(1, 0, 0);
            break;
        case CartesianCoordinateAxis::Y:
            numSlices = dims.y;
            dimsSlice = size3_t(dims.x, 1, dims.z);
            offsetInc = size3_t(0, 1, 0);
            break;
        case CartesianCoordinateAxis::Z:
            numSlices = dims.z;
            dimsSlice = size3_t(dims.x, dims.y, 1);
            offsetInc = size3_t(0, 0, 1);
            break;
        default:
            numSlices = dims.z;
            dimsSlice = size3_t(dims.x, dims.y, 1);
            offsetInc = size3_t(0, 0, 1);
            break;
    }

    size3_t offset = size3_t(0, 0, 0);

    auto volSeq = std::make_shared<VolumeSequence>();

    const auto& volume = inport_.getData();
    const VolumeRAM* volumeRAM = inport_.getData()->getRepresentation<VolumeRAM>();

    for (size_t idx = 0; idx < numSlices; idx++, offset += offsetInc) {

        std::shared_ptr<Volume> pVolume =
            std::make_shared<Volume>(Volume(VolumeRAMSubSet::apply(volumeRAM, dimsSlice, offset)));
        // Pass meta data on
        pVolume->copyMetaDataFrom(*volume);
        pVolume->dataMap_ = volume->dataMap_;
        pVolume->setInterpolation(volume->getInterpolation());

        // Adapt basis and offset
        vec3 volOffset = volume->getOffset();
        mat3 volBasis = volume->getBasis();

        const vec3 newOffset =
            volOffset + volBasis * (static_cast<vec3>(offset) / static_cast<vec3>(dims));

        mat3 newBasis = volBasis;
        vec3 dimRatio = (static_cast<vec3>(dimsSlice) / static_cast<vec3>(dims));
        newBasis[0] *= dimRatio[0];
        newBasis[1] *= dimRatio[1];
        newBasis[2] *= dimRatio[2];

        pVolume->setBasis(newBasis);
        pVolume->setOffset(newOffset);

        volSeq->push_back(pVolume);
    }

    outport_.setData(volSeq);
}

}  // namespace kth

}  // namespace inviwo
