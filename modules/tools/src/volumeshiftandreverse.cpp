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

#include <modules/tools/volumeshiftandreverse.h>

#include <inviwo/core/util/indexmapper.h>
#include <inviwo/core/util/assertion.h>
#include <inviwo/core/datastructures/volume/volumeram.h>
#include <inviwo/core/datastructures/volume/volumeramprecision.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo VolumeShiftAndReverse::processorInfo_{
    "org.inviwo.VolumeShiftAndReverse",  // Class identifier
    "Volume Shift And Reverse",          // Display name
    "Volume Operation",                  // Category
    CodeState::Experimental,             // Code state
    Tags::CPU,                           // Tags
};
const ProcessorInfo VolumeShiftAndReverse::getProcessorInfo() const { return processorInfo_; }

VolumeShiftAndReverse::VolumeShiftAndReverse()
    : Processor()
    , inport_("volume")
    , outport_("outport")
    , enabled_("enabled", "Enabled", true)
    , shift_("shift", "Shift", vec3(0.0f), vec3(-1.0f), vec3(1.0f))
    , reverseDimX_("reverseDimX", "Reverse X", false)
    , reverseDimY_("reverseDimY", "Reverse Y", false)
    , reverseDimZ_("reverseDimZ", "Reverse Z", false)
    , shiftReverseInfo_("shiftReverseInfo", "Shift and Reverse Info")
    , shiftInDims_("shiftInDims", "Shift in Voxels", ivec3(0), ivec3(-4096), ivec3(4096), ivec3(1),
                   InvalidationLevel::Valid, PropertySemantics::Text)
    , originalBasis_("originalBasis", "Original Basis", mat3(1.0f),
                     util::filled<mat3>(std::numeric_limits<float>::lowest()),
                     util::filled<mat3>(std::numeric_limits<float>::max()),
                     util::filled<mat3>(0.001f), InvalidationLevel::Valid)
    , originalOffset_("originalOffset", "Original Offset", vec3(0.0f),
                      vec3(std::numeric_limits<float>::lowest()),
                      vec3(std::numeric_limits<float>::max()), vec3(0.001f),
                      InvalidationLevel::Valid, PropertySemantics::Text)
    , finalBasis_("finalBasis", "Final Basis", mat3(1.0f),
                  util::filled<mat3>(std::numeric_limits<float>::lowest()),
                  util::filled<mat3>(std::numeric_limits<float>::max()), util::filled<mat3>(0.001f),
                  InvalidationLevel::Valid)
    , finalOffset_("finalOffset", "Final Offset", vec3(0.0f),
                   vec3(std::numeric_limits<float>::lowest()),
                   vec3(std::numeric_limits<float>::max()), vec3(0.001f), InvalidationLevel::Valid,
                   PropertySemantics::Text) {

    addPort(inport_);
    addPort(outport_);
    addProperties(enabled_, shift_, reverseDimX_, reverseDimY_, reverseDimZ_, shiftReverseInfo_);
    shiftReverseInfo_.addProperties(shiftInDims_, originalBasis_, originalOffset_, finalBasis_,
                                    finalOffset_);

    shiftInDims_.setReadOnly(true);
    originalBasis_.setReadOnly(true);
    originalOffset_.setReadOnly(true);
    finalBasis_.setReadOnly(true);
    finalOffset_.setReadOnly(true);
}

void VolumeShiftAndReverse::process() {
    if (!enabled_ || (shift_.get() == vec3(0.0f) && !reverseDimX_.get() && !reverseDimY_.get() &&
                      !reverseDimZ_.get())) {
        outport_.setData(inport_.getData());
        return;
    }

    auto volume = inport_.getData();

    const auto shift = ivec3(shift_.get() * vec3(inport_.getData()->getDimensions()));
    shiftInDims_.set(shift);

    auto volumeRam = volume->getRepresentation<VolumeRAM>()->dispatch<std::shared_ptr<VolumeRAM>>(
        [&shift, reverseDimX = reverseDimX_.get(), reverseDimY = reverseDimY_.get(),
         reverseDimZ = reverseDimZ_.get()](auto vr) {
            using ValueType = util::PrecisionValueType<decltype(vr)>;

            const auto src = vr->getDataTyped();
            const auto dim = ivec3(vr->getDimensions());
            const int size = glm::compMul(dim);
            // Maps between 1d and 3d index through i = x + y * dimx + z * dimxy;
            util::IndexMapper<3, int> im(dim);

            auto vol = std::make_shared<VolumeRAMPrecision<ValueType>>(
                vr->getDimensions(), vr->getSwizzleMask(), vr->getInterpolation(),
                vr->getWrapping());
            auto dst = vol->getDataTyped();
            for (int i = 0; i < size; ++i) {
                // Returns a linear index shifted index
                auto dstIndex = VolumeRAM::periodicPosToIndex(im(i) + shift, dim);
                auto shifted3d = im(dstIndex);
                // Map 0 to max index and vice verse
                if (reverseDimX) shifted3d.x = dim.x - 1 - shifted3d.x;
                if (reverseDimY) shifted3d.y = dim.y - 1 - shifted3d.y;
                if (reverseDimZ) shifted3d.z = dim.z - 1 - shifted3d.z;
                dstIndex = im(shifted3d);
                IVW_ASSERT(dstIndex < size, "invalid voxel index");
                dst[dstIndex] = src[i];
            }
            return vol;
        });

    auto finalVol = std::make_shared<Volume>(volumeRam);
    finalVol->copyMetaDataFrom(*(volume));
    finalVol->dataMap_ = volume->dataMap_;
    auto oldBasis = volume->getBasis();
    auto oldOffset = volume->getOffset();
    originalBasis_.set(oldBasis);
    originalOffset_.set(oldOffset);
    const vec3 volExtent(glm::length(oldBasis[0]), glm::length(oldBasis[1]),
                         glm::length(oldBasis[2]));
    const auto dim = ivec3(volume->getDimensions());
    // Basis stays the same when shifting, but offset neets to be recomputed
    // Coordinate transformer centers data on voxels, leading to unintended offsets when
    // understanding data as being on the vertices
    // const auto indexToModel = volume->getCoordinateTransformer().getIndexToModelMatrix();
    mat4 indexMatrix(1.0f);
    for (unsigned int i = 0; i < 3; ++i) {
        indexMatrix[i][i] = static_cast<float>(dim[i] - 1);
    }
    mat4 indexToModel = volume->getModelMatrix() * MatrixInvert(indexMatrix);
    auto oldIndexOfNewZero = -shift;
    if (oldIndexOfNewZero.x < 0) oldIndexOfNewZero.x += dim.x;
    if (oldIndexOfNewZero.y < 0) oldIndexOfNewZero.y += dim.y;
    if (oldIndexOfNewZero.z < 0) oldIndexOfNewZero.z += dim.z;
    const dvec3 pos{indexToModel * dvec4{oldIndexOfNewZero, 1}};
    auto newOffset = pos;
    // Basis vectors get flipped when reversing, and offset at the min is now the max
    if (reverseDimX_.get()) {
        // Sign of adding the offset is determined by the basis vector being aligned or not aligned
        // with the coordinate axis
        // For an axis aligned basis this results in +1 if the basis points positive direction of
        // the x-axsis and -1 if it points in negative x-direction
        newOffset.x = newOffset.x +
                      glm::dot(glm::normalize(oldBasis[0]), vec3(1.0f, 0.0f, 0.0f)) * volExtent.x;
        oldBasis[0] = -oldBasis[0];
    }
    if (reverseDimY_.get()) {
        newOffset.y = newOffset.y +
                      glm::dot(glm::normalize(oldBasis[1]), vec3(0.0f, 1.0f, 0.0f)) * volExtent.y;
        oldBasis[1] = -oldBasis[1];
    }
    if (reverseDimZ_.get()) {
        newOffset.z = newOffset.z +
                      glm::dot(glm::normalize(oldBasis[2]), vec3(0.0f, 0.0f, 1.0f)) * volExtent.z;
        oldBasis[2] = -oldBasis[2];
    }
    finalVol->setBasis(oldBasis);
    finalVol->setOffset(newOffset);
    finalBasis_.set(finalVol->getBasis());
    finalOffset_.set(finalVol->getOffset());
    finalVol->setWorldMatrix(volume->getWorldMatrix());

    outport_.setData(finalVol);
}

}  // namespace inviwo
