/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2022 Inviwo Foundation
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

#include <modules/tools/volumesequenceclampvaluerange.h>
#include <modules/base/algorithm/dataminmax.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo VolumeSequenceClampValueRange::processorInfo_{
    "org.inviwo.VolumeSequenceClampValueRange",  // Class identifier
    "Volume Sequence Clamp Value Range",         // Display name
    "Undefined",                                 // Category
    CodeState::Experimental,                     // Code state
    Tags::None,                                  // Tags
};
const ProcessorInfo VolumeSequenceClampValueRange::getProcessorInfo() const {
    return processorInfo_;
}

VolumeSequenceClampValueRange::VolumeSequenceClampValueRange()
    : Processor()
    , volumesInPort_("inport")
    , volumesOutPort_("outport")
    , actualDataRange_("dataRange", "Actual Range", 0.0, 1.0, std::numeric_limits<double>::lowest(),
                       std::numeric_limits<double>::max(), 0.01, 0.0, InvalidationLevel::Valid,
                       PropertySemantics::Text)
    , targetdataRange_("targetRange", "Target Range", 0.0, 1.0,
                       std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max(),
                       0.01, 0.0, InvalidationLevel::InvalidOutput, PropertySemantics::Text)
    , createMask_("createMask", "Create Mask for Value", false)
    , maskOutValue_("maskOutValue", "Value to Mask", 0.0, std::numeric_limits<double>::lowest(),
                    std::numeric_limits<double>::max(), 1e-10, InvalidationLevel::InvalidOutput,
                    PropertySemantics::Text)
    , maskOutEps_("maskOutEps", "Eps around Mask Value", 0.0, std::numeric_limits<double>::lowest(),
                  std::numeric_limits<double>::max(), 1e-10, InvalidationLevel::InvalidOutput,
                  PropertySemantics::Text)
    , maskOutNewValue_("maskOutNewValue", "New Value", 0.0, std::numeric_limits<double>::lowest(),
                       std::numeric_limits<double>::max(), 1e-10, InvalidationLevel::InvalidOutput,
                       PropertySemantics::Text) {

    addPort(volumesInPort_);
    addPort(volumesOutPort_);
    addProperties(actualDataRange_, targetdataRange_, createMask_, maskOutValue_, maskOutEps_,
                  maskOutNewValue_);

    actualDataRange_.setReadOnly(true);

    maskOutValue_.visibilityDependsOn(createMask_, [](const auto& p) { return p.get(); });
    maskOutEps_.visibilityDependsOn(createMask_, [](const auto& p) { return p.get(); });
    maskOutNewValue_.visibilityDependsOn(createMask_, [](const auto& p) { return p.get(); });

    volumesInPort_.onChange([&]() {
        auto vols = volumesInPort_.getData();
        if (!vols) return;
        auto numVolumes = vols->size();
        if (numVolumes < 1) return;
        dvec2 overallRange{std::numeric_limits<double>::max(),
                           std::numeric_limits<double>::lowest()};
        for (size_t i = 0; i < numVolumes; i++) {
            const VolumeRAM* vrIn = vols->at(i)->getRepresentation<VolumeRAM>();
            auto currentRange = util::volumeMinMax(vrIn);
            if (currentRange.first.x < overallRange.x) overallRange.x = currentRange.first.x;
            if (currentRange.second.x > overallRange.y) overallRange.y = currentRange.second.x;
        }
        actualDataRange_.set(overallRange);
    });
}

void VolumeSequenceClampValueRange::process() {
    auto vols = volumesInPort_.getData();
    if (!vols) return;
    auto numVolumes = vols->size();
    if (numVolumes < 1) return;

    auto vol = std::make_shared<Volume>(*vols->at(0));
    auto sequence = std::make_shared<VolumeSequence>(numVolumes);
    auto targetRange = targetdataRange_.get();

    bool createMask = createMask_.get();
    auto maskValue = maskOutValue_.get();
    auto maskEps = maskOutEps_.get();
    auto maskNewValue = maskOutNewValue_.get();

    for (size_t i = 0; i < numVolumes; i++) {
        auto currentInVol = vols->at(i);
        size3_t dims = currentInVol->getDimensions();
        const VolumeRAM* vrIn = currentInVol->getRepresentation<VolumeRAM>();
        // create volume and set basis and offset
        auto vrOut = createVolumeRAM(dims, DataFloat32::get());
        auto currentOutVol = std::make_shared<Volume>(vrOut);
        currentOutVol->setModelMatrix(currentInVol->getModelMatrix());
        currentOutVol->setWorldMatrix(currentInVol->getWorldMatrix());
        currentOutVol->copyMetaDataFrom(*currentInVol);
        sequence->at(i) = currentOutVol;

        size3_t idx;
        for (idx.z = 0; idx.z < dims.z; idx.z++) {
            for (idx.y = 0; idx.y < dims.y; idx.y++) {
                for (idx.x = 0; idx.x < dims.x; idx.x++) {
                    double value = vrIn->getAsDouble(idx);
                    if (createMask) {
                        if (abs(value - maskValue) < maskEps) {
                            value = maskNewValue;
                        }
                    }
                    if (value < targetRange.x) value = targetRange.x;
                    if (value > targetRange.y) value = targetRange.y;
                    vrOut->setFromDouble(idx, value);
                }
            }
        }

        currentOutVol->dataMap_.dataRange = targetdataRange_;
        currentOutVol->dataMap_.valueRange = targetdataRange_;
    }
    volumesOutPort_.setData(sequence);
}

}  // namespace inviwo
