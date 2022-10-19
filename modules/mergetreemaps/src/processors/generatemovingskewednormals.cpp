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

#include <inviwo/mergetreemaps/processors/generatemovingskewednormals.h>
#include <inviwo/mergetreemaps/mergetreemapsmodule.h>
#include <inviwo/core/util/filesystem.h>
#include <modules/base/algorithm/dataminmax.h>

namespace inviwo {

const std::string SkewedNormalProperty::classIdentifier = "org.inviwo.SkewedNormalProperty";
std::string SkewedNormalProperty::getClassIdentifier() const { return classIdentifier; }

auto SkewedNormalProperty::props() {
    return std::tie(shape_, cov_, scale_, location_, valueMin_, valueMax_);
}

SkewedNormalProperty::SkewedNormalProperty(std::string identifier, std::string displayName,
                                           InvalidationLevel invalidationLevel,
                                           PropertySemantics semantics)
    : CompositeProperty(identifier, displayName, invalidationLevel, semantics)
    , shape_("shape", "Shape (alpha)", vec2(0, 0), vec2(-10, -10), vec2(10, 10))
    , cov_("cov", "Covariance", mat2(1, 0, 0, 1), mat2(0), mat2(10), mat2(0.01),
           InvalidationLevel::InvalidOutput, PropertySemantics::Text)
    , scale_("scale", "Scale (omega)", 1.f, 0.00001f, 10.f)
    , location_("loc", "Location (xi)", vec2(0, 0), vec2(-10, -10), vec2(10, 10))
    , valueMin_("valueMin", "Mapped to Min", 0, -10, 10)
    , valueMax_("valueMax", "Mapped to Max", 1, -10, 10) {

    util::for_each_in_tuple(
        [&](auto& e) {
            e.setSerializationMode(PropertySerializationMode::All);
            this->addProperty(e);
        },
        props());
}

SkewedNormalProperty::SkewedNormalProperty(const SkewedNormalProperty& rhs)
    : CompositeProperty(rhs)
    , shape_(rhs.shape_)
    , cov_(rhs.cov_)
    , scale_(rhs.scale_)
    , location_(rhs.location_)
    , valueMin_(rhs.valueMin_)
    , valueMax_(rhs.valueMax_) {

    util::for_each_in_tuple([&](auto& e) { this->addProperty(e); }, props());
}

SkewedNormalProperty* SkewedNormalProperty::clone() const {
    return new SkewedNormalProperty(*this);
}

const std::string SkewedNormalTrackProperty::classIdentifier =
    "org.inviwo.SkewedNormalTrackProperty";
std::string SkewedNormalTrackProperty::getClassIdentifier() const { return classIdentifier; }

auto SkewedNormalTrackProperty::props() {
    return std::tie(timestepBegin_, featureBegin_, timestepEnd_, featureEnd_);
}

SkewedNormalTrackProperty::SkewedNormalTrackProperty(std::string identifier,
                                                     std::string displayName,
                                                     InvalidationLevel invalidationLevel,
                                                     PropertySemantics semantics)
    : CompositeProperty(identifier, displayName, invalidationLevel, semantics)
    , timestepBegin_("timestepBegin", "Time Step Begin", 0, 0, 512)
    , featureBegin_("featureBegin", "Feature Begin")
    , timestepEnd_("timestepEnd", "Time Step End", 16, 0, 512)
    , featureEnd_("featureEnd", "Feature End") {
    addProperties(timestepBegin_, featureBegin_, timestepEnd_, featureEnd_);
}

SkewedNormalTrackProperty::SkewedNormalTrackProperty(const SkewedNormalTrackProperty& rhs)
    : CompositeProperty(rhs)
    , timestepBegin_(rhs.timestepBegin_)
    , featureBegin_(rhs.featureBegin_)
    , timestepEnd_(rhs.timestepEnd_)
    , featureEnd_(rhs.featureEnd_) {
    addProperties(timestepBegin_, featureBegin_, timestepEnd_, featureEnd_);
}

SkewedNormalTrackProperty* SkewedNormalTrackProperty::clone() const {
    return new SkewedNormalTrackProperty(*this);
}

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo GenerateMovingSkewedNormals::processorInfo_{
    "org.inviwo.GenerateMovingSkewedNormals",  // Class identifier
    "Generate Moving Skewed Normals",          // Display name
    "Data Generation",                         // Category
    CodeState::Experimental,                   // Code state
    Tags::None,                                // Tags
};
const ProcessorInfo GenerateMovingSkewedNormals::getProcessorInfo() const { return processorInfo_; }

GenerateMovingSkewedNormals::GenerateMovingSkewedNormals()
    : Processor()
    , outport_("outport")
    , dimensions_("dims", "Dimensions", size2_t(128), size2_t(1), size2_t(1024))
    , xRange_("xRange", "X Range", -2, 2, -10, 10)
    , yRange_("yRange", "Y Range", -2, 2, -10, 10)
    , timesteps_("timesteps", "Time Steps", 16, 1, 512)
    , scaleTimes_("scaleTime", "Scale Times", true)
    , skewedNormals_(
          "features", "Features",
          std::make_unique<SkewedNormalTrackProperty>("skewedNormal", "Skewed Normal Feature"))
    , script_(InviwoApplication::getPtr()->getModuleByType<MergeTreeMapsModule>()->getPath(
                  ModulePath::Scripts) +
              "/skewedNormal2D.py") {
    addPort(outport_);

    addProperties(dimensions_, xRange_, yRange_, timesteps_, scaleTimes_, skewedNormals_);

    timesteps_.onChange([&]() {
        auto newNumTimesteps = timesteps_.get();
        for (auto p : skewedNormals_) {
            if (auto feature = dynamic_cast<SkewedNormalTrackProperty*>(p)) {
                auto timestepBeginProp = dynamic_cast<IntSizeTProperty*>(
                    feature->getPropertyByIdentifier("timestepBegin"));
                auto timestepEndProp = dynamic_cast<IntSizeTProperty*>(
                    feature->getPropertyByIdentifier("timestepEnd"));
                if (scaleTimes_.get() && prevNumTimesteps_ < std::numeric_limits<size_t>::max()) {
                    auto scale = newNumTimesteps / static_cast<float>(prevNumTimesteps_);
                    if (scale > 1) {
                        timestepBeginProp->setMaxValue(newNumTimesteps - 1);
                        timestepEndProp->setMaxValue(newNumTimesteps - 1);
                    }
                    // Map from index to number, then scale, then map back to index
                    // Guard against setting size_t to negtive number (will lead to underflow)
                    int newBeginning =
                        round(scale * static_cast<int>(timestepBeginProp->get() + 1)) - 1;
                    timestepBeginProp->set(std::max(newBeginning, 0));
                    int newEnd = round(scale * static_cast<int>(timestepEndProp->get() + 1)) - 1;
                    timestepEndProp->set(std::max(newEnd, 0));
                }
                timestepBeginProp->setMaxValue(newNumTimesteps - 1);
                timestepEndProp->setMaxValue(newNumTimesteps - 1);
            }
        }
        prevNumTimesteps_ = timesteps_;
    });

    skewedNormals_.PropertyOwnerObservable::addObserver(this);
}

void GenerateMovingSkewedNormals::process() {
    auto dims = dimensions_.get();
    auto sizeX = dims.x;
    auto sizeY = dims.y;
    auto vol =
        std::make_shared<Volume>(size3_t(sizeX, sizeY, timesteps_.get()), DataFloat32::get());
    auto vr = vol->getEditableRepresentation<VolumeRAM>();

    for (auto p : skewedNormals_) {
        if (auto feature = dynamic_cast<SkewedNormalTrackProperty*>(p)) {

            auto featureBegin = dynamic_cast<SkewedNormalProperty*>(
                feature->getPropertyByIdentifier("featureBegin"));
            auto timestepBegin =
                dynamic_cast<IntSizeTProperty*>(feature->getPropertyByIdentifier("timestepBegin"))
                    ->get();
            auto featureEnd =
                dynamic_cast<SkewedNormalProperty*>(feature->getPropertyByIdentifier("featureEnd"));
            auto timestepEnd =
                dynamic_cast<IntSizeTProperty*>(feature->getPropertyByIdentifier("timestepEnd"))
                    ->get();

            // Interpolate between end and beginning
            for (size_t timestep = timestepBegin;
                 timestep <= std::min(timesteps_.get() - 1, timestepEnd); timestep++) {

                auto locals = pybind11::globals();
                // Input
                // Needs to specify:
                // size_x, size_y,
                // range_x, range_y,
                // shape, cov, loc, scale,
                // valueMin, valueMax
                locals["size_x"] = pybind11::cast(sizeX);
                locals["size_y"] = pybind11::cast(sizeY);
                locals["range_x"] = pybind11::cast(xRange_.get());
                locals["range_y"] = pybind11::cast(yRange_.get());
                float t =
                    (timestep - timestepBegin) / static_cast<float>(timestepEnd - timestepBegin);

                locals["shape"] = pybind11::cast(
                    glm::lerp(featureBegin->shape_.get(), featureEnd->shape_.get(), t));
                // Interpolation between matrices could likely take semantics of it being a
                // covariance into account
                locals["cov"] =
                    pybind11::cast(featureBegin->cov_.get() * (1 - t) + featureEnd->cov_.get() * t);
                locals["loc"] = pybind11::cast(
                    glm::lerp(featureBegin->location_.get(), featureEnd->location_.get(), t));
                locals["scale"] = pybind11::cast(
                    glm::lerp(featureBegin->scale_.get(), featureEnd->scale_.get(), t));
                locals["valueMin"] = pybind11::cast(
                    glm::lerp(featureBegin->valueMin_.get(), featureEnd->valueMin_.get(), t));
                locals["valueMax"] = pybind11::cast(
                    glm::lerp(featureBegin->valueMax_.get(), featureEnd->valueMax_.get(), t));

                try {
                    script_.run(locals, [&](pybind11::dict dict) {
                        auto currVolume = pybind11::cast<Volume>(dict["volume"]);
                        auto currVr = currVolume.getRepresentation<VolumeRAM>();
                        size3_t idx(0);
                        for (idx.y = 0; idx.y < dims.y; idx.y++) {
                            for (idx.x = 0; idx.x < dims.x; idx.x++) {
                                auto newValue = currVr->getAsDouble(idx);
                                auto idxInTime = size3_t(idx.x, idx.y, timestep);
                                auto currValue = vr->getAsDouble(idxInTime) + newValue;
                                vr->setFromDouble(idxInTime, currValue);
                            }
                        }
                    });
                } catch (std::exception& e) {
                    LogError(e.what())
                }
            }
        }
    }

    auto range = util::volumeMinMax(vr);
    vol->dataMap_.dataRange = vec2(range.first.x, range.second.x);
    vol->dataMap_.valueRange = vec2(range.first.x, range.second.x);

    outport_.setData(vol);
}

void GenerateMovingSkewedNormals::onDidAddProperty(Property* property, size_t index) {
    skewedNormals_.setModified();
    this->invalidate(InvalidationLevel::InvalidOutput);
}

void GenerateMovingSkewedNormals::onDidRemoveProperty(Property* property, size_t index) {
    skewedNormals_.setModified();
    this->invalidate(InvalidationLevel::InvalidOutput);
}

}  // namespace inviwo
