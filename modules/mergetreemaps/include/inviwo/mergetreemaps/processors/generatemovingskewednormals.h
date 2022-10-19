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

#pragma once

#include <inviwo/mergetreemaps/mergetreemapsmoduledefine.h>
#include <inviwo/core/processors/processor.h>
#include <inviwo/core/properties/ordinalproperty.h>
#include <inviwo/core/properties/boolproperty.h>
#include <inviwo/core/properties/compositeproperty.h>
#include <inviwo/core/properties/stringproperty.h>
#include <inviwo/core/properties/minmaxproperty.h>
#include <inviwo/core/properties/listproperty.h>
#include <inviwo/core/datastructures/volume/volume.h>
#include <inviwo/core/ports/volumeport.h>
#include <modules/python3/pythonscript.h>
#include <pybind11/pybind11.h>

namespace inviwo {

/**
 * \ingroup properties
 * A CompositeProperty holding information for a skewed normal distribution
 */
class IVW_MODULE_MERGETREEMAPS_API SkewedNormalProperty : public CompositeProperty {
public:
    virtual std::string getClassIdentifier() const override;
    static const std::string classIdentifier;
    SkewedNormalProperty(std::string identifier, std::string displayName,
                         InvalidationLevel invalidationLevel = InvalidationLevel::InvalidResources,
                         PropertySemantics semantics = PropertySemantics::Default);
    SkewedNormalProperty(const SkewedNormalProperty& rhs);
    virtual SkewedNormalProperty* clone() const override;
    virtual ~SkewedNormalProperty() = default;

    FloatVec2Property shape_;
    FloatMat2Property cov_;
    FloatProperty scale_;
    FloatVec2Property location_;

    FloatProperty valueMin_;
    FloatProperty valueMax_;

private:
    auto props();
};

/**
 * \ingroup properties
 * A CompositeProperty holding information for a skewed normal distribution
 */
class IVW_MODULE_MERGETREEMAPS_API SkewedNormalTrackProperty : public CompositeProperty {
public:
    virtual std::string getClassIdentifier() const override;
    static const std::string classIdentifier;
    SkewedNormalTrackProperty(
        std::string identifier, std::string displayName,
        InvalidationLevel invalidationLevel = InvalidationLevel::InvalidResources,
        PropertySemantics semantics = PropertySemantics::Default);
    SkewedNormalTrackProperty(const SkewedNormalTrackProperty& rhs);
    virtual SkewedNormalTrackProperty* clone() const override;
    virtual ~SkewedNormalTrackProperty() = default;

    IntSizeTProperty timestepBegin_;
    SkewedNormalProperty featureBegin_;
    IntSizeTProperty timestepEnd_;
    SkewedNormalProperty featureEnd_;

private:
    auto props();
};

/** \docpage{org.inviwo.GenerateMovingSkewedNormals, Generate Moving Skewed Normals}
 * ![](org.inviwo.GenerateMovingSkewedNormals.png?classIdentifier=org.inviwo.GenerateMovingSkewedNormals)
 * Explanation of how to use the processor.
 *
 * ### Inports
 *   * __<Inport1>__ <description>.
 *
 * ### Outports
 *   * __<Outport1>__ <description>.
 *
 * ### Properties
 *   * __<Prop1>__ <description>.
 *   * __<Prop2>__ <description>
 */
class IVW_MODULE_MERGETREEMAPS_API GenerateMovingSkewedNormals : public Processor,
                                                                 public PropertyOwnerObserver {
public:
    GenerateMovingSkewedNormals();
    virtual ~GenerateMovingSkewedNormals() = default;

    virtual void process() override;

    virtual const ProcessorInfo getProcessorInfo() const override;
    static const ProcessorInfo processorInfo_;

    virtual void onDidAddProperty(Property* property, size_t index) override;
    virtual void onDidRemoveProperty(Property* property, size_t index) override;

private:
    VolumeOutport outport_;

    IntSize2Property dimensions_;
    FloatMinMaxProperty xRange_;
    FloatMinMaxProperty yRange_;
    IntSizeTProperty timesteps_;
    BoolProperty scaleTimes_;
    size_t prevNumTimesteps_ = std::numeric_limits<size_t>::max();

    ListProperty skewedNormals_;

    // Script generating a single skewed normal pdf
    PythonScriptDisk script_;
};

}  // namespace inviwo
