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

/** \docpage{org.inviwo.GenerateDataTargetLevelSet, Generate Data Target Level Set}
 * ![](org.inviwo.GenerateDataTargetLevelSet.png?classIdentifier=org.inviwo.GenerateDataTargetLevelSet)
 * Generates different 2D dataset and displays super- or sub-level set size for a choosen threshold.
 *
 *
 * ### Outports
 *   * __<Outport>__ Generated 2D function in a volume.
 *
 * ### Properties
 *   * __<Prop1>__ <description>.
 *   * __<Prop2>__ <description>
 */
class IVW_MODULE_MERGETREEMAPS_API GenerateDataTargetLevelSet : public Processor {
public:
    GenerateDataTargetLevelSet();
    virtual ~GenerateDataTargetLevelSet() = default;

    virtual void process() override;

    virtual const ProcessorInfo getProcessorInfo() const override;
    static const ProcessorInfo processorInfo_;

    enum class FunctionType {
        Gaussian,
        SkewedNormal,
        DeltaSeq,
        Ackley,
        Ridge,
        Himmelblau,
        Ring,
        Prototype
    };

    enum class ComparisonType { SMALLER, SMALLEREQUAL, GREATEREQUAL, GREATER };

private:
    VolumeOutport outport_;
    VolumeOutport segmentationOutport_;

    // Common properties
    IntSize2Property dimensions_;
    FloatMinMaxProperty xRange_;
    FloatMinMaxProperty yRange_;
    FloatProperty offset_;
    FloatProperty scaleFactor_;
    BoolProperty useCustomDataRange_;
    DoubleMinMaxProperty customDataRange_;
    DoubleMinMaxProperty dataRange_;
    TemplateOptionProperty<FunctionType> function_;

    // Level set properties
    CompositeProperty levelSet_;
    TemplateOptionProperty<ComparisonType> levelSetComparison_;
    FloatProperty levelSetThreshold_;
    IntSizeTProperty levelSetSize_;
    FloatProperty levelSetThreshold2_;
    IntSizeTProperty levelSetSize2_;

    // Gaussian Properties
    CompositeProperty gauss_;
    FloatMat2Property gaussCov_;
    FloatVec2Property gaussMu_;

    // Skewed normal properties
    CompositeProperty skewedNormal_;
    FloatVec2Property skewedNormalShape_;
    FloatMat2Property skewedNormalCov_;
    FloatProperty skewedNormalScale_;
    FloatVec2Property skewedNormalLocation_;

    // Delta sequence property
    CompositeProperty delta_;
    FloatProperty deltaEpsilon_;

    // Ring properties
    CompositeProperty ring_;
    FloatVec2Property ringEpicenter_;
    FloatProperty ringPeak_;
    FloatProperty ringMu_;
    FloatProperty ringStd_;

    // Script generating the different functions
    PythonScriptDisk script_;
};

}  // namespace inviwo
