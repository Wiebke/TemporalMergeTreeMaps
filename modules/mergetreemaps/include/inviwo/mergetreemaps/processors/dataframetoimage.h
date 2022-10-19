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

#pragma once

#include <inviwo/mergetreemaps/mergetreemapsmoduledefine.h>
#include <inviwo/core/processors/processor.h>
#include <inviwo/core/ports/imageport.h>
#include <inviwo/dataframe/datastructures/dataframe.h>
#include <inviwo/core/properties/stringproperty.h>
#include <inviwo/core/properties/ordinalproperty.h>
#include <inviwo/core/properties/boolproperty.h>
#include <inviwo/core/properties/minmaxproperty.h>
#include <inviwo/core/properties/transferfunctionproperty.h>
#include <inviwo/dataframe/properties/dataframeproperty.h>

namespace inviwo {

/** \docpage{org.inviwo.DataFrameToImage, Data Frame To Image}
 * ![](org.inviwo.DataFrameToImage.png?classIdentifier=org.inviwo.DataFrameToImage)
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
class IVW_MODULE_MERGETREEMAPS_API DataFrameToImage : public Processor {
public:
    DataFrameToImage();
    virtual ~DataFrameToImage() = default;

    virtual void process() override;

    virtual const ProcessorInfo getProcessorInfo() const override;
    static const ProcessorInfo processorInfo_;

private:
    DataFrameInport inport_{"dataframe"};
    ImageOutport outport_{"heatmap"};

    IntSize2Property dimensionMultipliers_;
    IntSize2Property dimensions_;

    BoolProperty nearestNeighborInterpolation_;

    // Full dataframe or one column only
    TemplateOptionProperty<int> mode_;

    // Column to select
    DataFrameColumnProperty column_;

    // Pad left and right side with half the size of one value in x
    BoolProperty padLeftRight_;
	// Pad on right and bottom until paddedDimensions is reached
    BoolProperty padUntilAtLeastDims_;
    IntSize2Property paddedDimensions_;
    FloatVec4Property padColor_;

    TransferFunctionProperty transferFunction_;
    BoolProperty useCustomDataRange_;
    DoubleMinMaxProperty customDataRange_;
    DoubleMinMaxProperty dataRange_;
    BoolProperty keepMinRangeAt0_;

    double rangeMin_ = 0.0;
    double rangeMax_ = 1.0;
};

}  // namespace inviwo
