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
#include <inviwo/mergetreemaps/topologysequenceports.h>
#include <inviwo/dataframe/properties/dataframeproperty.h>
#include <modules/tools/performancetimer.h>

namespace inviwo {

/** \docpage{org.inviwo.MergeTreeMap, Merge Tree Map}
 * ![](org.inviwo.MergeTreeMap.png?classIdentifier=org.inviwo.MergeTreeMap)
 * Map entries of a series of dataframes to an image, each dataframe corresponds to a time step and
 * later a column or row in the image. One of the columns of each dataframe is used as the positions
 * and another column as the value that is mapped to color
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
class IVW_MODULE_MERGETREEMAPS_API MergeTreeMap : public Processor {
public:
    MergeTreeMap();
    virtual ~MergeTreeMap() = default;

    virtual void process() override;

    virtual const ProcessorInfo getProcessorInfo() const override;
    static const ProcessorInfo processorInfo_;

private:
    DataFrameSequenceInport inport_{"inport"};
    DataFrameInport dataFramePort_{"modelDFInport"};
    ImageOutport outport_{"outport"};

    /* Image Dimensions */
    IntProperty maxDataDimSize_;
    IntProperty imageDataDimSize_;
    IntProperty timestepWidth_;
    IntProperty imageTimeDimSize_;
    BoolProperty nearestNeighborInterpolation_;

	/* Indices to exclude */
    BoolProperty restrictPositionRange_;
    IntProperty excludeFirstX_;
    FloatProperty excludeFirstXPercentage_;
    IntProperty excludeLastX_;
    FloatProperty excludeLastXPercentage_;

    /* Direction of time */
    TemplateOptionProperty<int> timeDirection_;

    /* Position Mapping */
    DataFrameColumnProperty positionColumn_;

    /* Color Mapping */
    DataFrameColumnProperty colorColumn_;
    BoolProperty colorColumnContainsColorStrings_;
    TransferFunctionProperty transferFunction_;
    BoolProperty useCustomDataRange_;
    DoubleMinMaxProperty customDataRange_;
    DoubleMinMaxProperty dataRange_;
    TemplateOptionProperty<int> interpolation_;

    /* Performance Timer  */
    kth::PerformanceTimer performanceTimer_;

    /* Property for the timer */
    FloatProperty timer_;

    double rangeMin_ = 0.0;
    double rangeMax_ = 1.0;
};

}  // namespace inviwo
