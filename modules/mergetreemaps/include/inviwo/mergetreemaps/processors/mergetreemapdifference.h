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
#include <inviwo/dataframe/datastructures/dataframe.h>
#include <inviwo/core/properties/boolproperty.h>
#include <inviwo/core/properties/ordinalproperty.h>
#include <inviwo/mergetreemaps/topologysequenceports.h>
#include <inviwo/dataframe/properties/dataframeproperty.h>
#include <modules/tools/performancetimer.h>

namespace inviwo {

/** \docpage{org.inviwo.MergeTreeMapDifference, Merge Tree Map Difference}
 * ![](org.inviwo.MergeTreeMapDifference.png?classIdentifier=org.inviwo.MergeTreeMapDifference)
 * Computes a difference map between two dataframe series
 *
 * ### Inports
 *   * __<Inport1>__ <description>.
 *   * __<Inport1>__ <description>.
 *
 * ### Outports
 *   * __<Outport1>__ <description>.
 *
 * ### Properties
 *   * __<Prop1>__ <description>.
 *   * __<Prop2>__ <description>
 */
class IVW_MODULE_MERGETREEMAPS_API MergeTreeMapDifference : public Processor {
public:
    MergeTreeMapDifference();
    virtual ~MergeTreeMapDifference() = default;

    virtual void process() override;

    virtual const ProcessorInfo getProcessorInfo() const override;
    static const ProcessorInfo processorInfo_;

private:
    DataFrameSequenceInport inport_{"inport"};
    DataFrameInport dataFramePort_{"modelDFInport"};
    DataFrameSequenceInport secondInport_{"inportsecond"};

    DataFrameSequenceOutport outport_{"matrices"};

    /* Position Mapping */
    DataFrameColumnProperty positionColumn_;

    /* Color Mapping */
    DataFrameColumnProperty valueColumn_;

    /* Output mode for output (absolute, squared, normalized to datarange, ...)*/
    TemplateOptionProperty<int> outputMode_;

    /* Performance Timer  */
    kth::PerformanceTimer performanceTimer_;

    /* Property for the timer */
    FloatProperty timer_;
};

}  // namespace inviwo
