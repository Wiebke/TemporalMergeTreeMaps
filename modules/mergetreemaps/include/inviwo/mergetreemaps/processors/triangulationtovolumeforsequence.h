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
#include <inviwo/core/properties/boolproperty.h>
#include <inviwo/core/properties/minmaxproperty.h>
#include <inviwo/core/ports/volumeport.h>
#include <inviwo/mergetreemaps/topologysequenceports.h>
#include <inviwo/topologytoolkit/ports/triangulationdataport.h>
#include <inviwo/core/properties/optionproperty.h>
#include <inviwo/core/properties/ordinalproperty.h>
#include <modules/tools/performancetimer.h>

namespace inviwo {

/** \docpage{org.inviwo.TriangulationToVolumeForSequence, Triangulation To Volume For Sequence}
 * ![](org.inviwo.TriangulationToVolumeForSequence.png?classIdentifier=org.inviwo.TriangulationToVolumeForSequence)
 * Converts a sequence of TTK triangulations to a sequence of regular Volumes using the associated
 * scalar values.
 *
 * ### Inports
 *   * __triangulation__ TTK triangulation
 *
 * ### Outports
 *   * __outport__  volume created from triangulation
 *
 * ### Properties
 *   * __useNearestNeighborInterpolation__ Change interpolation type to nearest neighbor.
 */
class IVW_MODULE_MERGETREEMAPS_API TriangulationToVolumeForSequence : public Processor {
public:
    TriangulationToVolumeForSequence();
    virtual ~TriangulationToVolumeForSequence() = default;

    virtual void process() override;

    virtual const ProcessorInfo getProcessorInfo() const override;
    static const ProcessorInfo processorInfo_;

private:
    TriangulationSequenceInport inport_;
    VolumeSequenceOutport outport_;

    BoolProperty nearestNeighborInterpolation_;

    BoolProperty useCustomDataRange_;
    DoubleMinMaxProperty customDataRange_;
    DoubleMinMaxProperty dataRange_;

    /* Performance Timer  */
    kth::PerformanceTimer performanceTimer_;

    /* Property for the timer */
    FloatProperty timer_;
};

}  // namespace inviwo
