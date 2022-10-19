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
#include <inviwo/core/ports/volumeport.h>
#include <inviwo/mergetreemaps/topologysequenceports.h>
#include <inviwo/topologytoolkit/ports/triangulationdataport.h>
#include <inviwo/core/properties/optionproperty.h>
#include <inviwo/core/properties/ordinalproperty.h>
#include <modules/tools/performancetimer.h>

namespace inviwo {

/** \docpage{org.inviwo.VolumeToTriangulationForSequence, Volume To Triangulation For Sequence}
 * ![](org.inviwo.VolumeToTriangulationForSequence.png?classIdentifier=org.inviwo.VolumeToTriangulationForSequence)
 * This processor converts a Volume into an implicit triangulation of a uniform grid used by the
 * Topology ToolKit (TTK).
 *
 * ### Inports
 *   * __volumes__  Sequence of input volumes.
 *
 * ### Outports
 *   * __ouport__   Matching TTK triangulations for each volume
 *
 * ### Properties
 *   * __channel__  channel of input volumes used as scalar data for the triangulation
 *   * __usePBC__  use of periodic boundary conditions
 */

/**
 * \class VolumeToTriangulationForSequence
 * \brief converts a sequence of volumes into a sequence of topology::TriangulationData
 */
class IVW_MODULE_MERGETREEMAPS_API VolumeToTriangulationForSequence : public Processor {
public:
    VolumeToTriangulationForSequence();
    virtual ~VolumeToTriangulationForSequence() = default;

    virtual void process() override;

    virtual const ProcessorInfo getProcessorInfo() const override;
    static const ProcessorInfo processorInfo_;

private:
    VolumeSequenceInport inport_;
    TriangulationSequenceOutport outport_;

    BoolProperty usePBC_;

    OptionPropertyInt channel_;

    /* Performance Timer  */
    kth::PerformanceTimer performanceTimer_;

    /* Property for the timer */
    FloatProperty timer_;
};

}  // namespace inviwo
