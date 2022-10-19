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
#include <inviwo/core/processors/poolprocessor.h>
#include <inviwo/core/properties/boolproperty.h>
#include <inviwo/core/properties/ordinalproperty.h>
#include <inviwo/core/properties/optionproperty.h>
#include <inviwo/core/ports/volumeport.h>
#include <inviwo/topologytoolkit/ports/triangulationdataport.h>
#include <inviwo/mergetreemaps/topologysequenceports.h>
#include <modules/tools/performancetimer.h>

namespace inviwo {

/** \docpage{org.inviwo.TopologicalSimplificationForSequence, Topological Simplification For Sequence}
 * ![](org.inviwo.TopologicalSimplificationForSequence.png?classIdentifier=org.inviwo.TopologicalSimplificationForSequence)
 * Removes critical points that have a persistence below or above the given threshold or keeps a specified number of points
 * Used in conjunction with PersistenceDiagram.
 *
 * ### Inports
 *   * __triangulation__   input triangulation. Its scalars are ignored, only the connectivity is important.
 *   * __scalars__         input scalars  
 *   * __persistance__     Series of matching persistence diagram
 *
 * ### Outports
 *   * __simplified__   output scalars with critical points below/above threshold removed
 *
 * ### Properties
 *   * __Mode__         Remove below persistance threshold, remove above threshold, keep number of most peristent
 *   * __Keep Points__  Number of of critical points to keep
 *   * __Threshold__    persistence threshold

 */
class IVW_MODULE_MERGETREEMAPS_API TopologicalSimplificationForSequence : public PoolProcessor {
public:
    TopologicalSimplificationForSequence();
    virtual ~TopologicalSimplificationForSequence() = default;

    virtual void process() override;

    virtual const ProcessorInfo getProcessorInfo() const override;
    static const ProcessorInfo processorInfo_;

private:
    TriangulationSequenceInport scalarsInport_;
    PersistenceDiagramSequenceInport pdInport_;

    TriangulationSequence simplified_;
    TriangulationSequenceOutport scalarsOutport_;

    TemplateOptionProperty<int> mode_;
    FloatProperty threshold_;
    IntSizeTProperty numKeepPoints_;

	/* Performance Timer  */
    kth::PerformanceTimer performanceTimer_;

    /* Property for the timer */
    FloatProperty timer_;

};

}  // namespace inviwo
