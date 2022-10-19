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
#include <inviwo/core/ports/volumeport.h>
#include <inviwo/topologytoolkit/ports/triangulationdataport.h>
#include <inviwo/mergetreemaps/topologysequenceports.h>
#include <modules/tools/performancetimer.h>

namespace inviwo {

/** \docpage{org.inviwo.ContourTreesForSequence, Contour Trees For Sequence}
 * ![](org.inviwo.ContourTreesForSequence.png?classIdentifier=org.inviwo.ContourTreesForSequence)
 * Computes a sequence of contour trees for a given TTK triangulation and several timesteps of a scalar field given by a volume sequence.
 *
 * \see ttk::FTMTre
 *
 * ### Inports
 *   * __triangulation__   input triangulation. Its scalars are ignored, only the connectivity is important.
 *   * __scalars__         input scalars  
 *
 * ### Outports
 *   * __Contour Trees__   Sequence of contour trees, each including the tree, its type and the underlying triangulation
 *
 * ### Properties
 *	 * __Contour Tree Extraction__
 *		+ __Tree Type__ Defines which tree type to calculate
 *		+ __Number of Threads__ Defines how many threads to use when calculating the tree
 *      + __Segmentation__ Also compute segmentation of the data based on the contour tree
 *      + __Normalization__ Normalize Node and Arc Ids over several runs
 */
class IVW_MODULE_MERGETREEMAPS_API ContourTreesForSequence : public PoolProcessor {
public:
    ContourTreesForSequence();
    virtual ~ContourTreesForSequence() = default;

    virtual void process() override;

    virtual const ProcessorInfo getProcessorInfo() const override;
    static const ProcessorInfo processorInfo_;

private:
    TriangulationSequenceInport scalarsInport_;
    ContourTreeSequenceOutport outport_;
    ContourTreeSequence trees_;

    TemplateOptionProperty<topology::TreeType> treeType_;
    IntProperty threadCount_;
    BoolProperty segmentation_;
    BoolProperty normalization_;

	/* Performance Timer */
    kth::PerformanceTimer performanceTimer_;

    /* Property for the timer */
    FloatProperty timer_;

};

}  // namespace inviwo
