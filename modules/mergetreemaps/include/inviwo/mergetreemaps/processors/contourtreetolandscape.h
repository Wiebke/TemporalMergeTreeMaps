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
#include <inviwo/core/common/inviwo.h>
#include <inviwo/core/processors/poolprocessor.h>
#include <inviwo/dataframe/datastructures/dataframe.h>
#include <inviwo/core/properties/stringproperty.h>
#include <inviwo/core/properties/ordinalproperty.h>
#include <inviwo/core/properties/boolproperty.h>
#include <inviwo/topologytoolkit/ports/contourtreeport.h>
#include <inviwo/topologytoolkit/ports/triangulationdataport.h>
#include <inviwo/mergetreemaps/topologysequenceports.h>
#include <modules/temporaltreemaps/datastructures/treeport.h>
#include <inviwo/mergetreemaps/mergetreemaputils.h>

namespace inviwo {

/** \docpage{org.inviwo.ContourTreeToLandscape, Contour Tree To Landscape}
 * ![](org.inviwo.ContourTreeToLandscape.png?classIdentifier=org.inviwo.ContourTreeToLandscape)
 * Explanation of how to use the processor.
 *
 * ### Inports
 *   * __contourTree__   Contour tree input.
 *   * __temporalTree__  Temporal tree input, contains order, mapping between contourtree and
 *                       temporal tree is given through node names, node names have the folling
 *                       form: "vertexId_timeStep"
 *
 * ### Outports
 *   * __dataframe__ Dataframe for linearized function for the contour tree.
 *
 * ### Properties
 *   * __treeType__ Type of the contour tree (Join, Split, Contour).
 */
class IVW_MODULE_MERGETREEMAPS_API ContourTreeToLandscape : public PoolProcessor {
public:
    ContourTreeToLandscape();
    virtual ~ContourTreeToLandscape() = default;

    virtual void process() override;

    virtual const ProcessorInfo getProcessorInfo() const override;
    static const ProcessorInfo processorInfo_;

private:
    topology::ContourTreeInport inport_{"contourTree"};
    topology::TriangulationInport originalScalars_{"originalScalars"};

    LandscapeDecisionInport decisionsInport_{"decisions"};
    kth::TemporalTreeInport temporalTreeInport_{"temporalTree"};

    DataFrameOutport outport_{"outport"};

    /* Type of tree: */
    StringProperty treeType_;

    /* Map original instead of simplified scalars */
    BoolProperty mapOriginalScalars_;

    /* Use unaugmented trees (only super nodes and super arc sized)*/
    BoolProperty useUnaugmentedTreeOnly_;

    /* Should column for position be added or not */
    BoolProperty position_;

    /* Dimensionality of the data */
    IntSizeTProperty dataDims_;

    /* In which way should the order between superarcs be established? */
    TemplateOptionProperty<mtmutils::SuperArcOrderMode> orderMode_;

    /* Time step to extract ordering information */
    IntSizeTProperty timestep_;

    /* Time step sequence index (timestep + 1) */
    IntSizeTProperty timestepSequenceIndex_;

    /* Should column for levelset be added or not or not */
    BoolProperty levelSet_;

    /* isoValue for super- or sub-level set (depends on tree type, join->sub, split->super) */
    FloatProperty isoValue_;

    /// Intermediate value for vertex order map (needed so it does not go out of scope)
    VertexOrderMap vertexOrderMap_;
};

}  // namespace inviwo
