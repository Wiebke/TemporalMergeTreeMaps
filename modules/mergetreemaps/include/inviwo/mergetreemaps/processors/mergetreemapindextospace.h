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
#include <inviwo/core/processors/poolprocessor.h>
#include <inviwo/dataframe/datastructures/dataframe.h>
#include <inviwo/topologytoolkit/ports/contourtreeport.h>
#include <inviwo/topologytoolkit/ports/triangulationdataport.h>
#include <inviwo/core/properties/ordinalproperty.h>
#include <inviwo/core/properties/boolproperty.h>
#include <inviwo/core/properties/minmaxproperty.h>
#include <inviwo/dataframe/properties/dataframeproperty.h>

namespace inviwo {

/** \docpage{org.inviwo.MergeTreeMapIndexToSpace, Merge Tree Map Index To Space}
 * ![](org.inviwo.MergeTreeMapIndexToSpace.png?classIdentifier=org.inviwo.MergeTreeMapIndexToSpace)
 * Takes dataframe with a vertex and a 1D mapping column and puts the 1D index back in the spatial
 * context.
 *
 * ### Inports
 *   * __mtmDf__ Dataframe with at least 2 columns: vertexID and 1D index.
 *   * __triangulation__ Triangulation with the original scalars.
 *
 * ### Outports
 *   * __outport__ Triangulation with the 1D index as a scalar.
 *
 * ### Properties
 *   * __vertexIdColumn__ Column of the vertex ids.
 *   * __posColumn__ Column of the 1D index.
 */
class IVW_MODULE_MERGETREEMAPS_API MergeTreeMapIndexToSpace : public PoolProcessor {
public:
    MergeTreeMapIndexToSpace();
    virtual ~MergeTreeMapIndexToSpace() = default;

    virtual void process() override;

    virtual const ProcessorInfo getProcessorInfo() const override;
    static const ProcessorInfo processorInfo_;

private:
    DataFrameInport mtmInport_{"mtm"};
    topology::TriangulationInport inport_{"triangulation"};
    topology::TriangulationOutport outport_{"indexMapped"};

    /// Scalar Mapping
    DataFrameColumnProperty vertexIdColumn_;

    /// 1D Position Mapping
    DataFrameColumnProperty positionColumn_;
};

}  // namespace inviwo
