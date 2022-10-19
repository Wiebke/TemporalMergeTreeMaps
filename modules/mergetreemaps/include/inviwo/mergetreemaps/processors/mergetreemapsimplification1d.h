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
#include <inviwo/core/properties/ordinalproperty.h>
#include <inviwo/core/properties/boolproperty.h>
#include <inviwo/core/properties/minmaxproperty.h>
#include <inviwo/dataframe/properties/dataframeproperty.h>

namespace inviwo {

/** \docpage{org.inviwo.MergeTreeMapSimplification1D, Merge Tree Map Simplification1D}
 * ![](org.inviwo.MergeTreeMapSimplification1D.png?classIdentifier=org.inviwo.MergeTreeMapSimplification1D)
 * Processor that simplifies a function given in a dataframe with Ramer–Douglas–Peucker.
 *
 * ### Inports
 *   * __<function>__ DataFrame where two columns correspond to x and y of a function.
 *
 * ### Outports
 *   * __<simplified>__ simplified version of the input.
 *
 * ### Properties
 *   * __epsilon__ <description>.
 */
class IVW_MODULE_MERGETREEMAPS_API MergeTreeMapSimplification1D : public Processor {
public:
    MergeTreeMapSimplification1D();
    virtual ~MergeTreeMapSimplification1D() = default;

    float distanceToLine(const vec3& point, const vec3& lineP1, const vec3& lineP2);
    std::vector<vec3> simplify(const std::vector<vec3>& points, float epsilon);
    virtual void process() override;

    virtual const ProcessorInfo getProcessorInfo() const override;
    static const ProcessorInfo processorInfo_;

private:
    DataFrameInport inport_{"function"};
    DataFrameOutport outport_{"simplified"};

    /// Position Mapping
    DataFrameColumnProperty positionColumn_;

    /// Scalar Mapping
    DataFrameColumnProperty scalarColumn_;

    /// Epsilon used for simplification
    FloatProperty epsilon_;
};

}  // namespace inviwo
