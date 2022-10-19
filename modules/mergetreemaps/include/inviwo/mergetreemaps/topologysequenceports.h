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
#include <inviwo/core/ports/datainport.h>
#include <inviwo/core/ports/dataoutport.h>
#include <inviwo/topologytoolkit/datastructures/contourtreedata.h>
#include <inviwo/topologytoolkit/datastructures/triangulationdata.h>
#include <inviwo/topologytoolkit/ports/persistencediagramport.h>
#include <inviwo/dataframe/datastructures/dataframe.h>

namespace inviwo {

using ContourTreeSequence = std::vector<std::shared_ptr<topology::ContourTreeData>>;

/**
 * \ingroup ports
 */
using ContourTreeSequenceInport = DataInport<ContourTreeSequence>;

/**
 * \ingroup ports
 */
using ContourTreeSequenceOutport = DataOutport<ContourTreeSequence>;

using PersistenceDiagramSequence = std::vector<std::shared_ptr<topology::PersistenceDiagramData>>;

/**
 * \ingroup ports
 */
using PersistenceDiagramSequenceInport = DataInport<PersistenceDiagramSequence>;

/**
 * \ingroup ports
 */
using PersistenceDiagramSequenceOutport = DataOutport<PersistenceDiagramSequence>;

using DataFrameSequence = std::vector<std::shared_ptr<DataFrame>>;

/**
 * \ingroup ports
 */
using DataFrameSequenceInport = DataInport<DataFrameSequence>;

/**
 * \ingroup ports
 */
using DataFrameSequenceOutport = DataOutport<DataFrameSequence>;

using TriangulationSequence = std::vector<std::shared_ptr<topology::TriangulationData>>;

/**
 * \ingroup ports
 */
using TriangulationSequenceInport = DataInport<TriangulationSequence>;

/**
 * \ingroup ports
 */
using TriangulationSequenceOutport = DataOutport<TriangulationSequence>;

using LandscapeDecisions = std::vector<std::vector<int>>;

using LandscapeDecisionInport = DataInport<LandscapeDecisions>;

using LandscapeDecisionOutport = DataOutport<LandscapeDecisions>;

using VertexOrderMap = std::map<ttk::SimplexId, size_t>;

}  // namespace inviwo
