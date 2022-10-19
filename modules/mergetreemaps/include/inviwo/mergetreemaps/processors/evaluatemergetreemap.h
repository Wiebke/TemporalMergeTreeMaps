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
#include <inviwo/dataframe/datastructures/dataframe.h>
#include <inviwo/core/properties/stringproperty.h>
#include <inviwo/core/properties/ordinalproperty.h>
#include <inviwo/core/properties/boolproperty.h>
#include <inviwo/core/properties/minmaxproperty.h>
#include <inviwo/core/properties/compositeProperty.h>
#include <inviwo/core/properties/transferfunctionproperty.h>
#include <inviwo/mergetreemaps/topologysequenceports.h>
#include <inviwo/mergetreemaps/mergetreemaputils.h>
#include <modules/tools/performancetimer.h>

namespace inviwo {

/** \docpage{org.inviwo.EvaluateMergeTreeMap, Evaluate Merge Tree Map}
 * ![](org.inviwo.EvaluateMergeTreeMap.png?classIdentifier=org.inviwo.EvaluateMergeTreeMap)
 * Evaluates a merge tree map, by computing overlap in nD and comparing it with overlap in 1D,
 * overlap in 1D is computed by computing only the ranges each arc covers in 1D, but can be checked
 * against a given landscape as well.
 *
 * ### Inports
 *   * __landscape__ Already computed landscape as sequence of data frames.
 *   * __contourTrees__ Sequence of contour trees, to compute nD overlap.
 *   * __decisions__ Sequence of ordering decisions at each upArc.
 *   * __temporalTree__ Temporal Tree for sequence of contour trees, including order of leaves.
 *
 * ### Outports
 *   * __statistics__ objective values for each pair of consecutive time steps.
 *   * __matrices__ Overlap matrices for each pair of consecutive time steps, can be 1D overlap, nd
 * Overlap, difference of difference^2 in vertex counts or fractions based on subtree size.
 *   * __treeInfos__ tree information for each contour tree.
 *   * __matricesOverlapType__ Overlap type matrices for each pair of consecutive time steps
 *
 * ### Properties
 *   * __<Prop1>__ <description>.
 *   * __<Prop2>__ <description>
 */
class IVW_MODULE_MERGETREEMAPS_API EvaluateMergeTreeMap : public Processor {
public:
    EvaluateMergeTreeMap();
    virtual ~EvaluateMergeTreeMap() = default;

    virtual void process() override;

    virtual const ProcessorInfo getProcessorInfo() const override;
    static const ProcessorInfo processorInfo_;

private:
    DataFrameSequenceInport inport_{"landscape"};
    ContourTreeSequenceInport treesInport_{"contourTrees"};
    kth::TemporalTreeInport temporalTreeInport_{"temporalTree"};
    LandscapeDecisionInport decisionsInport_{"decisions"};

    DataFrameOutport objectivesOutport_{"statistics"};
    DataFrameSequenceOutport matrixOutport_{"matrices"};
    DataFrameSequenceOutport treeInfosOutport_{"treeInfos"};
    DataFrameSequenceOutport matrixOverlapTypeOutport_{"overlapType"};

    /// In which way should the order between superarcs be established?
    TemplateOptionProperty<mtmutils::SuperArcOrderMode> orderMode_;

    /// Sum over all objective values in between time steps
    DoubleProperty evalValue_;

    /// Check calculated limits against the already calculated
    BoolProperty checkAgainstCalculated_;

    /// Composite for all other additional outputs
    CompositeProperty additionalOutput_;

    /// Create Matrix output
    BoolProperty createMatrixOutput_;
    /// Use Counts instead of fractions for matrices
    BoolProperty useCounts_;
    /// Normalize objective in sum
    BoolProperty normalizePerTObjective_;

    /// Outputmode for overlap matrices
    TemplateOptionProperty<int> matrixOutputMode_;
    /// Create overlap type matrix output
    BoolProperty createOverlapTypeOutput_;

    /// Create tree information output (size of Arcs)
    BoolProperty createTreeInfoOutput_;

    /// Create full modelInput to be used by MiniZic externally
    BoolProperty createModelInput_;
    /// ModelInput
    StringProperty modelInput_;

    /// Performance Timer
    kth::PerformanceTimer performanceTimer_;

    /// Property for the timer
    FloatProperty timer_;

    size_t domainSize_ = 4096;
    std::vector<mtmutils::SubTreeInfo> subTreeInfos_;
    std::vector<std::vector<mtmutils::LandscapeInfo>> landscapeInfos_;

    std::vector<mtmutils::Overlap> nDOverlap_;
    std::vector<mtmutils::Overlap> oneDOverlapFullLandscape_;
    std::vector<mtmutils::Overlap> oneDOverlapLandscapeRanges_;

    std::vector<VertexOrderMap> vertexOrderMapPerTimeStep_;

    void toMiniZinc();
};
}  // namespace inviwo
