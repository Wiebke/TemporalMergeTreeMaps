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
#include <inviwo/core/processors/poolprocessor.h>
#include <inviwo/dataframe/datastructures/dataframe.h>
#include <inviwo/core/properties/ordinalproperty.h>
#include <inviwo/core/properties/boolproperty.h>
#include <inviwo/core/properties/stringproperty.h>
#include <inviwo/core/properties/compositeproperty.h>
#include <inviwo/core/properties/fileproperty.h>
#include <inviwo/mergetreemaps/topologysequenceports.h>
#include <inviwo/mergetreemaps/mergetreemaputils.h>
#include <modules/tools/performancetimer.h>
#include <modules/python3/pythonscript.h>
#include <pybind11/pybind11.h>

namespace inviwo {

/** \docpage{org.inviwo.OptimizeMergeTreeMap, Optimize Merge Tree Map}
 * ![](org.inviwo.OptimizeMergeTreeMap.png?classIdentifier=org.inviwo.OptimizeMergeTreeMap)
 * Explanation of how to use the processor.
 *
 * ### Inports
 *   * __<Inport1>__ <description>.
 *
 * ### Outports
 *   * __<Outport1>__ <description>.
 *
 * ### Properties
 *   * __<Prop1>__ <description>.
 *   * __<Prop2>__ <description>
 */
class IVW_MODULE_MERGETREEMAPS_API OptimizeMergeTreeMap : public PoolProcessor {
public:
    OptimizeMergeTreeMap();
    virtual ~OptimizeMergeTreeMap() = default;

    virtual void process() override;

    virtual const ProcessorInfo getProcessorInfo() const override;
    static const ProcessorInfo processorInfo_;

    static void treeToMiniZinc(const topology::ContourTreeData &treeData,
                               const mtmutils::SubTreeInfo &info, const size_t i,
                               const bool firstFixed, const bool includeDefsAndConstraints,
                               std::string &modelTree);
    static void overlapToMiniZinc(const mtmutils::SubTreeInfo &infoFirst,
                                  const mtmutils::SubTreeInfo &infoSecond, const size_t i,
                                  const mtmutils::Overlap &currentNDOverlap,
                                  const bool includeDefsAndConstraints, std::string &modelOverlap);
    static void globalDefsToMiniZinc(const size_t numTrees, const size_t domainSize,
                                     std::string &globalDefs);
    static void objectiveToMiniZInc(const size_t numTrees, const size_t sumNumSubTrees,
                                    const size_t maxNumSubTrees,
                                    const std::vector<mtmutils::SubTreeInfo> &subTreeInfos,
                                    std::string &objectiveAndOptimize);

private:
    ContourTreeSequenceInport treesInport_{"contourTrees"};

    LandscapeDecisionOutport decisionsOutport_{"decisions"};
    DataFrameOutport optimizationStatisticsOutport_{"statistics"};

    std::string modelFile_;

    // Input properies
    BoolProperty allTimeSteps_;
    FileProperty allTimeStepsModelFile_;
    BoolProperty overwriteModel_;
    BoolProperty firstFixed_;
    BoolProperty solverTimeoutEnabled_;
    IntSizeTProperty solverTimeoutSeconds_;
    BoolProperty detailedStatsEnabled_;

    // Threshold for warning about close saddles
    IntSizeTProperty arcSizeThreshold_;

    // Total objective
    FloatProperty objective_;

    /* Property for the timer */
    FloatProperty timer_;

    BoolProperty modelDebuggingEnabled_;
    CompositeProperty modelDebugging_;
    // Time step which to show modelString
    IntSizeTProperty timestep_;
    StringProperty modelInput_;

    // Intermediate values to avoid setting properties from the pool call
    std::string modelInputString_;
    float totalObjective_ = 0.0f;
    /* Performance Timer  */
    kth::PerformanceTimer performanceTimer_;

    size_t domainSize_ = 0;
    std::vector<mtmutils::SubTreeInfo> subTreeInfos_;
    std::vector<mtmutils::Overlap> nDOverlap_;

    // Script doing the computations for two steps
    PythonScriptDisk script_;

    // Script doing the computations for all time steps simultaneouly
    PythonScriptDisk scriptAll_;
};

}  // namespace inviwo
