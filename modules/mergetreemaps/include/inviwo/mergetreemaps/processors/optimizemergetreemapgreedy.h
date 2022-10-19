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
#include <random>

namespace inviwo {

/** \docpage{org.inviwo.OptimizeMergeTreeMapGreedy, Optimize Merge Tree Map Greedy}
 * ![](org.inviwo.OptimizeMergeTreeMapGreedy.png?classIdentifier=org.inviwo.OptimizeMergeTreeMapGreedy)
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
class IVW_MODULE_MERGETREEMAPS_API OptimizeMergeTreeMapGreedy : public PoolProcessor {
public:
    OptimizeMergeTreeMapGreedy();
    virtual ~OptimizeMergeTreeMapGreedy() = default;

    virtual void process() override;

    virtual const ProcessorInfo getProcessorInfo() const override;
    static const ProcessorInfo processorInfo_;

private:
    ContourTreeSequenceInport treesInport_{"contourTrees"};

    LandscapeDecisionOutport decisionsOutport_{"decisions"};
    DataFrameOutport optimizationStatisticsOutport_{"statistics"};

    // Time step to extract ordering information
    IntSizeTProperty focusTimestep_;

    // Read-only property for number of time steps
    IntSizeTProperty timesteps_;
    // Should focus time step be scaled to new number?
    BoolProperty scaleTimes_;
    size_t prevNumTimesteps_ = std::numeric_limits<size_t>::max();

    // Order at focus time step randomized?
    BoolProperty randomizeOrder_;

    // Seed for order
    IntSizeTProperty orderSeed_;

    /// Use Counts instead of fractions for matrices
    BoolProperty useCounts_;
    /// Normalize objective in sum
    BoolProperty normalizePerTObjective_;

    CompositeProperty showcaseBadChoices_;
    BoolProperty chooseWorse_;
    BoolProperty randomizeFully_;

    // Total objective
    FloatProperty objective_;

    // Extracted directly from the trees
    size_t domainSize_ = 0;
    std::vector<mtmutils::SubTreeInfo> subTreeInfos_;
    std::vector<mtmutils::Overlap> nDOverlap_;
    // Slowly being filled as we make more decisions
    std::vector<std::vector<mtmutils::LandscapeInfo>> landscapeInfos_;

    float totalObjective_ = 0.0f;

    /* Property for the timer */
    FloatProperty timer_;

    /* Performance Timer  */
    kth::PerformanceTimer performanceTimer_;

    // Random generation
    mutable std::mt19937 randomGen;
};

}  // namespace inviwo
