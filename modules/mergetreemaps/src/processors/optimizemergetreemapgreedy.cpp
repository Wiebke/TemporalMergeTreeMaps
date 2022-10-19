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

#include <inviwo/mergetreemaps/processors/optimizemergetreemapgreedy.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo OptimizeMergeTreeMapGreedy::processorInfo_{
    "org.inviwo.OptimizeMergeTreeMapGreedy",  // Class identifier
    "Optimize Merge Tree Map Greedy",         // Display name
    "Merge Tree Maps",                        // Category
    CodeState::Experimental,                  // Code state
    Tags::None,                               // Tags
};
const ProcessorInfo OptimizeMergeTreeMapGreedy::getProcessorInfo() const { return processorInfo_; }

OptimizeMergeTreeMapGreedy::OptimizeMergeTreeMapGreedy()
    : PoolProcessor()
    , focusTimestep_("focustimestep", "Focus Time Step", 0)
    , timesteps_("timesteps", "Time Steps", 16, 1, std::numeric_limits<size_t>::max())
    , scaleTimes_("scaleTime", "Scale Times", true)
    , randomizeOrder_("randomizeOrders", "Randomize Focus Order", false)
    , orderSeed_("colorSeed", "Random Seed", 0, 0, RAND_MAX + 1, 1)
    , useCounts_("useCounts", "Use Counts, not Fractions", true)
    , normalizePerTObjective_("normalizeObj", "Normalize Objective", false)
    , showcaseBadChoices_("showcaseBadChoices", "Showcase Bad Choices (Eval)")
    , chooseWorse_("chooseWorse", "Choose Worse Order", false)
    , randomizeFully_("randomizeFully", "Randomize All Orders", false)
    , objective_("objective", "Objective Value", 0.0, 0.0, std::numeric_limits<float>::max(), 0.01,
                 InvalidationLevel::Valid, PropertySemantics::Text)
    , timer_("timer", "Eval Time (s)", 0.f, 0.f, std::numeric_limits<float>::max(), 0.001f,
             InvalidationLevel::Valid, PropertySemantics::Text) {

    addPort(treesInport_);
    addPort(decisionsOutport_);
    addPort(optimizationStatisticsOutport_);

    addProperties(focusTimestep_, timesteps_, scaleTimes_, randomizeOrder_, orderSeed_, useCounts_,
                  normalizePerTObjective_, showcaseBadChoices_, objective_, timer_);

    showcaseBadChoices_.addProperties(chooseWorse_, randomizeFully_);

    timesteps_.setSemantics(PropertySemantics::Text);
    timesteps_.setReadOnly(true);

    orderSeed_.setSemantics(PropertySemantics::Text);
    orderSeed_.visibilityDependsOn(randomizeOrder_, [](const auto &p) { return p.get(); });
    objective_.setReadOnly(true);
    timer_.setReadOnly(true);

    treesInport_.onChange([&]() {
        const auto treesData = treesInport_.getData();
        if (treesData) {
            const auto numNewTimeSteps = treesData.get()->size();
            timesteps_.set(numNewTimeSteps);
            if (scaleTimes_.get() && prevNumTimesteps_ < std::numeric_limits<size_t>::max()) {
                auto scale = numNewTimeSteps / static_cast<float>(prevNumTimesteps_);
                if (scale > 1) {
                    focusTimestep_.setMaxValue(numNewTimeSteps - 1);
                }
                // Map from index to number, then scale, then map back to index
                // Guard against setting size_t to negtive number (will lead to underflow)
                int newTimeStep = round(scale * static_cast<int>(focusTimestep_.get() + 1)) - 1;
                focusTimestep_.set(std::max(newTimeStep, 0));
            }
            focusTimestep_.setMaxValue(numNewTimeSteps - 1);
            prevNumTimesteps_ = timesteps_.get();
        }
    });
}

void OptimizeMergeTreeMapGreedy::process() {
    performanceTimer_.Reset();

    const auto treesData = treesInport_.getData();

    if (!treesData) return;

    auto numTrees = treesData.get()->size();
    size_t focusTimestep = std::min(focusTimestep_.get(), numTrees - 1);

    if (treesInport_.isChanged() || focusTimestep_.isModified()) {

        subTreeInfos_ = std::vector<mtmutils::SubTreeInfo>(numTrees);
        nDOverlap_ = std::vector<mtmutils::Overlap>(numTrees - 1);
        domainSize_ = treesData.get()->at(0).get()->getTree()->getNumberOfVertices();

        // pre-compute all subtree infos, will be used by both overlap methods
        for (size_t i = 0; i < numTrees; i++) {
            const topology::ContourTreeData &treeData = *treesData.get()->at(i).get();
            auto &info = subTreeInfos_[i];
            mtmutils::accumulateArcsTree(treeData.getTree(), info);
        }

        // compute overlap
        for (size_t i = 0; i < numTrees - 1; i++) {
            const topology::ContourTreeData &treeDataFirst = *treesData.get()->at(i).get();
            const mtmutils::SubTreeInfo &infoFirst = subTreeInfos_[i];
            const topology::ContourTreeData &treeDataSecond = *treesData.get()->at(i + 1).get();
            const mtmutils::SubTreeInfo &infoSecond = subTreeInfos_[i + 1];
            // Later we iterate backwards for everything before the focus time step,
            // Thus overlap matrices need to be transposed
            if (i < focusTimestep) {
                computeOverlapND(treeDataSecond, infoSecond, treeDataFirst, infoFirst,
                                 nDOverlap_[i]);
            } else {
                computeOverlapND(treeDataFirst, infoFirst, treeDataSecond, infoSecond,
                                 nDOverlap_[i]);
            }
        }
    }

    using Result = std::pair<std::shared_ptr<LandscapeDecisions>, std::shared_ptr<DataFrame>>;

    bool randomizeOrder = randomizeOrder_.get();
    if (randomizeOrder) {
        randomGen.seed(static_cast<std::mt19937::result_type>(orderSeed_.get()));
    }

    bool useCounts = useCounts_.get();
    bool normalizePerTObjective = normalizePerTObjective_.get();

    bool randomizeFully = randomizeFully_.get();
    bool chooseWorse = chooseWorse_.get();

	if (randomizeFully) LogProcessorWarn("Objective not computed, use evaluation processor.");

    const auto optimizeLandscape = [treesData, focusTimestep, randomizeOrder, useCounts,
                                    randomizeFully, chooseWorse, normalizePerTObjective, numTrees,
                                    this](pool::Stop stop, pool::Progress progress) {
        // Prepare output
        auto landscapeDecisions = std::make_shared<LandscapeDecisions>(numTrees);
        auto df = std::make_shared<DataFrame>();

        totalObjective_ = 0.0f;

        std::vector<float> objectives(numTrees - 1, 0.0f);
        landscapeInfos_ = std::vector<std::vector<mtmutils::LandscapeInfo>>(numTrees);

        const mtmutils::SubTreeInfo &infoFocus = subTreeInfos_[focusTimestep];
        std::vector<int> &decisionsFocus = landscapeDecisions->at(focusTimestep);
        std::vector<mtmutils::LandscapeInfo> &landscapeFocus = landscapeInfos_[focusTimestep];
        const topology::ContourTreeData &treeDataFocus = *treesData.get()->at(focusTimestep).get();
        // Set decision in initial time step all to false
        decisionsFocus = std::vector<int>(infoFocus.numArcs, 0);
        if (randomizeOrder || randomizeFully) {
            std::bernoulli_distribution d(0.5);
            for (size_t arcId = 0; arcId < infoFocus.numArcs; arcId++) {
                decisionsFocus[arcId] = d(randomGen);
            }
        }
        mtmutils::accumulateLimitsTree(
            treeDataFocus.getTree(), infoFocus, mtmutils::SuperArcOrderMode::ByDecision,
            std::map<ttk::ftm::idNode, size_t>(), decisionsFocus, landscapeFocus);

        // Go forward in time
        for (size_t i = focusTimestep; i < numTrees - 1; i++) {

            const mtmutils::SubTreeInfo &infoFirst = subTreeInfos_[i];
            const mtmutils::SubTreeInfo &infoSecond = subTreeInfos_[i + 1];
            const std::vector<mtmutils::LandscapeInfo> &landscapeFirst = landscapeInfos_[i];
            const topology::ContourTreeData &treeDataSecond = *treesData.get()->at(i + 1).get();

            // Initalize decisions vector
            std::vector<int> &decisions1 = landscapeDecisions->at(i + 1);
            decisions1 = std::vector<int>(infoSecond.numArcs);

            float objective = mtmutils::decideGreedyTree(
                treeDataSecond.getTree(), infoFirst, landscapeFirst, infoSecond, nDOverlap_[i],
                useCounts, chooseWorse, decisions1, landscapeInfos_[i + 1]);

            if (randomizeFully) {
                std::bernoulli_distribution d(0.5);
                for (size_t arcId = 0; arcId < infoSecond.numArcs; arcId++) {
                    decisions1[arcId] = d(randomGen);
                }
            }

            // Divide by domainSize for consistency with MiniZinc, which needs bounded floating
            // points numbers
            if (useCounts) objective /= domainSize_ * domainSize_;
            if (normalizePerTObjective) objective /= infoFirst.numArcs * infoSecond.numArcs;
            objectives[i] = objective;
            totalObjective_ += objective;
        }

        // Go forward in time
        for (size_t i = focusTimestep; i > 0; i--) {

            const mtmutils::SubTreeInfo &infoFirst = subTreeInfos_[i];
            const mtmutils::SubTreeInfo &infoSecond = subTreeInfos_[i - 1];
            const std::vector<mtmutils::LandscapeInfo> &landscapeFirst = landscapeInfos_[i];
            const topology::ContourTreeData &treeDataSecond = *treesData.get()->at(i - 1).get();

            // Initalize decisions vector
            std::vector<int> &decisions1 = landscapeDecisions->at(i - 1);
            decisions1 = std::vector<int>(infoSecond.numArcs);

            float objective = mtmutils::decideGreedyTree(
                treeDataSecond.getTree(), infoFirst, landscapeFirst, infoSecond, nDOverlap_[i - 1],
                useCounts, chooseWorse, decisions1, landscapeInfos_[i - 1]);

            if (randomizeFully) {
                std::bernoulli_distribution d(0.5);
                for (size_t arcId = 0; arcId < infoSecond.numArcs; arcId++) {
                    decisions1[arcId] = d(randomGen);
                }
            }

            // Divide by domainSize for consistency with MiniZinc, which needs bounded floating
            // points numbers
            if (useCounts) objective /= domainSize_ * domainSize_;
            if (normalizePerTObjective) objective /= infoFirst.numArcs * infoSecond.numArcs;
            objectives[i - 1] = objective;
            totalObjective_ += objective;
        }

        df->addColumnFromBuffer("Objective", util::makeBuffer<float>(std::move(objectives)));
        df->updateIndexBuffer();

        return std::make_pair(landscapeDecisions, df);
    };

    decisionsOutport_.setData(nullptr);
    dispatchOne(optimizeLandscape, [this](Result result) {
        decisionsOutport_.setData(result.first);
        objective_.set(totalObjective_);
        optimizationStatisticsOutport_.setData(result.second);
        newResults();
        timer_.set(performanceTimer_.ElapsedTimeAndReset());
    });
}

}  // namespace inviwo
