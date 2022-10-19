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

#include <inviwo/mergetreemaps/processors/optimizemergetreemap.h>
#include <inviwo/mergetreemaps/mergetreemapsmodule.h>
#include <inviwo/core/util/filesystem.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <regex>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo OptimizeMergeTreeMap::processorInfo_{
    "org.inviwo.OptimizeMergeTreeMap",  // Class identifier
    "Optimize Merge Tree Map",          // Display name
    "Merge Tree Maps",                  // Category
    CodeState::Experimental,            // Code state
    Tags::None,                         // Tags
};
const ProcessorInfo OptimizeMergeTreeMap::getProcessorInfo() const { return processorInfo_; }

OptimizeMergeTreeMap::OptimizeMergeTreeMap()
    : PoolProcessor()
    , modelFile_(InviwoApplication::getPtr()->getModuleByType<MergeTreeMapsModule>()->getPath(
                     ModulePath::Scripts) +
                 "/twoTimeSteps.mzn")
    , allTimeSteps_("allTimeSteps", "All timesteps")
    , allTimeStepsModelFile_(
          "allTimeStepsModelFile", "Intermediate Model File",
          InviwoApplication::getPtr()->getModuleByType<MergeTreeMapsModule>()->getPath(
              ModulePath::Scripts) +
              "/allTimeSteps.mzn")
    , overwriteModel_("overwriteModel", "Overwrite Model File", false)
    , firstFixed_("firstFixed", "Fix Ordering in First Time Step", true)
    , solverTimeoutEnabled_("solverTimeoutEnabled", "Enable Solver Timeout", false)
    , solverTimeoutSeconds_("solverTimeoutSeconds", "Solver Timeout (s)", 10, 1, 86400)
    , detailedStatsEnabled_("detailedStatsEnabled", "Enable Detailed Stats", false)
    , arcSizeThreshold_("arcSizeThreshold", "#Vertices for Close Saddles", 1)
    , objective_("objective", "Objective Value", 0.0, 0.0, std::numeric_limits<float>::max(), 0.01,
                 InvalidationLevel::Valid, PropertySemantics::Text)
    , timer_("timer", "Eval Time (s)", 0.f, 0.f, std::numeric_limits<float>::max(), 0.001f,
             InvalidationLevel::Valid, PropertySemantics::Text)
    , modelDebuggingEnabled_("modelDebuggingEnabled", "Enable Model Debugging", false)
    , modelDebugging_("modelDebugging", "Model Debugging")
    , timestep_("timestep", "Time Step", 0)
    , modelInput_("modelInput", "Model Input for Time Step", "", InvalidationLevel::Valid,
                  PropertySemantics::TextEditor)
    , script_(InviwoApplication::getPtr()->getModuleByType<MergeTreeMapsModule>()->getPath(
                  ModulePath::Scripts) +
              "/twoTimeSteps.py")
    , scriptAll_(InviwoApplication::getPtr()->getModuleByType<MergeTreeMapsModule>()->getPath(
                     ModulePath::Scripts) +
                 "/allTimeSteps.py") {

    addPort(treesInport_);
    addPort(decisionsOutport_);
    addPort(optimizationStatisticsOutport_);

    addProperties(allTimeSteps_, allTimeStepsModelFile_, overwriteModel_, firstFixed_,
                  solverTimeoutEnabled_, solverTimeoutSeconds_, detailedStatsEnabled_,
                  arcSizeThreshold_);
    allTimeStepsModelFile_.visibilityDependsOn(allTimeSteps_,
                                               [](const auto &p) { return p.get(); });
    allTimeStepsModelFile_.addNameFilter(FileExtension("mzn", "MiniZinc Model"));
    allTimeStepsModelFile_.setAcceptMode(AcceptMode::Save);
    solverTimeoutSeconds_.visibilityDependsOn(solverTimeoutEnabled_,
                                              [](const auto &p) { return p.get(); });
    treesInport_.onChange([&]() {
        if (treesInport_.hasData() && treesInport_.getData()->size() > 0) {
            arcSizeThreshold_.setMaxValue(
                treesInport_.getData().get()->at(0).get()->getTree()->getNumberOfVertices());
            timestep_.setMaxValue(treesInport_.getData()->size() - 1);
        }
    });

    addProperties(objective_, timer_, modelDebuggingEnabled_, modelDebugging_);
    objective_.setReadOnly(true);
    timer_.setReadOnly(true);

    modelDebugging_.visibilityDependsOn(modelDebuggingEnabled_,
                                        [](const auto &p) { return p.get(); });
    modelDebugging_.addProperties(timestep_, modelInput_);
}

void OptimizeMergeTreeMap::process() {

    performanceTimer_.Reset();

    const auto treesData = treesInport_.getData();

    if (!treesData) return;

    auto numTrees = treesData.get()->size();

    const std::string &allTimeStepsFilename = allTimeStepsModelFile_.get();

    if (allTimeSteps_.get() && filesystem::fileExists(allTimeStepsFilename) &&
        !overwriteModel_.get()) {
        LogProcessorWarn("File already exists: " << allTimeStepsFilename);
        LogProcessorWarn("Running on existing file");
    }

    if (treesInport_.isChanged()) {

        subTreeInfos_ = std::vector<mtmutils::SubTreeInfo>(numTrees);
        nDOverlap_ = std::vector<mtmutils::Overlap>(numTrees - 1);
        domainSize_ = treesData.get()->at(0).get()->getTree()->getNumberOfVertices();
        size_t numCloseSaddles = 0;

        // pre-compute all subtree infos, will be used by both overlap methods
        for (size_t i = 0; i < numTrees; i++) {
            const topology::ContourTreeData &treeData = *treesData.get()->at(i).get();
            auto &info = subTreeInfos_[i];
            mtmutils::accumulateArcsTree(treeData.getTree(), info);
            for (auto arcSize : info.arcSizes) {
                // ToDo: Add Information on values of closest saddles
                if (arcSize <= arcSizeThreshold_) numCloseSaddles++;
            }
        }

        if (numCloseSaddles > 0) {
            LogProcessorWarn("There are " << numCloseSaddles
                                          << " close saddles in the data set, seperated by at most "
                                          << arcSizeThreshold_ << " vertices. On average this is "
                                          << numCloseSaddles / static_cast<float>(numTrees)
                                          << " per timestep.");
        }

        // compute overlap
        for (size_t i = 0; i < numTrees - 1; i++) {
            const topology::ContourTreeData &treeDataFirst = *treesData.get()->at(i).get();
            const mtmutils::SubTreeInfo &infoFirst = subTreeInfos_[i];
            const topology::ContourTreeData &treeDataSecond = *treesData.get()->at(i + 1).get();
            const mtmutils::SubTreeInfo &infoSecond = subTreeInfos_[i + 1];
            mtmutils::computeOverlapND(treeDataFirst, infoFirst, treeDataSecond, infoSecond,
                                       nDOverlap_[i]);
        }
    }

    using Result = std::pair<std::shared_ptr<LandscapeDecisions>, std::shared_ptr<DataFrame>>;

    // Capture this, since script cannot be referenced
    const auto optimizeLandscape = [treesData, numTrees, allTimeSteps = allTimeSteps_.get(),
                                    firstFixed = firstFixed_.get(),
                                    solverTimeoutEnabled = solverTimeoutEnabled_.get(),
                                    solverTimeoutSeconds = solverTimeoutSeconds_.get(),
                                    detailedStatsEnabled = detailedStatsEnabled_.get(),
                                    modelDebuggingEnabled = modelDebuggingEnabled_.get(),
                                    timestep = timestep_.get(),
                                    this](pool::Stop stop, pool::Progress progress) {
        // Prepare output
        auto landscapeDecisions = std::make_shared<LandscapeDecisions>(numTrees);
        auto df = std::make_shared<DataFrame>();

        totalObjective_ = 0.0f;

        // Prepare model output file in case we optimize all time steps
        std::ofstream outfile;
        outfile.exceptions(std::ofstream::failbit | std::ofstream::badbit);
        if (allTimeSteps) {
            try {
                outfile.open(allTimeStepsModelFile_);
            } catch (const std::ofstream::failure &e) {
                LogProcessorError("File could not be opened: " << allTimeStepsModelFile_);
                LogProcessorError("  Error Code: " << e.code() << "    . " << e.what());
                return std::make_pair(landscapeDecisions, df);
            }
            std::string globalDefs;
            OptimizeMergeTreeMap::globalDefsToMiniZinc(numTrees, domainSize_, globalDefs);
            try {
                outfile << globalDefs << std::endl;
            } catch (const std::ofstream::failure &e) {
                LogProcessorError("Error during writing: " << allTimeStepsModelFile_);
                LogProcessorError("  Error Code: " << e.code() << "    . " << e.what());
                return std::make_pair(landscapeDecisions, df);
            }
        }

        std::vector<float> times;
        std::vector<float> objectives;
        // Detailed stats
        std::vector<int> nodes;
        std::vector<float> solveTimes;
        std::vector<float> flatTimes;
        if (!allTimeSteps) {
            times = std::vector<float>(numTrees - 1);
            objectives = std::vector<float>(numTrees - 1);
            if (detailedStatsEnabled) {
                nodes = std::vector<int>(numTrees - 1);
                solveTimes = std::vector<float>(numTrees - 1);
                flatTimes = std::vector<float>(numTrees - 1);
            }
        }

        std::string lastTreeAndDecision = "";
        size_t sumNumSubTrees = 0;
        size_t maxNumSubTrees = 0;

        for (size_t i = 0; i < numTrees - 1; i++) {
            if (stop) return std::make_pair(landscapeDecisions, df);
            std::ostringstream stream;
            if (!allTimeSteps) {
                stream << "domainSize = " << domainSize_ << ";\n";
            }

            std::string modelVariables(stream.str());

            const mtmutils::SubTreeInfo &infoFirst = subTreeInfos_[i];
            if (i == 0) {
                const topology::ContourTreeData &treeDataFirst = *treesData.get()->at(i).get();
                std::string modelTreeFirst;
                OptimizeMergeTreeMap::treeToMiniZinc(treeDataFirst, infoFirst, 0, firstFixed,
                                                     allTimeSteps, modelTreeFirst);
                modelVariables += modelTreeFirst;
                sumNumSubTrees += infoFirst.numArcs;
                if (infoFirst.numArcs > maxNumSubTrees) maxNumSubTrees = infoFirst.numArcs;
            } else {
                if (!allTimeSteps_) {
                    modelVariables += lastTreeAndDecision;
                }
            }

            const topology::ContourTreeData &treeDataSecond = *treesData.get()->at(i + 1).get();
            const mtmutils::SubTreeInfo &infoSecond = subTreeInfos_[i + 1];
            std::string modelTreeSecond;
            OptimizeMergeTreeMap::treeToMiniZinc(treeDataSecond, infoSecond,
                                                 allTimeSteps ? i + 1 : 1, false, allTimeSteps,
                                                 modelTreeSecond);
            sumNumSubTrees += infoSecond.numArcs;
            if (infoSecond.numArcs > maxNumSubTrees) maxNumSubTrees = infoSecond.numArcs;
            modelVariables += modelTreeSecond;

            const mtmutils::Overlap &currentNDOverlap = nDOverlap_[i];
            std::string modelOverlap;
            OptimizeMergeTreeMap::overlapToMiniZinc(infoFirst, infoSecond, i, currentNDOverlap,
                                                    allTimeSteps, modelOverlap);
            modelVariables += modelOverlap;

            if (allTimeSteps_.get()) {
                try {
                    outfile << modelVariables << std::endl;
                } catch (const std::ofstream::failure &e) {
                    LogProcessorError("Error during save: " << allTimeStepsModelFile_);
                    LogProcessorError("  Error Code: " << e.code() << "    . " << e.what());
                    return std::make_pair(landscapeDecisions, df);
                }
            }

            if (!allTimeSteps) {
                auto locals = pybind11::globals();

                // Input
                locals["modelFile"] = pybind11::cast(modelFile_);
                locals["modelVariables"] = pybind11::cast(modelVariables);
                if (i == timestep && modelDebuggingEnabled) modelInputString_ = modelVariables;
                // LogProcessorInfo("Model variables at " << i << ":\n" << modelVariables);
                // First time step is always fixed for any time step after 0
                locals["firstFixed"] = (i == 0) ? pybind11::cast(firstFixed) : pybind11::cast(true);
                locals["solverTimeoutEnabled"] = pybind11::cast(solverTimeoutEnabled);
                locals["solverTimeoutSeconds"] = pybind11::cast(solverTimeoutSeconds);

                std::vector<int> &decisions1 = landscapeDecisions->at(i + 1);

                try {
                    script_.run(locals, [&](pybind11::dict dict) {
                        float obj = pybind11::cast<float>(dict["obj"]);
                        objectives[i] = obj;
                        totalObjective_ += obj;
                        float time = pybind11::cast<float>(dict["time"]);
                        times[i] = time;
                        if (detailedStatsEnabled) {
                            nodes[i] = pybind11::cast<int>(dict["nodes"]);
                            flatTimes[i] = pybind11::cast<float>(dict["flatTime"]);
                            solveTimes[i] = pybind11::cast<float>(dict["solveTime"]);
                        }
                        if (i == 0) {
                            auto pyDecisions_0 =
                                pybind11::cast<pybind11::list>(dict["decisions_0"]);
                            landscapeDecisions->at(i) = std::vector<int>(infoFirst.numArcs);
                            std::vector<int> &decisions0 = landscapeDecisions->at(i);
                            // Ids within MiniZinc are reversed, thus iterate in opposite order
                            for (size_t d0 = 0; d0 < decisions0.size(); d0++) {
                                decisions0[d0] =
                                    pybind11::cast<int>(pyDecisions_0[infoFirst.numArcs - 1 - d0]);
                            }
                        }
                        auto pyDecisions_1 = pybind11::cast<pybind11::list>(dict["decisions_1"]);
                        landscapeDecisions->at(i + 1) = std::vector<int>(infoSecond.numArcs);
                        // Ids within MiniZinc are reversed, thus iterate in opposite order
                        for (size_t d1 = 0; d1 < decisions1.size(); d1++) {
                            decisions1[d1] =
                                pybind11::cast<int>(pyDecisions_1[infoSecond.numArcs - 1 - d1]);
                        }
                    });
                } catch (std::exception &e) {
                    LogError(e.what())
                }

                if (decisions1.empty()) {
                    LogProcessorError("Landscape Optimization failed.");
                    return std::make_pair(landscapeDecisions, df);
                }

                // decisions1 will be the input "decisions_0" for the next timestep
                std::ostringstream streamDecision;
                streamDecision << "decisions_0 = [";
                for (ttk::ftm::idSuperArc s = 0; s < infoSecond.numArcs; s++) {
                    streamDecision
                        << ((decisions1[infoSecond.numArcs - 1 - s] == 0) ? "false" : "true");
                    if (s != infoSecond.numArcs - 1) streamDecision << ", ";
                }
                // Closing brace for decision
                streamDecision << "];\n";
                lastTreeAndDecision = std::regex_replace(modelTreeSecond, std::regex("_1"), "_0") +
                                      streamDecision.str();

                // LogProcessorInfo("TimeStep " << i << ": " << streamDecision.str());
                // Update progress
                progress((i + 1) / static_cast<float>(numTrees - 1));
            }
        }

        // Do the entire optimization for all time steps
        if (allTimeSteps) {
            std::string objectiveAndOptimize;
            OptimizeMergeTreeMap::objectiveToMiniZInc(numTrees, sumNumSubTrees, maxNumSubTrees,
                                                      subTreeInfos_, objectiveAndOptimize);
            try {
                outfile << objectiveAndOptimize << std::endl;
            } catch (const std::ofstream::failure &e) {
                LogProcessorError("Error during writing: " << allTimeStepsModelFile_);
                LogProcessorError("  Error Code: " << e.code() << "    . " << e.what());
                return std::make_pair(landscapeDecisions, df);
            }
            outfile.close();

            auto locals = pybind11::globals();

            // Input
            locals["modelFile"] = pybind11::cast(allTimeStepsModelFile_.get());
            locals["solverTimeoutEnabled"] = pybind11::cast(solverTimeoutEnabled);
            locals["solverTimeoutSeconds"] = pybind11::cast(solverTimeoutSeconds);

            try {
                scriptAll_.run(locals, [&](pybind11::dict dict) {
                    float obj = pybind11::cast<float>(dict["obj"]);
                    objectives.push_back(obj);
                    totalObjective_ += obj;
                    float time = pybind11::cast<float>(dict["time"]);
                    times.push_back(time);
                    if (detailedStatsEnabled) {
                        int node = pybind11::cast<int>(dict["nodes"]);
                        nodes.push_back(node);
                        float flatTime = pybind11::cast<float>(dict["flatTime"]);
                        flatTimes.push_back(flatTime);
                        float solveTime = pybind11::cast<float>(dict["solveTime"]);
                        solveTimes.push_back(solveTime);
                    }

                    auto pyDecisions = pybind11::cast<pybind11::list>(dict["decisions_all"]);
                    // LogProcessorInfo(pyDecisions.size());

                    size_t runningIndex = 0;
                    for (size_t currentIndexPerTree = 0; currentIndexPerTree < maxNumSubTrees;
                         currentIndexPerTree++) {
                        // Go through all trees, add index if the tree has that amount of subtrees
                        // (May not be ideal, perhaps we need to do it by layer)
                        for (size_t i = 0; i < numTrees; i++) {
                            auto numArcs = subTreeInfos_[i].numArcs;
                            auto &decisions_i = landscapeDecisions->at(i);
                            if (currentIndexPerTree == 0) {
                                decisions_i = std::vector<int>(numArcs);
                            }
                            if (currentIndexPerTree < numArcs) {
                                // Ids within MiniZinc are reversed, thus iterate in opposite order
                                decisions_i[numArcs - 1 - currentIndexPerTree] =
                                    pybind11::cast<int>(pyDecisions[runningIndex]);
                                runningIndex++;
                            }
                        }
                    }
                });
            } catch (std::exception &e) {
                LogError(e.what())
            }
        }

        df->addColumnFromBuffer("Time in Seconds", util::makeBuffer<float>(std::move(times)));
        df->addColumnFromBuffer("Objective", util::makeBuffer<float>(std::move(objectives)));
        if (detailedStatsEnabled) {
            df->addColumnFromBuffer("Flat Time in Seconds",
                                    util::makeBuffer<float>(std::move(flatTimes)));
            df->addColumnFromBuffer("Solve Time in Seconds",
                                    util::makeBuffer<float>(std::move(solveTimes)));
            df->addColumnFromBuffer("Evaluated Nodes", util::makeBuffer<int>(std::move(nodes)));
        }

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
        if (modelDebuggingEnabled_) modelInput_.set(modelInputString_);
    });
}

void OptimizeMergeTreeMap::treeToMiniZinc(const topology::ContourTreeData &treeData,
                                          const mtmutils::SubTreeInfo &info, const size_t i,
                                          const bool firstFixed,
                                          const bool includeDefsAndConstraints,
                                          std::string &modelTree) {
    std::ostringstream stream;

    // Indices in MiniZinc start at 1, but here at 0 -> we need to always add 1
    if (includeDefsAndConstraints) {
        stream << "int: ";
    }
    stream << "rootArcIdx_" << i << " = " << info.numArcs - 1 - info.rootArcIdx + 1 << ";\n";
    if (includeDefsAndConstraints) {
        stream << "int: ";
    }
    stream << "numSubTrees_" << i << " = " << info.numArcs << ";\n";
    if (includeDefsAndConstraints) {
        stream << "array[1..numSubTrees_" << i << "] of set of 1..numSubTrees_" << i << ": ";
    }
    stream << "tree_" << i << " = [\n";
    auto tree = treeData.getTree();
    // Traverse in reverse order
    for (ttk::ftm::idSuperArc s = 0; s < info.numArcs; s++) {
        auto arc = tree->getSuperArc(info.numArcs - 1 - s);
        auto childId = arc->getDownNodeId();
        auto child = tree->getNode(childId);
        auto numChildren = child->getNumberOfDownSuperArcs();
        stream << "{";
        std::vector<ttk::ftm::idSuperArc> children;
        for (size_t childS = 0; childS < numChildren; childS++) {
            // Adding one for one-based index
            // stream << info.numArcs - 1 - child->getDownSuperArcId(childS) + 1;
            children.push_back(info.numArcs - child->getDownSuperArcId(childS));
        }
        std::sort(children.begin(), children.end());
        for (size_t childS = 0; childS < numChildren; childS++) {
            stream << children[childS];
            if (childS != numChildren - 1) stream << ", ";
        }
        stream << "}";
        if (s != info.numArcs - 1) stream << ",\n";
    }
    // Closing brace for tree structure
    stream << "\n];\n";

    // Starting arcSizes array
    if (includeDefsAndConstraints) {
        stream << "array[1..numSubTrees_" << i << "] of DomainSize: ";
    }
    stream << "arcSizes_" << i << " = [\n";
    for (ttk::ftm::idSuperArc s = 0; s < info.numArcs; s++) {
        stream << info.arcSizes[info.numArcs - 1 - s];
        if (s != info.numArcs - 1) stream << ",\n";
    }
    // Closing brace for arc sizes
    stream << "\n];\n";

    if (includeDefsAndConstraints) {
        stream << "array[1..numSubTrees_" << i << "] of DomainSize: ";
    }
    stream << "subTreeSizes_" << i << " = [\n";
    for (ttk::ftm::idSuperArc s = 0; s < info.numArcs; s++) {
        stream << info.treeSizes[info.numArcs - 1 - s];
        if (s != info.numArcs - 1) stream << ",\n";
    }
    // Closing brace for tree sizes
    stream << "\n];\n";

    if (includeDefsAndConstraints) {
        stream << "\narray[1..numSubTrees_" << i << ", 1..4] of var DomainSize : limits_" << i
               << ";\narray[1..numSubTrees_" << i << "] of var bool : decisions_" << i
               << ";\nconstraint treeLimits(tree_" << i << ", arcSizes_" << i << ", subTreeSizes_"
               << i << ", limits_" << i << ", decisions_" << i << ", rootArcIdx_" << i
               << ", numSubTrees_" << i << ");\n";
        if (!firstFixed) {
            stream << "constraint forall(idx in 1..numSubTrees_" << i << ")(noChildren(tree_" << i
                   << ", decisions_" << i << ", idx));\n";
        }
    }

    if (firstFixed)
        stream << "decisions_" << i << " = [false | i in 1..numSubTrees_" << i << "];\n";
    if (includeDefsAndConstraints) stream << "\n";

    modelTree = stream.str();
}

void OptimizeMergeTreeMap::overlapToMiniZinc(const mtmutils::SubTreeInfo &infoFirst,
                                             const mtmutils::SubTreeInfo &infoSecond,
                                             const size_t i,
                                             const mtmutils::Overlap &currentNDOverlap,
                                             const bool includeDefsAndConstraints,
                                             std::string &modelOverlap) {

    // Use 0 as the index if we are not in the mode for all time steps
    size_t j = includeDefsAndConstraints ? i : 0;

    std::ostringstream stream;
    if (includeDefsAndConstraints) {
        stream << "array[1..numSubTrees_" << j << " , 1..numSubTrees_" << j + 1
               << "] of DomainSize: ";
    }
    stream << "overlapND_" << j << "_" << j + 1 << " = array2d(1..numSubTrees_" << j
           << " , 1..numSubTrees_" << j + 1 << ", [\n";
    for (size_t subTreeIdFirst = 0; subTreeIdFirst < infoFirst.numArcs; subTreeIdFirst++) {
        auto output = std::vector<float>(infoSecond.numArcs);
        for (size_t subTreeIdSecond = 0; subTreeIdSecond < infoSecond.numArcs; subTreeIdSecond++) {
            stream << currentNDOverlap.treeTreeCount[infoFirst.numArcs - 1 - subTreeIdFirst]
                                                    [infoSecond.numArcs - 1 - subTreeIdSecond];
            if (subTreeIdFirst != infoFirst.numArcs - 1 ||
                subTreeIdSecond != infoSecond.numArcs - 1)
                stream << ",";
        }
        stream << "\n";
    }
    stream << "]);\n";

    if (includeDefsAndConstraints) {
        stream << "\narray[1..numSubTrees_" << j << " , 1..numSubTrees_" << j + 1
               << "] of var DomainSize: overlap1D_" << j << "_" << j + 1
               << " = array2d(1..numSubTrees_" << j << " , 1..numSubTrees_" << j + 1
               << ", [\noverlap1DCounts(limits_" << j << "[i, 1], limits_" << j << "[i, 2], limits_"
               << j + 1 << "[j, 1], limits_" << j + 1 << "[j, 2]) |\ni in 1..numSubTrees_" << j
               << ",\nj in 1..numSubTrees_" << j + 1 << "\n]);\n";
    }

    modelOverlap = stream.str();
}

void OptimizeMergeTreeMap::globalDefsToMiniZinc(const size_t numTrees, const size_t domainSize,
                                                std::string &globalDefs) {

    std::ostringstream stream;

    stream << "include \"globals.mzn\";\n";
    stream << "include \"traversalOrder.mzn\";\n\n";

    stream << "int: numTimeSteps = " << numTrees << ";\n";
    stream << "int: domainSize = " << domainSize << ";\n";
    stream << "set of int: DomainSize = 0..domainSize;\n"
           << "set of int: DiffDomain = -domainSize..domainSize;\n\n";

    globalDefs = stream.str();
}

void OptimizeMergeTreeMap::objectiveToMiniZInc(
    const size_t numTrees, const size_t sumNumSubTrees, const size_t maxNumSubTrees,
    const std::vector<mtmutils::SubTreeInfo> &subTreeInfos_, std::string &objectiveAndOptimize) {

    std::ostringstream stream;

    stream << "array[1..numTimeSteps-1] of var 0.0..domainSize: objPerTimeStep;\n";

    for (size_t i = 0; i < numTrees - 1; i++) {
        // arrays in MiniZinc start with index 1
        stream << "constraint objPerTimeStep[" << i + 1 << "] = normOfDiffCounts(numSubTrees_" << i
               << ", numSubTrees_" << i + 1 << ", overlapND_" << i << "_" << i + 1 << ", overlap1D_"
               << i << "_" << i + 1 << ");\n";
    }
    stream << "\n\n";

    for (size_t i = 0; i < numTrees; i++) {
        // arrays in MiniZinc start with index 1
    }
    stream << "var float: obj = sum(i in 1..numTimeSteps-1)(objPerTimeStep[i]);\n\n";

    stream << "array[1.." << sumNumSubTrees << "] of var bool: decisions_all ::add_to_output =[\n";
    // Todo: Export decisions by layer? just interleaving?
    size_t runningIndex = 0;
    for (size_t currentIndexPerTree = 0; currentIndexPerTree < maxNumSubTrees;
         currentIndexPerTree++) {
        // Go through all trees, add index if the tree has that amount of subtrees
        // (May not be ideal, perhaps we need to do it by layer)
        for (size_t i = 0; i < numTrees; i++) {
            auto numArcs = subTreeInfos_[i].numArcs;
            if (currentIndexPerTree < numArcs) {
                stream << "decisions_" << i << "[" << currentIndexPerTree + 1 << "]"
                       << (runningIndex == sumNumSubTrees - 1 ? "" : ",");
                runningIndex++;
            }
        }
        stream << (runningIndex == sumNumSubTrees - 1 ? "" : "\n");
    }

    stream << "];\n\n";

    stream << "solve::bool_search(decisions_all, input_order, indomain_max) minimize obj;\n";

    objectiveAndOptimize = stream.str();
}

}  // namespace inviwo
