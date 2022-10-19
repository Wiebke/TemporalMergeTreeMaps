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

#include <inviwo/mergetreemaps/processors/evaluatemergetreemap.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo EvaluateMergeTreeMap::processorInfo_{
    "org.inviwo.EvaluateMergeTreeMap",  // Class identifier
    "Evaluate Merge Tree Map",          // Display name
    "Merge Tree Maps",                  // Category
    CodeState::Experimental,            // Code state
    Tags::None,                         // Tags
};
const ProcessorInfo EvaluateMergeTreeMap::getProcessorInfo() const { return processorInfo_; }

EvaluateMergeTreeMap::EvaluateMergeTreeMap()
    : Processor()
    , orderMode_("orderMode", "Order Mode")
    , evalValue_("evalValue", "Objective Value", 0.0, 0.0, std::numeric_limits<double>::max(), 0.01,
                 InvalidationLevel::Valid, PropertySemantics::Text)
    , checkAgainstCalculated_("checkAgainstCalc", "Check Against Actual Landscape", false)
    , additionalOutput_("additionalOutput", "Additional Output")
    , createMatrixOutput_("createMatrixOutput", "Create Overlap Matrices")
    , useCounts_("useCounts", "Use Counts, not Fractions", false)
    , normalizePerTObjective_("normalizeObj", "Normalize Objective", false)
    , matrixOutputMode_("evalMode", "Matrix Output Mode")
    , createOverlapTypeOutput_("createOverlapTypeOutput", "Create Overlap Type Matrices")
    , createTreeInfoOutput_("createTreeInfos", "Create Tree Infos", false)
    , createModelInput_("createModelInput", "Create Model Input", false)
    , modelInput_("modelInput", "Model Input", "", InvalidationLevel::Valid,
                  PropertySemantics::TextEditor)
    , timer_("timer", "Eval Time (s)", 0.f, 0.f, std::numeric_limits<float>::max(), 0.001f,
             InvalidationLevel::Valid, PropertySemantics::Text) {

    addPort(inport_);
    addPort(treesInport_);

    addPort(decisionsInport_);
    decisionsInport_.setOptional(true);
    addPort(temporalTreeInport_);
    temporalTreeInport_.setOptional(true);
    temporalTreeInport_.onConnect([&]() { this->invalidate(InvalidationLevel::InvalidResources); });
    temporalTreeInport_.onDisconnect(
        [&]() { this->invalidate(InvalidationLevel::InvalidResources); });

    addPort(objectivesOutport_);
    addPort(matrixOutport_);
    addPort(treeInfosOutport_);
    addPort(matrixOverlapTypeOutport_);

    addProperties(orderMode_, checkAgainstCalculated_, useCounts_, normalizePerTObjective_,
                  evalValue_, additionalOutput_, timer_);

    orderMode_.addOption("none", "None", mtmutils::SuperArcOrderMode::None);
    orderMode_.addOption("byId", "By Id", mtmutils::SuperArcOrderMode::ById);
    orderMode_.addOption("byIdReverse", "By Reverse Id", mtmutils::SuperArcOrderMode::ByIdReverse);
    orderMode_.addOption("bySize", "Be Size Below", mtmutils::SuperArcOrderMode::BySize);
    orderMode_.addOption("byDecision", "By Left/Right Decision",
                         mtmutils::SuperArcOrderMode::ByDecision);
    orderMode_.addOption("byTemporal", "By Temporal Tree",
                         mtmutils::SuperArcOrderMode::ByTemporalTree);
    orderMode_.setSelectedValue(mtmutils::SuperArcOrderMode::ById);
    orderMode_.setCurrentStateAsDefault();
    orderMode_.onChange([this]() {
        decisionsInport_.setOptional(orderMode_.get() != mtmutils::SuperArcOrderMode::ByDecision);
        temporalTreeInport_.setOptional(orderMode_.get() !=
                                        mtmutils::SuperArcOrderMode::ByTemporalTree);
    });
    evalValue_.setReadOnly(true);
    timer_.setReadOnly(true);

    additionalOutput_.addProperties(createMatrixOutput_, matrixOutputMode_,
                                    createOverlapTypeOutput_, createTreeInfoOutput_,
                                    createModelInput_, modelInput_);
    matrixOutputMode_.addOption("diff", "Difference (nD-1D)", 0);
    matrixOutputMode_.addOption("diffSquared", "Difference^2", 1);
    matrixOutputMode_.addOption("oned", "1D Overlap", 2);
    matrixOutputMode_.addOption("nd", "nD Overlap", 3);
    matrixOutputMode_.setSelectedIndex(0);
    matrixOutputMode_.setCurrentStateAsDefault();
    matrixOutputMode_.visibilityDependsOn(createMatrixOutput_,
                                          [](const auto &p) { return p.get(); });
    createOverlapTypeOutput_.visibilityDependsOn(createMatrixOutput_,
                                                 [](const auto &p) { return p.get(); });
    modelInput_.visibilityDependsOn(createModelInput_, [](const auto &p) { return p.get(); });
}

void EvaluateMergeTreeMap::process() {

    performanceTimer_.Reset();

    const auto treesData = treesInport_.getData();
    const auto landscapeData = inport_.getData();

    auto numTrees = treesData.get()->size();
    if (landscapeData->size() != numTrees) {
        LogProcessorWarn("Number of time steps differ.");
        return;
    }

    if (treesInport_.isChanged()) {

        subTreeInfos_ = std::vector<mtmutils::SubTreeInfo>(numTrees);
        nDOverlap_ = std::vector<mtmutils::Overlap>(numTrees - 1);
        domainSize_ = treesData.get()->at(0).get()->getTree()->getNumberOfVertices();

        // pre-compute all subtree infos, will be used by both overlap methods
        for (size_t i = 0; i < numTrees; i++) {
            const topology::ContourTreeData &treeData = *treesData.get()->at(i).get();
            mtmutils::accumulateArcsTree(treeData.getTree(), subTreeInfos_[i]);
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

    if (createTreeInfoOutput_.get()) {
        auto outTreeInfos = std::make_shared<DataFrameSequence>(DataFrameSequence(numTrees));

        for (size_t i = 0; i < numTrees; i++) {
            if (createTreeInfoOutput_.get()) {
                outTreeInfos->at(i) = std::make_shared<DataFrame>();
                auto &dataframe = outTreeInfos->at(i);
                std::vector<int> arcSizes(subTreeInfos_[i].numArcs);
                for (size_t arcId = 0; arcId < subTreeInfos_[i].numArcs; arcId++) {
                    arcSizes[arcId] = static_cast<int>(subTreeInfos_[i].arcSizes[arcId]);
                }
                dataframe->addColumnFromBuffer("Arc Sizes",
                                               util::makeBuffer<int>(std::move(arcSizes)));
                dataframe->updateIndexBuffer();
            }
        }
        treeInfosOutport_.setData(outTreeInfos);
    }

    if (temporalTreeInport_.isConnected() && temporalTreeInport_.getData() &&
        temporalTreeInport_.isChanged()) {

        vertexOrderMapPerTimeStep_ = std::vector<VertexOrderMap>(treesData->size());

        auto temporalTree = temporalTreeInport_.getData();
        const auto contourTrees = *treesData.get();

        // Get order map
        if (temporalTree->order.empty()) {
            LogProcessorWarn("No order on leaves is given.");
        }

        // indexmap is initialized in the function
        bool success = mtmutils::getVertexOrderPerTimeStepFromTemporal(
            *temporalTree.get(), contourTrees, vertexOrderMapPerTimeStep_);
        if (!success) {
            LogProcessorError(
                "Creating node index map between contour and temporal tree was not successful.");
            return;
        }
    }

    oneDOverlapFullLandscape_ = std::vector<mtmutils::Overlap>(numTrees - 1);

    if (checkAgainstCalculated_.get()) {

        for (size_t i = 0; i < numTrees - 1; i++) {
            auto superArcLandscapeFirst = std::dynamic_pointer_cast<const TemplateColumn<int>>(
                landscapeData->at(i)->getColumn("Super Arc ID"));
            if (!superArcLandscapeFirst) return;
            auto mapPositionFirst = std::dynamic_pointer_cast<const TemplateColumn<int>>(
                landscapeData->at(i)->getColumn("Map Position"));
            if (!mapPositionFirst) return;
            const mtmutils::SubTreeInfo &infoFirst = subTreeInfos_[i];
            auto superArcLandscapeSecond = std::dynamic_pointer_cast<const TemplateColumn<int>>(
                landscapeData->at(i + 1)->getColumn("Super Arc ID"));
            if (!superArcLandscapeSecond) return;
            auto mapPositionSecond = std::dynamic_pointer_cast<const TemplateColumn<int>>(
                landscapeData->at(i + 1)->getColumn("Map Position"));
            if (!mapPositionSecond) return;

            const mtmutils::SubTreeInfo &infoSecond = subTreeInfos_[i + 1];
            mtmutils::computeOverlap1D(*superArcLandscapeFirst, *mapPositionFirst, infoFirst,
                                       *superArcLandscapeSecond, *mapPositionSecond, infoSecond,
                                       oneDOverlapFullLandscape_[i]);
        }
    }

    oneDOverlapLandscapeRanges_ = std::vector<mtmutils::Overlap>(numTrees - 1);
    landscapeInfos_ = std::vector<std::vector<mtmutils::LandscapeInfo>>(numTrees);

    if (decisionsInport_.isConnected() && decisionsInport_.getData()) {
        auto decisionsPerStep = decisionsInport_.getData();
        if (decisionsPerStep->size() < treesData->size()) {
            LogProcessorError(
                "Array of decisions per time step does not contain enough time steps.");
            return;
        }
    }

    // Compute landscape limits without actually computing the landscape
    for (size_t i = 0; i < numTrees; i++) {
        std::vector<int> decisions;
        const topology::ContourTreeData &treeData = *treesData.get()->at(i).get();
        if (decisionsInport_.isConnected() && decisionsInport_.getData()) {
            auto decisionsPerStep = decisionsInport_.getData();
            decisions = decisionsInport_.getData()->at(i);
            if (decisions.size() != treeData.getTree()->getNumberOfSuperArcs()) {
                LogProcessorError("Array with decisions does not contain enough super arcs.");
                return;
            }
        }

        const auto &currentVertexOrderMap = (vertexOrderMapPerTimeStep_.size() > i)
                                                ? vertexOrderMapPerTimeStep_[i]
                                                : VertexOrderMap();
        std::map<ttk::ftm::idNode, size_t> orderMap;
        if (orderMode_.get() == mtmutils::SuperArcOrderMode::ByTemporalTree) {
            auto tree = treeData.getTree();
            std::vector<size_t> sizeBelow(tree->getNumberOfNodes(), 0);
            mtmutils::accumulateOrderAndSizes(tree, currentVertexOrderMap, mtmutils::findRoot(tree),
                                              sizeBelow, orderMap);
        }

        mtmutils::accumulateLimitsTree(treeData.getTree(), subTreeInfos_[i], orderMode_.get(),
                                       orderMap, decisions, landscapeInfos_[i]);
    }
    // compute overlap
    for (size_t i = 0; i < numTrees - 1; i++) {
        const mtmutils::SubTreeInfo &infoFirst = subTreeInfos_[i];
        const std::vector<mtmutils::LandscapeInfo> &landscapeInfoFirst = landscapeInfos_[i];
        const mtmutils::SubTreeInfo &infoSecond = subTreeInfos_[i + 1];
        const std::vector<mtmutils::LandscapeInfo> &landscapeInfoSecond = landscapeInfos_[i + 1];
        mtmutils::computeOverlap1D(landscapeInfoFirst, infoFirst, landscapeInfoSecond, infoSecond,
                                   oneDOverlapLandscapeRanges_[i]);
    }

    auto outMatrices = std::make_shared<DataFrameSequence>(DataFrameSequence(numTrees - 1));
    auto outOverlapTypeMatrices =
        std::make_shared<DataFrameSequence>(DataFrameSequence(numTrees - 1));
    auto outObjectives = std::make_shared<DataFrame>();
    std::vector<float> objectives(numTrees - 1);

    double evalValue = 0.0;
    for (size_t i = 0; i < numTrees - 1; i++) {
        double diffSum = 0.0;
        // Initialize data frames for overlap (even if they are not needed
        // Little overhead while everything is empty
        outMatrices->at(i) = std::make_shared<DataFrame>();
        auto &dataframe = outMatrices->at(i);
        outOverlapTypeMatrices->at(i) = std::make_shared<DataFrame>();
        auto &overlapTypeDataFrame = outOverlapTypeMatrices->at(i);
        const mtmutils::SubTreeInfo &infoFirst = subTreeInfos_[i];
        const mtmutils::SubTreeInfo &infoSecond = subTreeInfos_[i + 1];
        const mtmutils::Overlap &currentNDOverlap = nDOverlap_[i];

        // Overlap fraction normalized by subtree size for nD
        std::vector<std::vector<float>> nDOverlapTreetree;
        if (!useCounts_.get())
            currentNDOverlap.computeTreeTreeFraction(infoFirst.treeSizes, infoSecond.treeSizes,
                                                     nDOverlapTreetree);

        const mtmutils::Overlap &current1DOverlapFullLandscape = oneDOverlapFullLandscape_[i];
        // Overlap fraction normalized by subtree size fpr 1D with full landscape
        std::vector<std::vector<float>> oneDOverlapFullLandscapeTreetree;
        if (!useCounts_.get() && checkAgainstCalculated_.get())
            current1DOverlapFullLandscape.computeTreeTreeFraction(
                infoFirst.treeSizes, infoSecond.treeSizes, oneDOverlapFullLandscapeTreetree);

        const mtmutils::Overlap &current1DOverlapLandscapeRanges = oneDOverlapLandscapeRanges_[i];
        // Overlap fraction normalized by subtree size fpr 1D with landscape ranges
        std::vector<std::vector<float>> oneDOverlapLandscapeRangesTreetree;
        if (!useCounts_.get())
            current1DOverlapLandscapeRanges.computeTreeTreeFraction(
                infoFirst.treeSizes, infoSecond.treeSizes, oneDOverlapLandscapeRangesTreetree);

        for (size_t subTreeIdFirst = 0; subTreeIdFirst < infoFirst.numArcs; subTreeIdFirst++) {
            auto output = std::vector<float>(infoSecond.numArcs);
            auto overlapType = std::vector<int>(infoSecond.numArcs);
            for (size_t subTreeIdSecond = 0; subTreeIdSecond < infoSecond.numArcs;
                 subTreeIdSecond++) {
                if (createMatrixOutput_.get() && createOverlapTypeOutput_.get()) {
                    bool hasOverlapND =
                        currentNDOverlap.treeTreeCount[subTreeIdFirst][subTreeIdSecond] > 0;
                    bool hasOverlap1D = current1DOverlapLandscapeRanges
                                            .treeTreeCount[subTreeIdFirst][subTreeIdSecond] > 0;
                    // Encode overlap type with
                    // 00 -> no overlap in either nD or 1D
                    // 01 -> no overlap in nD, but overlap in 1D
                    // 10 -> overlap in nD, but none in 1D
                    // 11 -> overlap in both nD and 1D
                    overlapType[subTreeIdSecond] = 2 * hasOverlapND + hasOverlap1D;
                }

                // Standard output value in objective is squared difference of counts devided by
                // domainSize
                double outputValue =
                    static_cast<double>(
                        currentNDOverlap.treeTreeCount[subTreeIdFirst][subTreeIdSecond]) -
                    current1DOverlapLandscapeRanges.treeTreeCount[subTreeIdFirst][subTreeIdSecond];
                double addition = outputValue / domainSize_;
                // Use fractions instead
                if (!useCounts_) {
                    outputValue =
                        static_cast<double>(nDOverlapTreetree[subTreeIdFirst][subTreeIdSecond]) -
                        oneDOverlapLandscapeRangesTreetree[subTreeIdFirst][subTreeIdSecond];
                    addition = outputValue;
                }
                diffSum += addition * addition;
                outputValue *= outputValue;
                switch (matrixOutputMode_.get()) {
                    case 0:
                        if (useCounts_)
                            outputValue = static_cast<double>(
                                              currentNDOverlap
                                                  .treeTreeCount[subTreeIdFirst][subTreeIdSecond]) -
                                          current1DOverlapLandscapeRanges
                                              .treeTreeCount[subTreeIdFirst][subTreeIdSecond];
                        else
                            outputValue =
                                static_cast<double>(
                                    nDOverlapTreetree[subTreeIdFirst][subTreeIdSecond]) -
                                oneDOverlapLandscapeRangesTreetree[subTreeIdFirst][subTreeIdSecond];
                        break;
                    case 2:  // 1D
                        if (useCounts_)
                            outputValue = current1DOverlapLandscapeRanges
                                              .treeTreeCount[subTreeIdFirst][subTreeIdSecond];
                        else
                            outputValue =
                                oneDOverlapLandscapeRangesTreetree[subTreeIdFirst][subTreeIdSecond];
                        break;
                    case 3:  // nD
                        if (useCounts_)
                            outputValue =
                                currentNDOverlap.treeTreeCount[subTreeIdFirst][subTreeIdSecond];
                        else
                            outputValue = nDOverlapTreetree[subTreeIdFirst][subTreeIdSecond];
                        break;
                    default:
                        // case 1 is already covered with the default objective calculation
                        break;
                }
                output[subTreeIdSecond] = outputValue;

                if (checkAgainstCalculated_.get() &&
                    (useCounts_.get() && current1DOverlapFullLandscape
                                                 .treeTreeCount[subTreeIdFirst][subTreeIdSecond] !=
                                             current1DOverlapLandscapeRanges
                                                 .treeTreeCount[subTreeIdFirst][subTreeIdSecond] ||
                     !useCounts_ &&
                         oneDOverlapFullLandscapeTreetree[subTreeIdFirst][subTreeIdSecond] !=
                             oneDOverlapLandscapeRangesTreetree[subTreeIdFirst][subTreeIdSecond])) {
                    LogProcessorInfo("Disagreement in computations.");
                }
            }

            if (createMatrixOutput_.get())
                dataframe->addColumnFromBuffer(
                    "Arc " + std::to_string(subTreeIdFirst) + " in step " + std::to_string(i),
                    util::makeBuffer<float>(std::move(output)));

            if (createMatrixOutput_.get() && createOverlapTypeOutput_.get())
                overlapTypeDataFrame->addColumnFromBuffer(
                    "Arc " + std::to_string(subTreeIdFirst) + " in step " + std::to_string(i),
                    util::makeBuffer<int>(std::move(overlapType)));
        }
        if (createMatrixOutput_.get()) dataframe->updateIndexBuffer();
        if (createMatrixOutput_.get() && createOverlapTypeOutput_.get())
            dataframe->updateIndexBuffer();
        if (normalizePerTObjective_.get()) {
            diffSum /= infoFirst.numArcs * infoSecond.numArcs;
        }
        evalValue += diffSum;
        objectives[i] = diffSum;
        // LogProcessorInfo("Eval value in step " << std::to_string(i) << ": " << diffSum);
    }

    outObjectives->addColumnFromBuffer("Objective", util::makeBuffer<float>(std::move(objectives)));
    outObjectives->updateIndexBuffer();

    if (createModelInput_.get()) toMiniZinc();
    evalValue_.set(evalValue);
    if (createMatrixOutput_) matrixOutport_.setData(outMatrices);
    if (createMatrixOutput_.get() && createOverlapTypeOutput_.get())
        matrixOverlapTypeOutport_.setData(outOverlapTypeMatrices);
    objectivesOutport_.setData(outObjectives);
    timer_.set(performanceTimer_.ElapsedTimeAndReset());
}

void EvaluateMergeTreeMap::toMiniZinc() {
    std::ostringstream stream;

    const auto treesData = treesInport_.getData();
    if (!treesData) return;
    auto numTrees = treesData.get()->size();

    if (nDOverlap_.size() != numTrees - 1) return;
    if (subTreeInfos_.size() != numTrees) return;

    stream << "include \"globals.mzn\";\n"
           << "include \"traversalOrder.mzn\";\n";

    /* Input for all time steps
    - number of time steps
    - domain size */
    auto domainSize = treesData.get()->at(0).get()->getTree()->getNumberOfVertices();
    stream << "int: numTimeSteps = " << numTrees << ";\n"
           << "int: domainSize = " << domainSize << ";\n"
           << "set of int: DomainSize = 0..domainSize;\n\n";

    /* Input per time step:
    - root index
    - number of subtrees
    - tree structure
    - arc sizes
    - subtree sizes
    Decision variables per time step:
    - Min/Max Outer/Inner Limit for Subtree in 1D landscape
    - Decision at each inner node
    */
    for (size_t i = 0; i < numTrees; i++) {
        const topology::ContourTreeData &treeData = *treesData.get()->at(i).get();
        const mtmutils::SubTreeInfo &info = subTreeInfos_[i];
        // Indices in MiniZinc start at 1, but here at 0 -> we need to always add 1
        stream << "int: rootArcIdx_" << i << " = " << info.numArcs - 1 - info.rootArcIdx + 1
               << ";\n"
               << "int: numSubTrees_" << i << " = " << info.numArcs << ";\n";
        stream << "array[1..numSubTrees_" << i << "] of set of 1..numSubTrees_" << i << ": tree_"
               << i << " = [\n";
        auto tree = treeData.getTree();
        // Traverse in reverse order
        for (ttk::ftm::idSuperArc s = 0; s < info.numArcs; s++) {
            auto arc = tree->getSuperArc(info.numArcs - 1 - s);
            auto childId = arc->getDownNodeId();
            auto child = tree->getNode(childId);
            auto numChildren = child->getNumberOfDownSuperArcs();
            stream << "{";
            for (size_t childS = 0; childS < numChildren; childS++) {
                // Adding one for one-based index
                // stream << info.numArcs - 1 - child->getDownSuperArcId(childS) + 1;
                stream << info.numArcs - child->getDownSuperArcId(childS);

                if (childS != numChildren - 1) stream << ", ";
            }
            stream << "}";
            if (s != info.numArcs - 1) stream << ",\n";
        }
        // Closing brace for tree structure
        stream << "\n];\n";

        // Starting arcSizes array
        stream << "array[1..numSubTrees_" << i << "] of DomainSize : arcSizes_" << i << " = [\n";
        for (ttk::ftm::idSuperArc s = 0; s < info.numArcs; s++) {
            stream << info.arcSizes[info.numArcs - 1 - s];
            if (s != info.numArcs - 1) stream << ",\n";
        }
        // Closing brace for arc sizes
        stream << "\n];\n";
        stream << "array[1..numSubTrees_" << i << "] of DomainSize : subTreeSizes_" << i
               << " = [\n";
        for (ttk::ftm::idSuperArc s = 0; s < info.numArcs; s++) {
            stream << info.treeSizes[info.numArcs - 1 - s];
            if (s != info.numArcs - 1) stream << ",\n";
        }
        // Closing brace for tree sizes
        stream << "\n];\n";

        // Decision variables per timestep
        stream << "array[1..numSubTrees_" << i << ", 1..4] of var DomainSize : limits_" << i
               << ";\n"
               << "array[1..numSubTrees_" << i << "] of var bool: decisions_" << i << ";\n";

        // Constraint for limits
        stream << "constraint treeLimits(tree_" << i << ", arcSizes_" << i << ", subTreeSizes_" << i
               << ", limits_" << i << ", decisions_" << i << ", rootArcIdx_" << i
               << ", numSubTrees_" << i << ");\n";

        stream << "constraint forall(idx in 1..numSubTrees_" << i << ") (noChildren(tree_" << i
               << ", decisions_" << i << ", idx));";

        // Empty line after each time step
        stream << "\n\n";
    }

    stream << "array[1..numTimeSteps-1] of var 0.0..domainSize: objPerTimeStep;\n\n";

    for (size_t i = 0; i < numTrees - 1; i++) {
        stream << "array[1..numSubTrees_" << i << " , 1..numSubTrees_" << i + 1
               << "] of DomainSize: overlapND_" << i << "_" << i + 1 << " = array2d(1..numSubTrees_"
               << i << " , 1..numSubTrees_" << i + 1 << ", [\n";
        const mtmutils::SubTreeInfo &infoFirst = subTreeInfos_[i];
        const mtmutils::SubTreeInfo &infoSecond = subTreeInfos_[i + 1];
        const mtmutils::Overlap &currentNDOverlap = nDOverlap_[i];
        for (size_t subTreeIdFirst = 0; subTreeIdFirst < infoFirst.numArcs; subTreeIdFirst++) {
            auto output = std::vector<float>(infoSecond.numArcs);
            for (size_t subTreeIdSecond = 0; subTreeIdSecond < infoSecond.numArcs;
                 subTreeIdSecond++) {
                stream << currentNDOverlap.treeTreeCount[infoFirst.numArcs - 1 - subTreeIdFirst]
                                                        [infoSecond.numArcs - 1 - subTreeIdSecond];
                if (subTreeIdFirst != infoFirst.numArcs - 1 ||
                    subTreeIdSecond != infoSecond.numArcs - 1)
                    stream << ",";
            }
            stream << "\n";
        }
        stream << "]);\n";

        // Decision variable: Overlap in 1D
        stream << "array[1..numSubTrees_" << i << " , 1..numSubTrees_" << (i + 1)
               << "] of var DomainSize: overlap1D_" << i << "_" << (i + 1)
               << " = array2d(1..numSubTrees_" << i << " , 1..numSubTrees_" << (i + 1) << ", [\n"
               << "overlap1DCounts(limits_" << i << "[i, 1], limits_" << i << "[i, 2], limits_"
               << (i + 1) << "[j, 1], limits_" << (i + 1) << "[j, 2]) |\n"
               << "i in 1..numSubTrees_" << i << ",\n"
               << "j in 1..numSubTrees_" << (i + 1) << "\n"
               << "]);\n";

        stream << "constraint objPerTimeStep[" << (i + 1) << "] = normOfDiffCounts(numSubTrees_"
               << i << ", numSubTrees_" << (i + 1) << ", overlapND_" << i << "_" << (i + 1)
               << ", overlap1D_" << i << "_" << (i + 1) << ");\n\n";
    }

    stream << "var float: obj = sum(i in 1..numTimeSteps-1)(objPerTimeStep[i]);\n\n";
    if (!(stream << "solve minimize obj;\n"))
        LogProcessorInfo("Exporting MiniZinc Model not successful.");

    modelInput_.set(stream.str());
}

}  // namespace inviwo
