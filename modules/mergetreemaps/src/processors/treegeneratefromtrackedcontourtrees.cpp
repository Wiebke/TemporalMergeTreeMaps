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

#include <inviwo/mergetreemaps/processors/treegeneratefromtrackedcontourtrees.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo TreeGenerateFromTrackedContourTrees::processorInfo_{
    "org.inviwo.TreeGenerateFromTrackedContourTrees",  // Class identifier
    "Tree Generate From Tracked Contour Trees",        // Display name
    "Temporal Tree",                                   // Category
    CodeState::Experimental,                           // Code state
    Tags::None,                                        // Tags
};
const ProcessorInfo TreeGenerateFromTrackedContourTrees::getProcessorInfo() const {
    return processorInfo_;
}

TreeGenerateFromTrackedContourTrees::TreeGenerateFromTrackedContourTrees()
    : PoolProcessor()
    , treesInport_("contourTrees")
    , trackingInport_("tracking")
    , treeOutport_("outTree") {

    addPort(treesInport_);
    addPort(trackingInport_);
    addPort(treeOutport_);
}

void TreeGenerateFromTrackedContourTrees::process() {

    auto tracking = trackingInport_.getData();
    auto trees = treesInport_.getData();

    if (trees->size() == 0) return;

    topology::TreeType type = (*trees->at(0)).type;
    if (!(type == topology::TreeType::Join || type == topology::TreeType::Split)) {
        LogProcessorError("Treetype" << toString(type) << " not suppoerted ");
        return;
    }

    using Result = std::shared_ptr<kth::TemporalTree>;

    auto compute = [trees, tracking](pool::Stop stop, pool::Progress progress) -> Result {
        if (stop) return nullptr;
        Result temporalTree = std::make_shared<kth::TemporalTree>(kth::TemporalTree());

        // Per timestep map from vertexId to temporal tree node Id
        size_t numTrees = trees->size();

        // Assume the same treetype for all trees
        topology::TreeType type = (*trees->at(0)).type;

        std::vector<std::map<ttk::SimplexId, size_t>> indexMap(numTrees);

        kth::TemporalTree::TNode globalRoot{"root", {{0, 0.0f}, {numTrees - 1, 0.0f}}};
        temporalTree->addNode(globalRoot);

        size_t time = 0;
        for (const auto currentTree : *trees.get()) {

            auto tree = currentTree->getTree();
            // Join tree -> one max that is root
            // Split tree -> one min that is root
            ttk::ftm::idNode rootNodeId;
            const size_t numNodes = tree->getNumberOfNodes();
            for (ttk::ftm::idNode i = 0; i < numNodes; ++i) {
                auto node = tree->getNode(i);
                const bool up = node->getNumberOfUpSuperArcs() > 0;
                const bool down = node->getNumberOfDownSuperArcs() > 0;

                if (down && !up) {
                    rootNodeId = i;
                    break;
                }
            }

            std::function<size_t(const ttk::ftm::idNode&)> traverseTree;
            traverseTree = [&](const ttk::ftm::idNode& nodeId) -> size_t {
                auto node = tree->getNode(nodeId);
                /* Create temporal tree node for current node at the current time */
                ttk::SimplexId vertexId = node->getVertexId();
                std::string name(std::to_string(vertexId) + "_" + std::to_string(time));
                // Note: Value is set to 0.0 here, could be size below
                kth::TemporalTree::TNode temporalNode{name, {{time, 0.0f}}};

                size_t nodeIndex = temporalTree->addNode(temporalNode);

                indexMap[time][vertexId] = nodeIndex;

                auto numDown = node->getNumberOfDownSuperArcs();
                // Leaf
                if (numDown == 0) {
                    return nodeIndex;
                }

                // Not leaf
                for (ttk::ftm::idSuperArc i = 0; i < numDown; ++i) {
                    auto superArcId = node->getDownSuperArcId(i);
                    auto superArc = tree->getSuperArc(superArcId);
                    auto downNodeId = superArc->getDownNodeId();
                    size_t childIndex = traverseTree(downNodeId);
                    temporalTree->addHierarchyEdge(nodeIndex, childIndex);
                }

                return nodeIndex;
            };

            size_t currentRootIndex = traverseTree(rootNodeId);
            temporalTree->addHierarchyEdge(0, currentRootIndex);
            time++;

            progress((time + 1) / numTrees * 0.75f);
        }

        size_t rows = tracking->getNumberOfRows();
        size_t columns = tracking->getNumberOfColumns();

        if (columns != 9) {
            return nullptr;
        }

        // Columns
        // 0 - Index
        // 1 - Vertex Id Start
        std::shared_ptr<const Column> columnIdStart = tracking->getColumn(1);
        // 2 - Critical Type Start
        std::shared_ptr<const Column> columnCpTypeStart = tracking->getColumn(2);
        // 3 - Time Start
        std::shared_ptr<const Column> columnTimeStart = tracking->getColumn(3);
        // 4 - Vertex Id End
        std::shared_ptr<const Column> columnIdEnd = tracking->getColumn(4);
        // 5 - Critical Type End
        std::shared_ptr<const Column> columnCpTypeEnd = tracking->getColumn(5);
        // 6 - Time End
        std::shared_ptr<const Column> columnTimeEnd = tracking->getColumn(6);
        // 7 - Cost
        // 8 - Component Id

        for (size_t r(0); r < rows; r++) {

            ttk::CriticalType cpType =
                ttk::CriticalType(std::stoi(columnCpTypeStart->getAsString(r)));

            // Join tree -> all leaves are minima
            // Split tree -> all leaves are maxima
            // Disregard the respective other
            if ((type == topology::TreeType::Join && cpType != ttk::CriticalType::Local_minimum) ||
                (type == topology::TreeType::Split && cpType != ttk::CriticalType::Local_maximum)) {
                continue;
            }

            // Outside temporal range
            auto tStart = std::stoi(columnTimeStart->getAsString(r));
            if (tStart < 0 || tStart >= numTrees) {
                continue;
            }
            auto vertexIdStart = std::stoi(columnIdStart->getAsString(r));

            auto itStart = indexMap[tStart].find(vertexIdStart);
            if (itStart == indexMap[tStart].end()) {
                continue;
            }
            size_t temporalTreeNodeIdStart = itStart->second;

            auto tEnd = std::stoi(columnTimeEnd->getAsString(r));
            if (tEnd < 0 || tEnd >= numTrees) {
                continue;
            }
            auto vertexIdEnd = std::stoi(columnIdEnd->getAsString(r));

            auto itEnd = indexMap[tEnd].find(vertexIdEnd);
            if (itEnd == indexMap[tEnd].end()) {
                continue;
            }
            size_t temporalTreeNodeIdEnd = itEnd->second;

            temporalTree->nodes[temporalTreeNodeIdStart].values[tEnd] = 0.0f;
            temporalTree->addTemporalEdge(temporalTreeNodeIdStart, temporalTreeNodeIdEnd);
        }

        temporalTree->aggregation = kth::TemporalTree::TTreeAggregation::FullyDeaggregated;
        temporalTree->computeReverseEdges();

        return temporalTree;
    };

    treeOutport_.clear();
    dispatchOne(compute, [this](Result result) {
        treeOutport_.setData(result);

        if (result) {
            // Check for untracked leafes
            const auto& leaves = result->getLeaves();
            size_t numOnlyLeaves = 0;
            for (auto leafIndex : leaves) {
                if (!result->hasTemporalSuccessors(leafIndex) &&
                    !result->hasTemporalPredecessorsWithReverse(leafIndex)) {
                    numOnlyLeaves++;
                    const auto& leaf = result->nodes[leafIndex];
                    LogProcessorWarn("Leaf " << leaf.name << " [" << leaf.startTime() << ","
                                             << leaf.endTime() << "] is not tracked.");
                }
            }
            if (numOnlyLeaves != 0)
                LogProcessorWarn("There are " << numOnlyLeaves
                                              << " leaves that are not part of a correspondance.");
        } else {
            LogProcessorError("Error occurred while computing temporal tree from tracking.");
		}

        newResults();
    });
}

}  // namespace inviwo
