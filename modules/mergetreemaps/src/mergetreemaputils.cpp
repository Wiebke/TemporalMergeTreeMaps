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

#include <inviwo/mergetreemaps/mergetreemaputils.h>
#include <modules/temporaltreemaps/datastructures/treeorder.h>
#include <inviwo/core/util/zip.h>

namespace inviwo {

namespace mtmutils {

bool getVertexOrderPerTimeStepFromTemporal(const kth::TemporalTree& temporalTree,
                                           const ContourTreeSequence& contourTrees,
                                           std::vector<VertexOrderMap>& vertexOrderMapPerTimeStep,
                                           size_t timeStep) {

    bool singleTimeStep = contourTrees.size() == 1;

    // Create orderMap
    kth::TemporalTree::TTreeOrderMap temporalTreeOrderMap;
    if (temporalTree.order.empty()) return false;
    kth::treeorder::toOrderMap(temporalTreeOrderMap, temporalTree.order);

    std::vector<std::map<ttk::SimplexId, size_t>> indexMapPerTimestep(contourTrees.size());

    auto& temporalNodes = temporalTree.nodes;
    // Populate map with all nodes that have the correct time step in their name
    size_t currentId = 0;
    for (auto temporalNode : temporalNodes) {
        // Extract time step and vertex id of temporal node
        size_t temporalNodeVertexId;
        size_t temporalNodeTimeStep;
        if (std::string("root").compare(temporalNode.name) == 0) {
            currentId++;
            continue;
        }
        const char* name = temporalNode.name.c_str();
        char* token = std::strtok(const_cast<char*>(name), "_");
        IVW_ASSERT(token, "Temporal Tree node names are not following expected pattern");
        if (token) {
            temporalNodeVertexId = std::stoi(token);
            token = std::strtok(NULL, "_");
            IVW_ASSERT(token, "Temporal Tree node names are not following expected pattern");
            if (token) {
                temporalNodeTimeStep = std::stoi(token);
                if (singleTimeStep) {
                    if (temporalNodeTimeStep == timeStep) {
                        indexMapPerTimestep[0].emplace(temporalNodeVertexId, currentId);
                    }
                } else {
                    indexMapPerTimestep[temporalNodeTimeStep].emplace(temporalNodeVertexId,
                                                                      currentId);
                }
            } else {
                // Error state
                return false;
            }
        } else {
            // Error state
            false;
        }
        currentId++;
    }

    size_t currentTimeStep = 0;
    for (auto contourTree : contourTrees) {
        auto tree = contourTree->getTree();
        auto numNodes = tree->getNumberOfNodes();

        auto& currentVertexOrderMap = vertexOrderMapPerTimeStep[currentTimeStep];
        // Check if the map contains all nodes, meaning that the temporal tree contains all nodes
        for (ttk::ftm::idNode i = 0; i < numNodes; ++i) {
            const auto& node = tree->getNode(i);
            auto vertexId = node->getVertexId();
            const std::map<ttk::SimplexId, size_t>& indexMap = indexMapPerTimestep[currentTimeStep];
            auto it = indexMap.find(vertexId);
            IVW_ASSERT(it != indexMap.end(), "Temporal Tree is missing node with vertex Id "
                                                 << vertexId << " at time step " << currentTimeStep
                                                 << ".")
            if (it == indexMap.end()) {
                return false;
            }
            // Node is a leaf, will later be turned into a node vertex map
            if (node->getNumberOfDownSuperArcs() == 0) {
                currentVertexOrderMap[vertexId] = temporalTreeOrderMap[it->second];
            }
        }
        // Reduce order, so it has at most number of leaves
        std::vector<std::pair<ttk::SimplexId, size_t>> orderReductionHelper;
        std::copy(currentVertexOrderMap.begin(), currentVertexOrderMap.end(),
                  std::back_inserter<std::vector<std::pair<ttk::SimplexId, size_t>>>(
                      orderReductionHelper));

        // sort the vector by increasing the order of its pair's second value
        // if the second value is equal, order by the pair's first value
        std::sort(orderReductionHelper.begin(), orderReductionHelper.end(),
                  [](const std::pair<ttk::SimplexId, size_t>& a,
                     const std::pair<ttk::SimplexId, size_t>& b) {
                      if (a.second != b.second) {
                          return a.second < b.second;
                      }

                      return a.first < b.first;
                  });

        size_t orderCounter = 0;
        for (const auto& vertexOrderPair : orderReductionHelper) {
            currentVertexOrderMap[vertexOrderPair.first] = orderCounter;
            orderCounter++;
        }

        currentTimeStep++;
    }
    return true;
}

std::pair<int, size_t> accumulateOrderAndSizes(ttk::ftm::FTMTree_MT* tree,
                                               const VertexOrderMap& vertexOrderMap,
                                               const ttk::ftm::idNode& nodeId,
                                               std::vector<size_t>& sizeBelow,
                                               std::map<ttk::ftm::idNode, size_t>& orderMap) {

    size_t numNodes = tree->getNumberOfNodes();
    auto node = tree->getNode(nodeId);
    auto numDown = node->getNumberOfDownSuperArcs();
    size_t minOrder = numNodes;
    if (numDown == 0 && !vertexOrderMap.empty()) {
        // We have checked before that all these map accesses are possible
        minOrder = vertexOrderMap.at(node->getVertexId());
    }
    // Not leaf
    for (ttk::ftm::idSuperArc i = 0; i < numDown; ++i) {
        auto superArcId = node->getDownSuperArcId(i);
        auto superArc = tree->getSuperArc(superArcId);
        auto sizeAndOrder = accumulateOrderAndSizes(tree, vertexOrderMap, superArc->getDownNodeId(),
                                                    sizeBelow, orderMap);
        sizeBelow[nodeId] += superArc->regionSize() + sizeAndOrder.first;
        minOrder = std::min(minOrder, sizeAndOrder.second);
    }

    // Add this node (also means leafs will have value 1)
    sizeBelow[nodeId] += 1;
    if (!vertexOrderMap.empty()) orderMap.emplace(nodeId, minOrder);

    return std::make_pair(sizeBelow[nodeId], minOrder);
}

void IVW_MODULE_MERGETREEMAPS_API accumulateSizesPerTimeStep(
    ttk::ftm::FTMTree_MT* tree, const TimeStepMapper& pointMapping, const size_t numTimeSteps,
    const ttk::ftm::idNode& nodeId, std::vector<std::vector<size_t>>& sizeBelowNode,
    std::vector<std::vector<size_t>>& sizeOnEdge) {

    auto node = tree->getNode(nodeId);
    auto numDown = node->getNumberOfDownSuperArcs();
    auto vertexId = node->getVertexId();
    // Assumes that steps in z are timesteps
    auto timeStep = pointMapping.getTimeStep(vertexId);

    // Not leaf
    for (ttk::ftm::idSuperArc i = 0; i < numDown; ++i) {
        auto superArcId = node->getDownSuperArcId(i);
        auto superArc = tree->getSuperArc(superArcId);
        // Iterate over superArc
        auto arcSize = superArc->regionSize();
        for (size_t i = 0; i < arcSize; i++) {
            vertexId = superArc->getRegularNodeId(i);
            auto timeStepSuperArc = pointMapping.getTimeStep(vertexId);
            sizeOnEdge[timeStepSuperArc][superArcId]++;
        }
        // Go further with children
        auto downNodeId = superArc->getDownNodeId();
        accumulateSizesPerTimeStep(tree, pointMapping, numTimeSteps, superArc->getDownNodeId(),
                                   sizeBelowNode, sizeOnEdge);
        // Update based on children
        for (size_t t = 0; t < numTimeSteps; t++) {
            sizeBelowNode[t][nodeId] += sizeOnEdge[t][superArcId] + sizeBelowNode[t][downNodeId];
        }
    }

    // Add this node (also means leafs will have value 1)
    sizeBelowNode[timeStep][nodeId] += 1;
}

ttk::ftm::idNode findRoot(ttk::ftm::FTMTree_MT* tree) {
    /*
    auto roots = tree->getRoots();
    std::sort(roots.begin(), roots.end());
    roots.erase(std::unique(roots.begin(), roots.end()), roots.end());
    LogProcessorInfo(roots.size());
    for (auto r : roots) LogProcessorInfo(r << " " << tree->getNode(r)->getVertexId())
    The vector always contains 10 zeros, where 0
    does not correspond to the global max or min
    => This array does not give us anything, we need to search for the root ourselves
    */

    const size_t numNodes = tree->getNumberOfNodes();
    // Join tree -> one max that is root
    // Split tree -> one min that is root
    // Leaves seem to be filled correctly
    ttk::ftm::idNode rootNodeId = 0;
    for (ttk::ftm::idNode i = 0; i < numNodes; ++i) {
        auto node = tree->getNode(i);
        const bool up = node->getNumberOfUpSuperArcs() > 0;
        const bool down = node->getNumberOfDownSuperArcs() > 0;

        // The root node is the node where there is only a down arc and no up arc
        if (down && !up) {
            rootNodeId = i;
            break;
        }
    }
    return rootNodeId;
}

void landscapeSortSuperArcsOrder(ttk::ftm::FTMTree_MT* tree,
                                 const std::map<ttk::ftm::idNode, size_t>& orderMap,
                                 std::vector<ttk::ftm::idSuperArc>& superArcIds) {
    // No order map is given (e.g. tree not connected) or the node was a leaf
    if (orderMap.empty() || superArcIds.empty()) return;
    // auto first = superArcIds[0];
    std::sort(
        superArcIds.begin(), superArcIds.end(),
        [&orderMap, &tree](const ttk::ftm::idSuperArc a, const ttk::ftm::idSuperArc b) -> bool {
            auto itIndexA = orderMap.find(tree->getSuperArc(a)->getDownNodeId());
            auto itIndexB = orderMap.find(tree->getSuperArc(b)->getDownNodeId());
            // If either index a or b are not in the order, we cannot sort the given
            ivwAssert(itIndexA != orderMap.end() && itIndexB != orderMap.end(),
                      "We cannot sort nodes when their indices are not in the given order.");
            return itIndexA->second < itIndexB->second;
        });
    // bool changed = (first != superArcIds[0]);
}

void IVW_MODULE_MERGETREEMAPS_API landscapeSortSuperArcsDecisions(
    ttk::ftm::FTMTree_MT* tree, const ttk::ftm::idSuperArc& idParentArc,
    const std::vector<int>& decisions, std::vector<ttk::ftm::idSuperArc>& superArcIds) {
    // auto first = superArcIds[0];
    std::sort(superArcIds.begin(), superArcIds.end(),
              [&decisions, &tree, &idParentArc](const ttk::ftm::idSuperArc& a,
                                                const ttk::ftm::idSuperArc& b) -> bool {
                  // Decision array can be empty if the port was not conntected
                  // Otherwise: If decision is false -> order by id
                  // if decision is true -> order inversely by id
                  const bool& decision =
                      (decisions.size() < idParentArc || decisions[idParentArc] == 0);
                  return (decision ? a < b : a > b);
              });
    // bool changed = (first != superArcIds[0]);
}

void landscapeSortSuperArcsSize(ttk::ftm::FTMTree_MT* tree, const std::vector<size_t>& sizeBelow,
                                std::vector<ttk::ftm::idSuperArc>& superArcIds) {
    // No order map is given (e.g. tree not connected) or the node was a leaf
    if (superArcIds.empty()) return;
    // auto first = superArcIds[0];
    std::sort(
        superArcIds.begin(), superArcIds.end(),
        [&sizeBelow, &tree](const ttk::ftm::idSuperArc a, const ttk::ftm::idSuperArc b) -> bool {
            auto sizeA = sizeBelow[tree->getSuperArc(a)->getDownNodeId()];
            auto sizeB = sizeBelow[tree->getSuperArc(b)->getDownNodeId()];
            return sizeA < sizeB;
        });
    // bool changed = (first != superArcIds[0]);
}

void landscapeSortSuperArcs(const SuperArcOrderMode& orderMode, ttk::ftm::FTMTree_MT* tree,
                            const std::vector<size_t>& sizeBelow,
                            const ttk::ftm::idSuperArc& idParentArc,
                            const std::vector<int>& decisions,
                            const std::map<ttk::ftm::idNode, size_t>& orderMap,
                            std::vector<ttk::ftm::idSuperArc>& superArcIds) {

    switch (orderMode) {
        case SuperArcOrderMode::None:
            return;
        case SuperArcOrderMode::ById:
            std::sort(superArcIds.begin(), superArcIds.end());
            break;
        case SuperArcOrderMode::ByIdReverse:
            std::sort(superArcIds.rbegin(), superArcIds.rend());
            break;
        case SuperArcOrderMode::BySize:
            landscapeSortSuperArcsSize(tree, sizeBelow, superArcIds);
            break;
        case SuperArcOrderMode::ByDecision:
            landscapeSortSuperArcsDecisions(tree, idParentArc, decisions, superArcIds);
            break;
        case SuperArcOrderMode::ByTemporalTree:
            landscapeSortSuperArcsOrder(tree, orderMap, superArcIds);
            break;
        default:
            return;
    };
}

void accumulateArcs(ttk::ftm::FTMTree_MT* tree, const ttk::ftm::idSuperArc& superArcId,
                    SubTreeInfo& info) {

    auto superArc = tree->getSuperArc(superArcId);
    info.subTreeArcs[superArcId] = {superArcId};
    // Arc size is region size + end node ()
    info.arcSizes[superArcId] = superArc->regionSize() + 1;
    auto nodeId = superArc->getDownNodeId();
    auto node = tree->getNode(nodeId);
    auto numDown = node->getNumberOfDownSuperArcs();
    info.treeSizes[superArcId] = superArc->regionSize() + 1;

    // Not leaf
    for (ttk::ftm::idSuperArc i = 0; i < numDown; ++i) {
        auto superArcIdChild = node->getDownSuperArcId(i);
        accumulateArcs(tree, superArcIdChild, info);
        auto& subtreeArcsChild = info.subTreeArcs[superArcIdChild];
        std::copy(subtreeArcsChild.begin(), subtreeArcsChild.end(),
                  std::back_inserter(info.subTreeArcs[superArcId]));
        info.treeSizes[superArcId] += info.treeSizes[superArcIdChild];
    }
}

void accumulateArcsTree(ttk::ftm::FTMTree_MT* tree, SubTreeInfo& info) {

    auto rootIdx = findRoot(tree);
    auto root = tree->getNode(rootIdx);
    auto numDown = root->getNumberOfDownSuperArcs();
    auto rootArcIdx = root->getDownSuperArcId(0);

    if (info.numArcs == 0) {
        auto numArcs = tree->getNumberOfSuperArcs();
        info.numArcs = numArcs;
        info.rootArcIdx = rootArcIdx;
        info.arcSizes = std::vector<size_t>(numArcs);
        info.treeSizes = std::vector<size_t>(numArcs);
        info.subTreeArcs = std::vector<std::vector<ttk::ftm::idSuperArc>>(numArcs);
    }

    for (ttk::ftm::idSuperArc i = 0; i < numDown; ++i) {
        auto superArcIdChild = root->getDownSuperArcId(i);
        accumulateArcs(tree, superArcIdChild, info);
    }

    info.arcSizes[root->getDownSuperArcId(0)]++;
    info.treeSizes[root->getDownSuperArcId(0)]++;
}

void computeOverlap1D(const TemplateColumn<int>& superArcLandscapeFirst,
                      const TemplateColumn<int>& mapPositionFirst, const SubTreeInfo& infoFirst,
                      const TemplateColumn<int>& superArcLandscapeSecond,
                      const TemplateColumn<int>& mapPositionSecond, const SubTreeInfo& infoSecond,
                      Overlap& overlap) {

    const size_t numSuperArcsFirst = infoFirst.numArcs;
    const size_t numSuperArcsSecond = infoSecond.numArcs;
    overlap = Overlap(numSuperArcsFirst, numSuperArcsSecond);

    // Simply zipping the columns is not possible since index != mapPosition
    // We have to build a map first
    size_t numVertices = superArcLandscapeFirst.getSize();

    std::vector<int> superArcMapFirst(numVertices);
    std::vector<int> superArcMapSecond(numVertices);

    for (size_t i = 0; i < numVertices; i++) {
        auto mapPosFirst = mapPositionFirst.get(i);
        superArcMapFirst[mapPosFirst] = superArcLandscapeFirst.get(i);
        auto mapPosSecond = mapPositionSecond.get(i);
        superArcMapSecond[mapPosSecond] = superArcLandscapeSecond.get(i);
    }

    for (size_t mapPos = 0; mapPos < numVertices; mapPos++) {
        auto superArcIdFirst = superArcMapFirst[mapPos];
        auto superArcIdSecond = superArcMapSecond[mapPos];
        overlap.arcArcCount[superArcIdFirst][superArcIdSecond]++;
    }

    overlap.computeTreeTreeCount(infoFirst, infoSecond);
}

void computeOverlapND(const topology::ContourTreeData& treeDataFirst, const SubTreeInfo& infoFirst,
                      const topology::ContourTreeData& treeDataSecond,
                      const SubTreeInfo& infoSecond, Overlap& overlap) {

    /* Will only be evaluated *once* per data set */

    auto treeFirst = treeDataFirst.getTree();
    auto treeSecond = treeDataSecond.getTree();

    const size_t numSuperArcsFirst = treeFirst->getNumberOfSuperArcs();
    const size_t numSuperArcsSecond = treeSecond->getNumberOfSuperArcs();
    overlap = Overlap(numSuperArcsFirst, numSuperArcsSecond);

    // both contourTrees are from the same domain, i.e. have the same number of vertices, count
    // simply count overlap
    size_t numVertices = treeFirst->getNumberOfVertices();
    for (ttk::SimplexId vert = 0; vert < numVertices; vert++) {
        int superArcIdFirst;
        if (treeFirst->isCorrespondingArc(vert))
            superArcIdFirst = treeFirst->getCorrespondingSuperArcId(vert);
        else {
            auto nodeId = treeFirst->getCorrespondingNodeId(vert);
            auto node = treeFirst->getNode(nodeId);
            if (node->getNumberOfUpSuperArcs() >= 1) {
                // Assign leaf and saddles to their up superArcId (there will only be one)
                superArcIdFirst = node->getUpSuperArcId(0);
            } else {
                // This is the case *only* for the root node, assign it to the first one
                superArcIdFirst = node->getDownSuperArcId(0);
            }
        }
        int superArcIdSecond;
        if (treeSecond->isCorrespondingArc(vert))
            superArcIdSecond = treeSecond->getCorrespondingSuperArcId(vert);
        else {
            auto nodeId = treeSecond->getCorrespondingNodeId(vert);
            auto node = treeSecond->getNode(nodeId);
            if (node->getNumberOfUpSuperArcs() >= 1) {
                // Assign leaf and saddles to their up superArcId (there will only be one)
                superArcIdSecond = node->getUpSuperArcId(0);
            } else {
                // This is the case *only* for the root node, assign it to the first one
                superArcIdSecond = node->getDownSuperArcId(0);
            }
        }
        overlap.arcArcCount[superArcIdFirst][superArcIdSecond]++;
    }

    overlap.computeTreeTreeCount(infoFirst, infoSecond);
}

void accumulateLimits(ttk::ftm::FTMTree_MT* tree, const SubTreeInfo& info,
                      const SuperArcOrderMode& orderMode,
                      const std::map<ttk::ftm::idNode, size_t>& orderMap,
                      const std::vector<int>& decisions, const ttk::ftm::idSuperArc& superArcId,
                      const size_t& lastAssigned, std::vector<LandscapeInfo>& landscapeInfos) {

    auto& thisArcInfo = landscapeInfos[superArcId];
    thisArcInfo.computeLimit(lastAssigned, info.arcSizes[superArcId], info.treeSizes[superArcId]);

    auto superArc = tree->getSuperArc(superArcId);
    auto nodeId = superArc->getDownNodeId();
    auto node = tree->getNode(nodeId);
    auto numDown = node->getNumberOfDownSuperArcs();

    // Leaf, termina
    if (numDown == 0) {
        // Assign next space
        thisArcInfo.setDownNodePos(thisArcInfo.getInnerLimMin() + 1);
        return;
    }

    std::vector<ttk::ftm::idSuperArc> superArcIds;
    for (ttk::ftm::idSuperArc i = 0; i < numDown; ++i) {
        auto superArcId = node->getDownSuperArcId(i);
        superArcIds.push_back(superArcId);
    }
    ttk::ftm::idSuperArc upArcId =
        node->getNumberOfSuperArcs() != 0 ? node->getUpSuperArcId(0) : tree->getNumberOfSuperArcs();
    // std::sort(superArcIds.begin(), superArcIds.end());
    if (superArcIds.size() > 1)
        mtmutils::landscapeSortSuperArcs(orderMode, tree, info.treeSizes, upArcId, decisions,
                                         orderMap, superArcIds);

    size_t lastAssignedChildren = thisArcInfo.getInnerLimMin();
    size_t arcCounter = 0;
    for (ttk::ftm::idSuperArc superArcIdChild : superArcIds) {
        accumulateLimits(tree, info, orderMode, orderMap, decisions, superArcIdChild,
                         lastAssignedChildren, landscapeInfos);
        auto& childArcInfo = landscapeInfos[superArcIdChild];
        lastAssignedChildren = childArcInfo.getOuterLimMax();
        if (arcCounter == 0) {
            // Place downnode after first child
            lastAssignedChildren++;
            thisArcInfo.setDownNodePos(lastAssignedChildren);
        }
        arcCounter++;
    }
}

void accumulateLimitsTree(ttk::ftm::FTMTree_MT* tree, const SubTreeInfo& info,
                          const SuperArcOrderMode& orderMode,
                          const std::map<ttk::ftm::idNode, size_t>& orderMap,
                          const std::vector<int>& decisions,
                          std::vector<LandscapeInfo>& landscapeInfos) {

    auto numArcs = tree->getNumberOfSuperArcs();
    landscapeInfos = std::vector<LandscapeInfo>(numArcs);

    auto roodId = findRoot(tree);
    auto root = tree->getNode(roodId);
    auto rootNumDown = root->getNumberOfDownSuperArcs();
    auto rootSuperArcId = root->getDownSuperArcId(0);

    if (rootNumDown > 1) return;

    auto& rootArcInfo = landscapeInfos[rootSuperArcId];
    rootArcInfo.computeLimitRoot(info.arcSizes[rootSuperArcId], info.treeSizes[rootSuperArcId]);

    auto rootSuperArc = tree->getSuperArc(rootSuperArcId);
    auto nextNodeId = rootSuperArc->getDownNodeId();
    auto nextNode = tree->getNode(nextNodeId);
    auto numDown = nextNode->getNumberOfDownSuperArcs();

    std::vector<ttk::ftm::idSuperArc> superArcIds;
    for (ttk::ftm::idSuperArc i = 0; i < numDown; ++i) {
        auto superArcId = nextNode->getDownSuperArcId(i);
        superArcIds.push_back(superArcId);
    }
    if (superArcIds.size() > 1)
        mtmutils::landscapeSortSuperArcs(orderMode, tree, info.treeSizes, rootSuperArcId, decisions,
                                         orderMap, superArcIds);

    size_t lastAssignedChildren = rootArcInfo.getInnerLimMin();
    size_t arcCounter = 0;
    for (ttk::ftm::idSuperArc superArcIdChild : superArcIds) {
        accumulateLimits(tree, info, orderMode, orderMap, decisions, superArcIdChild,
                         lastAssignedChildren, landscapeInfos);
        auto& childArcInfo = landscapeInfos[superArcIdChild];
        lastAssignedChildren = childArcInfo.getOuterLimMax();
        if (arcCounter == 0) {
            // Place down node after first child
            lastAssignedChildren++;
            rootArcInfo.setDownNodePos(lastAssignedChildren);
        }
        arcCounter++;
    }
}

float objectiveForSeveralArcs(const SubTreeInfo& infoFromPrev,
                              const std::vector<LandscapeInfo>& landscapeFromPrev,
                              const Overlap& overlapND, const bool useCounts,
                              const std::vector<ttk::ftm::idSuperArc>& superArcIds,
                              const std::vector<LandscapeInfo>& landscapes) {
    float objective = 0.0f;
    for (size_t superArcIdPrev = 0; superArcIdPrev < infoFromPrev.numArcs; superArcIdPrev++) {
        auto& landscapeFromPrevCurr = landscapeFromPrev[superArcIdPrev];
        for (size_t i = 0; i < superArcIds.size(); i++) {
            float objectiveCurrent = 0.0f;
            const auto& superArcId = superArcIds[i];
            const auto& landscape = landscapes[i];
            if (useCounts) {
                objectiveCurrent =
                    static_cast<float>(overlapND.treeTreeCount[superArcIdPrev][superArcId]) -
                    LandscapeInfo::overLapCountTrees(landscapeFromPrevCurr, landscape);
            } else {
                size_t superArcPrevSize = landscapeFromPrevCurr.getSubTreeSize();
                size_t arcSize = landscape.getSubTreeSize();
                float objectiveNDCurrent =
                    static_cast<float>(overlapND.treeTreeCount[superArcIdPrev][superArcId]);
                objectiveCurrent =
                    objectiveNDCurrent / (superArcPrevSize + arcSize - objectiveNDCurrent) -
                    LandscapeInfo::overLapFractionTrees(landscapeFromPrevCurr, landscape);
            }
            objective += objectiveCurrent * objectiveCurrent;
        }
    }
    return objective;
}

float decideGreedy(ttk::ftm::FTMTree_MT* tree, const SubTreeInfo& infoFromPrev,
                   const std::vector<LandscapeInfo>& landscapeFromPrev, const SubTreeInfo& info,
                   const Overlap& overlapND, const ttk::ftm::idSuperArc& superArcId,
                   const bool useCounts, const bool chooseWorse, std::vector<int>& decisions,
                   std::vector<LandscapeInfo>& landscapes) {

    auto& thisArcInfo = landscapes[superArcId];
    auto superArc = tree->getSuperArc(superArcId);
    auto nodeId = superArc->getDownNodeId();
    auto nextNode = tree->getNode(nodeId);
    auto numDown = nextNode->getNumberOfDownSuperArcs();

    std::vector<ttk::ftm::idSuperArc> superArcIds;
    for (ttk::ftm::idSuperArc i = 0; i < numDown; ++i) {
        auto superArcId = nextNode->getDownSuperArcId(i);
        superArcIds.push_back(superArcId);
    }
    // Sort here since MiniZinc will do this internally too
    std::sort(superArcIds.begin(), superArcIds.end());

    // Leaf
    if (superArcIds.size() == 0) {
        thisArcInfo.setDownNodePos(thisArcInfo.getInnerLimMin() + 1);
        return 0.0f;
    }

    // Try out the two ordering options
    std::vector<LandscapeInfo> landscapesDecisionTrue(superArcIds.size());
    std::vector<LandscapeInfo> landscapesDecisionFalse(superArcIds.size());
    std::vector<ttk::ftm::idSuperArc> superArcIdsReverse(superArcIds.size());
    std::reverse_copy(superArcIds.begin(), superArcIds.end(), superArcIdsReverse.begin());

    size_t lastAssigned = thisArcInfo.getInnerLimMin();
    size_t lastAssignedRev = thisArcInfo.getInnerLimMin();

    // If decision is false: Order by Id, if it is true, order inversly by id
    for (size_t i = 0; i < superArcIds.size(); i++) {
        // True -> superarcs in reverse order
        const auto& superArcIdCurrRev = superArcIdsReverse[i];
        LandscapeInfo& landscapeCurrTrue = landscapesDecisionTrue[i];
        landscapeCurrTrue.computeLimit(lastAssigned, info.arcSizes[superArcIdCurrRev],
                                       info.treeSizes[superArcIdCurrRev]);
        lastAssigned = landscapeCurrTrue.getOuterLimMax();
        // False -> superarcs in order
        const auto& superArcIdCurr = superArcIds[i];
        LandscapeInfo& landscapeCurrFalse = landscapesDecisionFalse[i];
        landscapeCurrFalse.computeLimit(lastAssignedRev, info.arcSizes[superArcIdCurr],
                                        info.treeSizes[superArcIdCurr]);
        lastAssignedRev = landscapeCurrFalse.getOuterLimMax();
        // Reserve space for downNode of current arc
        if (i == 0) {
            lastAssigned++;
            lastAssignedRev++;
        }
    }

    float objTrue = objectiveForSeveralArcs(infoFromPrev, landscapeFromPrev, overlapND, useCounts,
                                            superArcIdsReverse, landscapesDecisionTrue);

    float objFalse = objectiveForSeveralArcs(infoFromPrev, landscapeFromPrev, overlapND, useCounts,
                                             superArcIds, landscapesDecisionFalse);
    int decision = objTrue < objFalse;

	// Choose opposite
    if (chooseWorse) decision = !decision;
    decisions[superArcId] = decision;

    if (decision) {
        for (size_t i = 0; i < superArcIds.size(); i++) {
            const auto& superArcIdCurrRev = superArcIdsReverse[i];
            landscapes[superArcIdCurrRev] = landscapesDecisionTrue[i];
        }
    } else {
        for (size_t i = 0; i < superArcIds.size(); i++) {
            const auto& superArcIdCurr = superArcIds[i];
            landscapes[superArcIdCurr] = landscapesDecisionFalse[i];
        }
    }

    // Set downnode after the first child (we reserved this space earlier)
    thisArcInfo.setDownNodePos((decision ? landscapesDecisionTrue[0].getOuterLimMax() + 1
                                         : landscapesDecisionFalse[0].getOuterLimMax() + 1));

    float objectiveChildren = 0.0;
    for (size_t i = 0; i < superArcIds.size(); i++) {
        objectiveChildren +=
            decideGreedy(tree, infoFromPrev, landscapeFromPrev, info, overlapND, superArcIds[i],
                         useCounts, chooseWorse, decisions, landscapes);
    }

    return (decision ? objTrue : objFalse) + objectiveChildren;
}

float decideGreedyTree(ttk::ftm::FTMTree_MT* tree, const SubTreeInfo& infoFromPrev,
                       const std::vector<LandscapeInfo>& landscapeFromPrev, const SubTreeInfo& info,
                       const Overlap& overlapND, const bool useCounts, const bool chooseWorse,
                       std::vector<int>& decisions, std::vector<LandscapeInfo>& landscapes) {

    auto numArcs = tree->getNumberOfSuperArcs();
    landscapes = std::vector<LandscapeInfo>(numArcs);

    auto roodId = findRoot(tree);
    auto root = tree->getNode(roodId);
    auto rootNumDown = root->getNumberOfDownSuperArcs();
    auto rootSuperArcId = root->getDownSuperArcId(0);

    if (rootNumDown > 1) return pow(tree->getNumberOfSuperArcs() * tree->getNumberOfVertices(), 2);

    auto& rootArcInfo = landscapes[rootSuperArcId];
    rootArcInfo.computeLimitRoot(info.arcSizes[rootSuperArcId], info.treeSizes[rootSuperArcId]);

    // The overlap with the root is equal to the subtree size for each arc
    // This is true for ND and 1D, obj value will thus be 0
    // float objRoot = ...

    return decideGreedy(tree, infoFromPrev, landscapeFromPrev, info, overlapND, rootSuperArcId,
                        useCounts, chooseWorse, decisions, landscapes);
}

void computeOverlap1D(const std::vector<LandscapeInfo>& superArcLandscapesFirst,
                      const SubTreeInfo& infoFirst,
                      const std::vector<LandscapeInfo>& superArcLandscapesSecond,
                      const SubTreeInfo& infoSecond, Overlap& overlap,
                      bool checkOverLapCountTrees) {
    const size_t numSuperArcsFirst = infoFirst.numArcs;
    const size_t numSuperArcsSecond = infoSecond.numArcs;
    overlap = Overlap(numSuperArcsFirst, numSuperArcsSecond);

    if (superArcLandscapesFirst.size() != numSuperArcsFirst) return;
    if (superArcLandscapesSecond.size() != numSuperArcsSecond) return;

    // Overlap the computed limits for the arcs
    for (size_t superArcIdFirst = 0; superArcIdFirst < numSuperArcsFirst; superArcIdFirst++) {
        for (size_t superArcIdSecond = 0; superArcIdSecond < numSuperArcsSecond;
             superArcIdSecond++) {
            overlap.arcArcCount[superArcIdFirst][superArcIdSecond] =
                LandscapeInfo::overLapCountArcs(superArcLandscapesFirst[superArcIdFirst],
                                                superArcLandscapesSecond[superArcIdSecond]);
        }
    }

    overlap.computeTreeTreeCount(infoFirst, infoSecond);

    if (checkOverLapCountTrees) {
        // Check if this is the same as computing tree overlaps directly
        for (size_t superArcIdFirst = 0; superArcIdFirst < numSuperArcsFirst; superArcIdFirst++) {
            for (size_t superArcIdSecond = 0; superArcIdSecond < numSuperArcsSecond;
                 superArcIdSecond++) {
                auto treeOverlap =
                    LandscapeInfo::overLapCountTrees(superArcLandscapesFirst[superArcIdFirst],
                                                     superArcLandscapesSecond[superArcIdSecond]);
                if (treeOverlap != overlap.treeTreeCount[superArcIdFirst][superArcIdSecond]) {
                    std::cout << "Different overlaps counted.";
                }
            }
        }
    }
}

}  // namespace mtmutils

}  // namespace inviwo
