/*********************************************************************
 *  Author  : Tino Weinkauf and Wiebke Koepp
 *  Init    : Tuesday, March 27, 2018 - 15:05:01
 *
 *  Project : KTH Inviwo Modules
 *
 *  License : Follows the Inviwo BSD license model
 *********************************************************************
 */

#include <modules/temporaltreemaps/datastructures/constraint.h>

namespace inviwo {
namespace kth {

namespace constraint {
void ConstraintsStatistic::clear() {
    unhappyLeaves.clear();
    std::fill(fulfilledByLevelMergeSplit.begin(), fulfilledByLevelMergeSplit.end(), 0);
    std::fill(fulfilledByLevelHierarchy.begin(), fulfilledByLevelHierarchy.end(), 0);
}

size_t ConstraintsStatistic::numFulfilledHierarchyConstraints() const {
    return numConstraints(fulfilledByLevelHierarchy);
}

size_t ConstraintsStatistic::numFulFilledMergeSplitConstraints() const {
    return numConstraints(fulfilledByLevelMergeSplit);
}

void ConstraintsStatistic::update(const Constraint& constraint) {
    if (constraint.fulfilled) {
        auto& levelStatistic = constraint.type == ConstraintType::Hierarchy
                                   ? fulfilledByLevelHierarchy
                                   : fulfilledByLevelMergeSplit;
        if (levelStatistic.size() < constraint.level + 1) {
            levelStatistic.resize(constraint.level + 1);
            levelStatistic[constraint.level] = 0;
        }
        levelStatistic[constraint.level]++;
    } else {
        unhappyLeaves.insert(constraint.leaves.begin(), constraint.leaves.end());
    }
}

size_t numConstraints(const std::vector<size_t>& numByLevel) {
    size_t sum(0);
    for (auto number : numByLevel) {
        sum += number;
    }
    return sum;
}

bool isFulFilled(Constraint& constraint, std::shared_ptr<const TemporalTree>& tree,
                 const TemporalTree::TTreeOrder& order, const TemporalTree::TTreeOrderMap& orderMap,
                 const std::vector<Constraint>& constraints,
                 const std::vector<size_t>& rivalConstraints) {
    size_t minOrder(order.size());  // numbere of leaves is maximum order
    size_t maxOrder(0);             // 0 is minimum order

    // Record minimum and maximum order index for each leaf
    for (const auto leaf : constraint.leaves) {
        const auto mappedTo = orderMap.at(leaf);
        if (mappedTo < minOrder) minOrder = mappedTo;
        if (mappedTo > maxOrder) maxOrder = mappedTo;
    }

    // Check that the leaves are all together
    int NumOverlap((int)constraint.leaves.size());
    if (int(maxOrder) - int(minOrder) + 1 == NumOverlap) {
        constraint.fulfilled = true;
        return true;
    } else {
        // NumOverlap >= 0 leads to terminating early, so we do not have to go through the whole
        // order if we already know the constraint it not fulfilled
        for (size_t r(minOrder); r <= maxOrder && r < order.size() && NumOverlap >= 0; r++) {
            // A leaf in the drawing area; may not be ours. If it is not ours, but it overlaps
            // then we do not fulfill the hierarchy constraint.
            // For simple temporal overlap there are some considerations to be made:
            // This means that if a node has no children at some point, still drawing children
            // within that parent would warrant a violation of the hierarchy constraint for that
            // parent Therefore we need to split up parent A: splitTemporalLeaves
            // (--- child of parent A, xxx child of parent B)
            // |------||  A has no children  |
            // |-------|   |xxxxxxxx|        |--------|
            // Similarly if other leaves start or end at the exact same time as the parent under
            // investigation ends (for leaf starting) or starts, this will lead to a violation as
            // well
            //     |--------|
            // |---| or     |-----|
            // We need to exclude this case for hierarchy constraints, for merge/split constraints
            // it is still relevant, and needs to be handled elsewhere to not rule out viable
            // solutions
            auto leafId = order[r];
            bool hasPredecessor = tree->getTemporalPredecessorsWithReverse(leafId).size() > 0;
            bool hasSucessors = tree->getTemporalSuccessors(leafId).size() > 0;
            if (constraint.leaves.find(leafId) != constraint.leaves.end() ||
                isOverlappingWithConstraint(tree->nodes[leafId], constraint, hasPredecessor,
                                            hasSucessors)) {
                NumOverlap--;
            }
        }
        ivwAssert(NumOverlap <= 0, "Missed a child? How? Not ok!");
        if (constraint.type == ConstraintType::Hierarchy) {
            if (NumOverlap == 0) {
                constraint.fulfilled = true;
                return true;
            } else {
                constraint.fulfilled = false;
                return false;
            }
        }
    }

    if (NumOverlap == 0 && constraint.type != ConstraintType::Hierarchy) {
        if (rivalConstraints.size() > 0) {
            // Check against crossing of other non-hierarchy constraints
            std::vector<std::pair<size_t, size_t>> edgesConstraint;
            getOrderEdgesFromConstraint(constraint, tree, order, orderMap, std::set<size_t>(),
                                        edgesConstraint);
            for (auto rivalId : rivalConstraints) {
                const auto& rival = constraints[rivalId];
                std::vector<std::pair<size_t, size_t>> edgesRival;
                getOrderEdgesFromConstraint(rival, tree, order, orderMap, constraint.leaves,
                                            edgesRival);
                if (areOrderEdgesCrossing(edgesConstraint, edgesRival)) {
                    constraint.fulfilled = false;
                    return false;
                }
            }
            constraint.fulfilled = true;
            return true;
            // If there are no rivals, there can also not be a crossing
        } else {
            constraint.fulfilled = true;
            return true;
        }
    }

    constraint.fulfilled = false;
    return false;
}

size_t numFulfilledConstraints(std::shared_ptr<const TemporalTree>& tree,
                               const TemporalTree::TTreeOrder& order,
                               const TemporalTree::TTreeOrderMap& orderMap,
                               const std::map<size_t, std::vector<size_t>>& rivalConstraints,
                               std::vector<Constraint>& constraints,
                               ConstraintsStatistic& statistic) {
    size_t numFullfilled = 0;

    // For each constraint, make sure leaves are together at the time of the constraint
    size_t constraintIdx = 0;
    for (auto& constraint : constraints) {
        auto rivalIt = rivalConstraints.find(constraintIdx);
        std::vector<size_t> rivals;
        if (rivalIt != rivalConstraints.end()) rivals = rivalIt->second;
        if (isFulFilled(constraint, tree, order, orderMap, constraints, rivals)) {
            numFullfilled++;
        }
        statistic.update(constraint);
        constraintIdx++;
    }

    return numFullfilled;
}

void extractMergeSplitConstraints(std::shared_ptr<const TemporalTree>& tree, bool leafOnly,
                                  bool removeRedundant, std::vector<Constraint>& constraints,
                                  std::map<size_t, std::vector<size_t>>& rivalConstraints,
                                  std::vector<size_t>& numByLevel) {
    std::vector<std::pair<int, int>> constraintsForAllNodes;

    const size_t numConstraintsAlreadyPresent = constraints.size();

    // Would do similar traversal of constraints
    // const size_t numConstraints = tree->getConstraintClusters(constraintsForAllNodes);
    // constraints.resize(numConstraintsAlreadyPresent + numConstraints);

    std::vector<std::pair<size_t, size_t>> numLeftRight(1, {0, 0});

    // Get reversed time edges
    TemporalTree::TAdjacency reversedEdgesTime = tree->getReverseEdges(tree->edgesTime);
	// Get time points
    std::set<uint64_t> times;
    tree->getTimes(0, times);

    size_t constraintId = 0;
    bool addConstraint = false;
    // Extract splits and direct correspondances
    for (const auto& edge : tree->edgesTime) {
        addConstraint = false;
        Constraint newConstraint;

        // Shorthands
        const size_t idFromLeft(edge.first);

        // Skip if node is not a leaf (for leaf only setting)
        if (leafOnly && !tree->isLeaf(idFromLeft)) continue;

        newConstraint.level = tree->depthWithReverse(idFromLeft);

        std::set<size_t> leaves;
        const auto& time = tree->nodes[idFromLeft].endTime();
        const auto& prevTime = TemporalTree::getPrevTimeFromTimes(time, times);
        tree->getLeaves(idFromLeft, prevTime, prevTime, leaves);
        newConstraint.leaves.insert(leaves.begin(), leaves.end());
        newConstraint.startTime = time;
        newConstraint.endTime = time;
        newConstraint.fulfilled = false;
        numLeftRight[constraintId].first++;

        // Safety
        const std::vector<size_t>& idsToRight = edge.second;
        ivwAssert(!idsToRight.empty(), "Time map is empty.");
        if (idsToRight.empty()) continue;

        // All outgoing nodes must be leaves too in leaf only setting
        if (leafOnly) {
            bool allLeaves = true;
            for (auto idToRight : idsToRight) {
                allLeaves = allLeaves && tree->isLeaf(idToRight);
                if (!allLeaves) break;
            }
            if (!allLeaves) continue;
        }

        // This is an actual split
        if (idsToRight.size() > 1) {
            for (auto idToRight : idsToRight) {
                leaves.clear();
                tree->getLeaves(idToRight, time, time, leaves);
                newConstraint.leaves.insert(leaves.begin(), leaves.end());
                numLeftRight[constraintId].second++;
                addConstraint = true;
            }
        } else {
            auto leftForThatOneRight = reversedEdgesTime.find(idsToRight.front());
            if (leftForThatOneRight != reversedEdgesTime.end()) {
                // If it is a direct correspondance
                if (leftForThatOneRight->second.size() == 1) {
                    leaves.clear();
                    tree->getLeaves(idsToRight.front(), time, time, leaves);
                    newConstraint.leaves.insert(leaves.begin(), leaves.end());
                    numLeftRight[constraintId].second++;
                    addConstraint = true;
                }
            }
        }

        constraintId++;

        if (removeRedundant && addConstraint)
            addConstraint = !isRedundant(newConstraint, constraints);

        if (!addConstraint || newConstraint.leaves.size() < 2) {
            constraintId--;
            numLeftRight[constraintId] = {0, 0};
            continue;
        };

        constraints.push_back(newConstraint);
        // Prepare for the next one
        numLeftRight.emplace_back(0, 0);
    }

    // Extract Merges
    for (const auto& edge : reversedEdgesTime) {
        addConstraint = false;
        Constraint newConstraint;

        // Shorthands
        const size_t idToRight(edge.first);

        // Skip if node is not a leaf (for leaf only setting)
        if (leafOnly && !tree->isLeaf(idToRight)) continue;

        newConstraint.level = tree->depthWithReverse(idToRight);

        std::set<size_t> leaves;
        const auto& time = tree->nodes[idToRight].startTime();
        const auto& prevTime = TemporalTree::getPrevTimeFromTimes(time, times);
        tree->getLeaves(idToRight, time, time, leaves);
        newConstraint.leaves.insert(leaves.begin(), leaves.end());
        newConstraint.startTime = time;
        newConstraint.endTime = time;
        newConstraint.fulfilled = false;
        numLeftRight[constraintId].second++;

        const std::vector<size_t>& idsFromLeft = edge.second;
        ivwAssert(!idsFromLeft.empty(), "Time map is empty.");
        if (idsFromLeft.empty()) continue;

        // All incoming nodes must be leaves too in leaf only setting
        if (leafOnly) {
            bool allLeaves = true;
            for (auto idFromLeft : idsFromLeft) {
                allLeaves = allLeaves && tree->isLeaf(idFromLeft);
                if (!allLeaves) break;
            }
            if (!allLeaves) continue;
        }

        if (idsFromLeft.size() > 1) {
            for (auto idFromLeft : idsFromLeft) {
                leaves.clear();
                tree->getLeaves(idFromLeft, prevTime, prevTime, leaves);
                newConstraint.leaves.insert(leaves.begin(), leaves.end());
                numLeftRight[constraintId].first++;
                addConstraint = true;
            }
        }

        constraintId++;

        if (removeRedundant && addConstraint)
            addConstraint = !isRedundant(newConstraint, constraints);

        if (!addConstraint || newConstraint.leaves.size() < 2) {
            constraintId--;
            numLeftRight[constraintId] = {0, 0};
            continue;
        };

        constraints.push_back(newConstraint);
        // Prepare for the next one
        numLeftRight.emplace_back(0, 0);
    }

    size_t numConstraints = constraintId;

    std::map<uint64_t, std::vector<size_t>> nonHierarchyPerTimeStep;
    for (constraintId = 0; constraintId < numConstraints; constraintId++) {
        auto& currentConstraint = constraints[numConstraintsAlreadyPresent + constraintId];
        size_t level = currentConstraint.level;
        if (numByLevel.size() < level + 1) {
            numByLevel.resize(level + 1);
        }
        numByLevel[level]++;

        if (numLeftRight[constraintId].first == 1 && numLeftRight[constraintId].second > 1) {
            currentConstraint.type = ConstraintType::Split;
        } else if (numLeftRight[constraintId].first > 1 && numLeftRight[constraintId].second == 1) {
            currentConstraint.type = ConstraintType::Merge;
        } else if (numLeftRight[constraintId].first == 1 &&
                   numLeftRight[constraintId].second == 1) {
            currentConstraint.type = ConstraintType::Correspondance;
        } else {
            currentConstraint.type = ConstraintType::MergeSplit;
        }
        if (currentConstraint.type != ConstraintType::Hierarchy) {
            nonHierarchyPerTimeStep[currentConstraint.startTime].push_back(
                numConstraintsAlreadyPresent + constraintId);
        }
    }

    // Extract rivaling merge-split constraints: These are constraints that could lead to
    // crossings Merge-Split Constraints only concern a single timestep, a part of the leaves
    // end at this point and another part begins In relation to one constraint A, constraint B
    // (also non-hierarchy, also at the same time step) is a rival if either the intersection of
    // leaves from A and B is empty or if A is a superset of B (i.e. all leaves of A are
    // contained in B), then connections within B may cross connections in A.
    for (auto it = nonHierarchyPerTimeStep.begin(); it != nonHierarchyPerTimeStep.end(); it++) {
        auto potentialRivals = it->second;

        for (size_t idFirst = 0; idFirst < potentialRivals.size() - 1; idFirst++) {
            size_t constraintIdFirst = potentialRivals[idFirst];
            const auto& leavesFirst = constraints[constraintIdFirst].leaves;
            for (size_t idSecond = idFirst + 1; idSecond < potentialRivals.size(); idSecond++) {
                size_t constraintIdSecond = potentialRivals[idSecond];
                const auto& leavesSecond = constraints[constraintIdSecond].leaves;
                std::set<uint64_t> intersect;
                std::set_intersection(leavesFirst.begin(), leavesFirst.end(), leavesSecond.begin(),
                                      leavesSecond.end(),
                                      std::inserter(intersect, intersect.begin()));
                if (intersect.empty()) {
                    // Rivals are symmetric if intersection is empty
                    rivalConstraints[constraintIdFirst].push_back(constraintIdSecond);
                    rivalConstraints[constraintIdSecond].push_back(constraintIdFirst);
                } else {
                    bool isFirstSubsetOfSecond = (leavesFirst.size() - intersect.size()) == 0;
                    bool isSecondSubsetOfFirst = (leavesSecond.size() - intersect.size()) == 0;
                    if (isFirstSubsetOfSecond && !isSecondSubsetOfFirst) {
                        rivalConstraints[constraintIdFirst].push_back(constraintIdSecond);
                    } else if (isSecondSubsetOfFirst && !isFirstSubsetOfSecond) {
                        rivalConstraints[constraintIdSecond].push_back(constraintIdFirst);
                    }
                    // ToDo: Do we need to check for other things than that?
                }
            }
        }
    }
}

void extractHierarchyConstraints(std::shared_ptr<const TemporalTree>& tree,
                                 HierarchyAggregation aggregateType,
                                 std::vector<Constraint>& constraints,
                                 std::vector<size_t>& numByLevel) {

    // Exclude 0 as this is always the one global root
    std::vector<size_t> indices(tree->nodes.size() - 1);
    std::iota(std::begin(indices), std::end(indices), 1);

    // For aggregation we need to iterate over nodes sorted by time, so that we will have extracted
    // constraints for a timestep t before t+1 (and may be able to extend constraints based on that)
    if (aggregateType != HierarchyAggregation::None) tree->sortByTime(indices);

    std::set<uint64_t> times = tree->getTimes();

    /// Keeps track of leaves that have been replaced
    std::vector<std::set<size_t>> replacedLeaves;

    for (size_t nodeIndex : indices) {
        // Leafs by themselves do not impose a constraints
        if (tree->isLeaf(nodeIndex)) {
            continue;
        }
        // Get nodes leaves and times
        uint64_t nodeStartTime = tree->nodes[nodeIndex].startTime();
        uint64_t nodeEndTime = tree->nodes[nodeIndex].endTime();
        bool hasSucessors = tree->hasTemporalSuccessors(nodeIndex);
        // If the node has sucessors, it has been extended for the purpose of continuity between
        // splitees and mergees, but this node does not actually exist anymore and nodeEndTime
        // Use previous time step for the constraint
        if (nodeStartTime != nodeEndTime && hasSucessors) {
            nodeEndTime = TemporalTree::getPrevTimeFromTimes(nodeEndTime, times);
        }

        std::set<size_t> leaves;
        tree->getLeaves(nodeIndex, nodeStartTime, nodeEndTime, leaves);
        // No constraints for just a single leaf
        if (leaves.size() < 2) {
            continue;
        }
        // No constraints if constraint would be to have all leaves
        std::set<size_t> leavesRoot;
        tree->getLeaves(0, nodeStartTime, nodeEndTime, leavesRoot);
        if (leaves.size() == leavesRoot.size()) {
            continue;
        }

        bool continuingConstraint = false;

        if (aggregateType != HierarchyAggregation::None) {
            size_t constraintId = 0;

            // Go through all other already extracted constraints
            for (constraintId; constraintId < constraints.size(); constraintId++) {
                const auto& oldConstraint = constraints[constraintId];
                const auto& nextTimeStepConstraint =
                    TemporalTree::getNextTimeFromTimes(oldConstraint.endTime, times);
                // If the constraintNode (the latest node that induced the hierarchy constraint)
                // has no sucessors it either ends to exist at its endTime t or we do not have
                // correspondance information about it. Its constraint can be extended if the
                // new node has starting time one after t If the constraintNode has successor,
                // then the current node should start at the end of the constraintNode
                if (nextTimeStepConstraint == nodeStartTime) {
                    if (aggregateType == HierarchyAggregation::FullMatch) {
                        // Leaves are an exact match
                        if (leaves.size() == oldConstraint.leaves.size() &&
                            std::equal(leaves.cbegin(), leaves.cend(),
                                       oldConstraint.leaves.cbegin())) {
                            continuingConstraint = true;
                            break;
                        }
                    } else {
                        std::set<size_t> leavesWithMergeSplitReplaced;
                        std::set<size_t> replacedLeavesCurrent;
                        auto& replacedLeavesPreviously = replacedLeaves[constraintId];
                        for (auto leaf : oldConstraint.leaves) {
                            // Leaf was already replaced before, not relevant here
                            if (replacedLeavesPreviously.find(leaf) !=
                                replacedLeavesPreviously.end())
                                continue;
                            // Leaf can only be replaced if it has sucessors and ends in the next
                            // time step
                            if (tree->nodes[leaf].endTime() == nextTimeStepConstraint) {
                                auto successors = tree->getTemporalSuccessors(leaf);
                                if (!successors.empty()) {
                                    leavesWithMergeSplitReplaced.insert(successors.begin(),
                                                                        successors.end());
                                    replacedLeavesCurrent.insert(leaf);
                                } else {
                                    leavesWithMergeSplitReplaced.insert(leaf);
                                }
                            } else {
                                leavesWithMergeSplitReplaced.insert(leaf);
                            }
                        }

                        if (aggregateType == HierarchyAggregation::MergeSplitMatch) {
                            // With the replaced leaves, we check for an exact match
                            if (leaves.size() == leavesWithMergeSplitReplaced.size() &&
                                std::equal(leaves.cbegin(), leaves.cend(),
                                           leavesWithMergeSplitReplaced.cbegin())) {
                                continuingConstraint = true;
                                replacedLeavesPreviously.insert(replacedLeavesCurrent.begin(),
                                                                replacedLeavesCurrent.end());
                                break;
                            }
                        } else {

                            // In addition to leaves splitting and merging, we allow for nodes to
                            // end before and start later than the current time
                            // aggregateType == HierarchyAggregation::ExcludeAllEndsBegins
                            // Check everything that it not in the intersection between
                            // leaves and leavesWithMergeSplitReplaced
                            std::set<uint64_t> intersect;
                            std::set_intersection(leaves.begin(), leaves.end(),
                                                  leavesWithMergeSplitReplaced.begin(),
                                                  leavesWithMergeSplitReplaced.end(),
                                                  std::inserter(intersect, intersect.begin()));
                            // Exact match, early termination
                            if (leaves.size() == leavesWithMergeSplitReplaced.size() &&
                                intersect.size() == leaves.size()) {
                                continuingConstraint = true;
                                replacedLeavesPreviously.insert(replacedLeavesCurrent.begin(),
                                                                replacedLeavesCurrent.end());
                                break;
                            }
                            // Nothing at all in common, early termination, check for next
                            // constraint
                            if (intersect.empty()) {
                                continue;
                            }

                            // Check if leaves are the same, apart from
                            // the ones in the old constraint ending in the relevant time
                            // and the ones starting at nodeStartTime
                            auto itLeavesOld = leavesWithMergeSplitReplaced.cbegin();
                            while (itLeavesOld != leavesWithMergeSplitReplaced.cend()) {
                                // An old leaf needs to be in the intersection or end at the
                                // relevant time
                                auto& node = tree->nodes[*itLeavesOld];
                                // Node ist not in the intersection, but exists beyond current
                                // relevant end time or until current relevant end time, but has
                                // other successors
                                bool isInIntersection =
                                    intersect.find(*itLeavesOld) != intersect.end();
                                if (!isInIntersection && node.endTime() > nextTimeStepConstraint ||
                                    node.endTime() == nextTimeStepConstraint &&
                                        tree->hasTemporalSuccessors(*itLeavesOld))
                                    break;
                                // Leaf will not be relevant in future extension checks
                                if (!isInIntersection) replacedLeavesCurrent.insert(*itLeavesOld);
                                itLeavesOld++;
                            }
                            auto itLeavesNew = leaves.cbegin();
                            while (itLeavesNew != leaves.cend()) {
                                // A new leaf needs to be in the intersection or start at the
                                // relevant time to allow for constraint aggregation
                                auto& node = tree->nodes[*itLeavesNew];
                                // Node exists already before current node but is not in
                                // the intersection -> no continuation
                                if (node.startTime() < nodeStartTime &&
                                    intersect.find(*itLeavesNew) == intersect.end())
                                    break;
                                itLeavesNew++;
                            }

                            if (itLeavesOld == leavesWithMergeSplitReplaced.cend() &&
                                itLeavesNew == leaves.cend()) {
                                // Todo: Should we try to find the best match?
                                continuingConstraint = true;
                                replacedLeavesPreviously.insert(replacedLeavesCurrent.begin(),
                                                                replacedLeavesCurrent.end());
                                break;
                            }
                        }
                    }
                }
            }

            if (continuingConstraint) {
                auto& constraint = constraints[constraintId];
                // Extend constraint
                constraint.endTime = nodeEndTime;
                constraint.leaves.insert(leaves.begin(), leaves.end());
            }
        }

        if (!continuingConstraint) {
            Constraint constraint;
            constraint.leaves = leaves;
            constraint.type = ConstraintType::Hierarchy;
            constraint.startTime = nodeStartTime;
            constraint.endTime = nodeEndTime;
            constraint.level = tree->depthWithReverse(nodeIndex);

            if (numByLevel.size() < constraint.level + 1) {
                numByLevel.resize(constraint.level + 1);
            }
            numByLevel[constraint.level]++;

            constraints.push_back(constraint);
            // Initialize replacedLeaves data structure
            if (aggregateType != HierarchyAggregation::None) {
                replacedLeaves.push_back(std::set<size_t>());
            }
        }
    }
}

bool isRedundant(const Constraint& constraint, const std::vector<Constraint>& constraints) {

    const auto& leaves = constraint.leaves;

    for (const auto& constraintToCheck : constraints) {
        if (TemporalTree::TNode::isOverlappingTemporally(constraint.startTime, constraint.endTime,
                                                         constraintToCheck.startTime,
                                                         constraintToCheck.endTime)) {
            const auto& leavesToCheck = constraintToCheck.leaves;
            if (leaves.size() == leavesToCheck.size() &&
                std::equal(leaves.begin(), leaves.end(), leavesToCheck.begin())) {
                return true;
            }
        }
    }

    return false;
}

bool isOverlappingWithConstraint(const TemporalTree::TNode& leaf, const Constraint& constraint,
                                 bool hasPredecessor, bool hasSucessor) {
    const uint64_t overlapStart = std::max(constraint.startTime, leaf.startTime());
    const uint64_t overlapEnd = std::min(constraint.endTime, leaf.endTime());
    const uint64_t leafStart = leaf.startTime();
    const uint64_t leafEnd = leaf.endTime();
    // Previous version:
    // return overlapStart < overlapEnd ||
    // Unless we have a single timestep hierarchy or merge Constraints
    //  (overlapStart == overlapEnd &&
    //  (constraint.startTime == constraint.endTime || leafStart == leafEnd));
    // This actually excludes cases that are not posing a problem, and does not recognize others
    // For hierarchy constraints:
    //      |-----|  Constraint (coming from node with no sucessors)
    //            |-----| Leaf starting at constraint end -> returned false, but should be true
    // |----|             Leaf ending at constraint start -> returned false, but should be true
    //      |-----|-----| Constraint (node with sucessors) needs to yiel a different result
    //            |-----| Should be false, overlaps with succesor, but not the constraint node
    // Last case should be handeled outside function, by ending constraint one step earlier if a
    // node has one or more successors
    //      |       Constraint (coming from single step hierarchy constraint)
    // returned true in any case where there is overlap, but should not in all, e.g.
    // |----|----| when the overlap is in the endpoint of the leaf and the leaf has a successor
    // For merge constraints:
    // Any overlap returned true, but again that should not be the case, e.g. when overlap is
    // only in the endpoint of the leaf and the leaf has a sucessors (this can still be a problem
    // but does not have to be):

    // Similarly with leafs starting at the constraint time (still can be a problem if connection to
    // predecessor causes a crossing), e.g.
    //     (1)|\
	//     (4)  |  Merge Constraint (just one time step)
    //     (2)|/
    //     (3)|||----|  Leaf starting at the time step, would only cause a violation if it has a
    //      predecessor that come before (or in between) the constraint leaves

    // New version:
    // For hierarchy constraints, we exclude cases where the overlap is exclusively in the end
    // point of the leaf and it has a sucessor (means the leaf has converted in a different one
    // already)
    if (constraint.type == ConstraintType::Hierarchy) {
        return overlapStart < overlapEnd ||
               (overlapStart == overlapEnd && !(leafEnd == overlapEnd && hasSucessor));
    }
    // For non-hierarchy constraints, the overlap will always be end==start, if there is overlap
    // This means in case of overlap:
    // overlapStart = overlapEnd = constraint.startTime = constraint.endTime
    //       |       Constraint
    // |---|         (oStart > oEnd) Leaf begins and ends before -> return false
    //   |---|       Leaf begins before and ends at the constraint, problem if leaf is ending
    //     |---|     Leaf begins before and ends after -> return true
    //       |       Leaf begins and ends at constraint time -> return false
    //       |---|   Leaf begins at constraints -> false, pred connection checked elsewhere
    //        |---|  (oEnd < oStart) Leaf begins after constraint -> return false
    return overlapStart == overlapEnd && !(leafEnd == overlapEnd && hasSucessor) &&
           leafStart != overlapStart;
}

void getOrderEdgesFromConstraint(const Constraint& constraint,
                                 std::shared_ptr<const TemporalTree>& tree,
                                 const TemporalTree::TTreeOrder& order,
                                 const TemporalTree::TTreeOrderMap& orderMap,
                                 const std::set<size_t>& excludeLeaves,
                                 std::vector<std::pair<size_t, size_t>>& edges) {
    std::vector<size_t> left;
    std::vector<size_t> right;
    for (auto leaf : constraint.leaves) {
        // Skip leaves that are given in excludeLeaves
        if (excludeLeaves.find(leaf) != excludeLeaves.end()) continue;
        const auto& leafNode = tree->nodes[leaf];
        // Leaves that end here, are starting points of edges
        if (leafNode.endTime() <= constraint.endTime) {
            left.push_back(leaf);
        }
        // Leaves that begin here, are end points of edges
        if (leafNode.startTime() == constraint.startTime) {
            right.push_back(leaf);
        }
    }

    for (auto& leftLeaf : left) {
        const auto& sucessors = tree->getTemporalSuccessors(leftLeaf);
        for (auto& rightLeaf : right) {
            // Skip single time step edges
            if (leftLeaf != rightLeaf) {
                // Check if this is an actual edge in the tree
                // if (std::find(sucessors.begin(), sucessors.end(), rightLeaf) != sucessors.end())
                    edges.emplace_back(orderMap.at(leftLeaf), orderMap.at(rightLeaf));
            }
        }
    }
}

bool areOrderEdgesCrossing(const std::vector<std::pair<size_t, size_t>>& edgesConstraintA,
                           const std::vector<std::pair<size_t, size_t>>& edgesconstraintB) {
    for (const auto& edgeA : edgesConstraintA) {
        for (const auto& edgeB : edgesconstraintB) {
            // Starting points and end points need have the same order
            // A1 < B1, but A2 > B2 (term below evaluates to true != false)
            // A1 > B1, but A2 < B2 (term below evaluates to false != true)
            bool orderFirst = edgeA.first <= edgeB.first;
            bool orderSecond = edgeA.second <= edgeB.second;
            // both smaller or both larger is fine
            if (orderFirst != orderSecond) {
                return true;
            }
        }
    }
    // No crossing detected
    return false;
}

}  // namespace constraint

}  // namespace kth
}  // namespace inviwo
