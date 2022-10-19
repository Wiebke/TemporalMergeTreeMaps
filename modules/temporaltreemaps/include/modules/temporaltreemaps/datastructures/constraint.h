/*********************************************************************
 *  Author  : Tino Weinkauf and Wiebke Koepp
 *  Init    : Tuesday, March 27, 2018 - 15:05:01
 *
 *  Project : KTH Inviwo Modules
 *
 *  License : Follows the Inviwo BSD license model
 *********************************************************************
 */

#pragma once

#include <modules/temporaltreemaps/temporaltreemapsmoduledefine.h>
#include <inviwo/core/common/inviwo.h>
#include <modules/temporaltreemaps/datastructures/tree.h>
#include <modules/temporaltreemaps/datastructures/treeorder.h>

namespace inviwo {
namespace kth {

namespace constraint {

enum class ConstraintType { Merge, Split, MergeSplit, Correspondance, Hierarchy, None };

// Ordered from least aggregation to most aggregation
enum class HierarchyAggregation { None, FullMatch, MergeSplitMatch, ExcludeAllEndsBegins };

struct Constraint {
    std::set<size_t> leaves;
    uint64_t startTime = 0;
    uint64_t endTime = std::numeric_limits<uint64_t>::max();
    size_t level = 0;
    ConstraintType type = ConstraintType::None;
    bool fulfilled = false;

    friend std::ostream& operator<<(std::ostream& os, const Constraint& c) {
        std::ostringstream stream;
        stream << "{";
        for (auto it = c.leaves.begin(); it != c.leaves.end(); it++) {
            stream << *it;
            if (std::next(it) != c.leaves.end()) stream << ", ";
        }
        stream << "}";
        std::string constraintType;
        switch (c.type) {
            case ConstraintType::Hierarchy:
                constraintType = "H";
                break;
            case ConstraintType::Merge:
                constraintType = "M";
                break;
            case ConstraintType::Split:
                constraintType = "S";
                break;
            case ConstraintType::MergeSplit:
                constraintType = "B";
                break;
            case ConstraintType::Correspondance:
                constraintType = "C";
                break;
            default:
                constraintType = "N";
                break;
        };
        return (os << (c.fulfilled ? "+ " : "- ") << constraintType << " " << stream.str() << "["
                   << c.startTime << "," << c.endTime << "]" << std::endl);
    }
};

struct ConstraintsStatistic {
    std::set<size_t> unhappyLeaves;
    std::vector<size_t> fulfilledByLevelMergeSplit;
    std::vector<size_t> fulfilledByLevelHierarchy;

    void clear();

    void update(const Constraint& constraint);

    size_t numFulfilledHierarchyConstraints() const;

    size_t numFulFilledMergeSplitConstraints() const;
};

/// Sums over the given vector
size_t numConstraints(const std::vector<size_t>& numByLevel);

/// CHecks if the given constraints if fulfilled
bool isFulFilled(Constraint& constraint, std::shared_ptr<const TemporalTree>& tree,
                 const TemporalTree::TTreeOrder& order, const TemporalTree::TTreeOrderMap& orderMap,
                 const std::vector<Constraint>& constraints,
                 const std::vector<size_t>& rivalConstraints);

/// Get the number of fulfilled constraints, where constraints are given as a set of leaves and a
/// time interval at which the constraint has to be fulfilled
size_t numFulfilledConstraints(std::shared_ptr<const TemporalTree>& tree,
                               const TemporalTree::TTreeOrder& order,
                               const TemporalTree::TTreeOrderMap& orderMap,
                               std::vector<Constraint>& constraints,
                               const std::map<size_t, std::vector<size_t>>& rivalConstraints,
                               ConstraintsStatistic& statistic);

void extractMergeSplitConstraints(std::shared_ptr<const TemporalTree>& tree, bool leafOnly,
                                  bool removeRedundant, std::vector<Constraint>& constraints,
                                  std::map<size_t, std::vector<size_t>>& rivalConstraints,
                                  std::vector<size_t>& numByLevel);

void extractHierarchyConstraints(std::shared_ptr<const TemporalTree>& tree,
                                 HierarchyAggregation aggregateType,
                                 std::vector<Constraint>& constraints,
                                 std::vector<size_t>& numByLevel);

bool isRedundant(const Constraint& constraint, const std::vector<Constraint>& constraints);

/// Determines is a leaf (leafStart, leafEnd) overlaps with a constraint (constraintStart,
/// constraintEnd), excluding overlap that is only in the endpoints of the leaf (unless the leaf
/// only has a single time step)
bool isOverlappingWithConstraint(const TemporalTree::TNode& leaf, const Constraint& constraint,
                                 bool hasSucessor, bool hasPredecessor);

/// For a non-hierarchy constraint, creates an edge-set connecting all leaves that end at the time
/// for the constraint to all leaves that begin. The position of each node in this edge-set is
/// determined through the order, leaves can be excluded (to avoid duplicates when checked against a
/// constraint superset)
void getOrderEdgesFromConstraint(const Constraint& constraint,
                                 std::shared_ptr<const TemporalTree>& tree,
                                 const TemporalTree::TTreeOrder& order,
                                 const TemporalTree::TTreeOrderMap& orderMap,
                                 const std::set<size_t>& excludeLeaves,
                                 std::vector<std::pair<size_t, size_t>>& edges);

/// Checks if any of the edges in A intersect with edges in B
bool areOrderEdgesCrossing(const std::vector<std::pair<size_t, size_t>>& edgesConstraintA,
                           const std::vector<std::pair<size_t, size_t>>& edgesconstraintB);

}  // namespace constraint

}  // namespace kth
}  // namespace inviwo
