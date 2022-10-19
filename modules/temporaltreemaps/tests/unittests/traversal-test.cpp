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

#include <warn/push>
#include <warn/ignore/all>
#include <gtest/gtest.h>
#include <warn/pop>

#include <modules/temporaltreemaps/datastructures/tree.h>

namespace inviwo {

TEST(TemporalTreeTraversal, TemporalChildren) {

    kth::TemporalTree tree;

    // Root node existing for the entire time span
    kth::TemporalTree::TNode root{"root", {{0, 10.0f}, {5, 10.f}}};
    tree.addNode(root);

    // Merge in the first layer
    kth::TemporalTree::TNode childA{"childA", {{0, 10.0f}, {3, 10.f}}};
    tree.addNode(childA);
    tree.addHierarchyEdge(0, 1);

    kth::TemporalTree::TNode childB{"childB", {{0, 10.0f}, {3, 10.f}}};
    tree.addNode(childB);
    tree.addHierarchyEdge(0, 2);

    kth::TemporalTree::TNode childAB{"childAB", {{3, 10.0f}, {5, 10.f}}};
    tree.addNode(childAB);
    tree.addHierarchyEdge(0, 3);
    tree.addTemporalEdge(1, 3);
    tree.addTemporalEdge(2, 3);

    // Split in the last layer
    kth::TemporalTree::TNode childCD{"childCD", {{0, 10.0f}, {3, 10.f}}};
    tree.addNode(childCD);
    tree.addHierarchyEdge(1, 4);

    kth::TemporalTree::TNode childC{"childC", {{3, 10.0f}, {5, 10.f}}};
    tree.addNode(childC);
    tree.addHierarchyEdge(3, 5);
    kth::TemporalTree::TNode childD{"childD", {{3, 10.0f}, {5, 10.f}}};
    tree.addNode(childC);
    tree.addHierarchyEdge(3, 6);
    tree.addTemporalEdge(4, 5);
    tree.addTemporalEdge(4, 6);

    kth::TemporalTree::TNode childE{"childE", {{0, 10.0f}, {5, 10.f}}};
    tree.addNode(childE);
    tree.addHierarchyEdge(2, 7);
    tree.addHierarchyEdge(3, 7);
    kth::TemporalTree::TNode childF{"childF", {{3, 10.0f}, {5, 10.f}}};
    tree.addNode(childF);
    tree.addHierarchyEdge(3, 8);

    EXPECT_EQ(tree.getLeaves().size(), 5);
    std::set<size_t> leaves;
    tree.getLeaves(0, 3, 5, leaves);
    EXPECT_EQ(leaves.size(), 4);
}

}  // namespace inviwo
