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

#include <inviwo/mergetreemaps/processors/segmentationfromcontourtree.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo SegmentationFromContourTree::processorInfo_{
    "org.inviwo.SegmentationFromContourTree",  // Class identifier
    "Segmentation From Contour Tree",          // Display name
    "Topology",                                // Category
    CodeState::Experimental,                   // Code state
    Tags::None,                                // Tags
};
const ProcessorInfo SegmentationFromContourTree::getProcessorInfo() const { return processorInfo_; }

SegmentationFromContourTree::SegmentationFromContourTree()
    : PoolProcessor()
    , treeType_("treeType", "Tree Type", "", InvalidationLevel::Valid)
    , leafOnly_("leafOnly", "Leaf Only", false) {

    addPort(inport_);
    addPort(treeInport_);
    addPort(outport_);

    addProperty(treeType_);
    treeType_.setReadOnly(true);
    treeInport_.onChange([this]() {
        treeType_.set([this]() -> std::string {
            if (treeInport_.hasData()) {
                return toString(treeInport_.getData()->type);
            } else {
                return "";
            }
        }());
    });

    addProperty(leafOnly_);
}

void SegmentationFromContourTree::process() {

    const auto inportScalars = inport_.getData();
    const auto contourTree = treeInport_.getData();

    using Result = std::shared_ptr<topology::TriangulationData>;

    auto compute = [inportScalars, contourTree, leafOnly = leafOnly_.get()](
                       pool::Stop stop, pool::Progress progress) -> Result {
        return inportScalars->getScalarValues()
            ->getRepresentation<BufferRAM>()
            ->dispatch<std::shared_ptr<topology::TriangulationData>, dispatching::filter::Scalars>(
                [inportScalars, contourTree, leafOnly, stop,
                 progress](const auto buffer) -> Result {
                    if (stop) return nullptr;
                    using ValueType = util::PrecisionValueType<decltype(buffer)>;

                    progress(0.1f);

                    // create a copy of the data values, which we will use to fill the segmentation
                    // auto segmentationValues = //buffer->getDataContainer();
                    std::vector<float> segmentationValues(buffer->getDataContainer().size());
                    std::vector<int> offsets(inportScalars->getOffsets());

                    // In the tree: vertexId = node->getVertexId();
                    auto tree = contourTree->getTree();
                    ttk::SimplexId vertexId;
                    float min = std::numeric_limits<float>::max();
                    float max = std::numeric_limits<float>::lowest();

                    // Iterate over all normal nodes in the tree associated with super arcs
                    size_t numSuperArcs = tree->getNumberOfSuperArcs();
                    float maxId = static_cast<float>(numSuperArcs) + 1.0f;
                    for (ttk::ftm::idSuperArc superArcId = 0; superArcId < numSuperArcs;
                         superArcId++) {

                        auto superArc = tree->getSuperArc(superArcId);
                        auto downNodeId = superArc->getDownNodeId();
                        auto node = tree->getNode(downNodeId);
                        auto numDown = node->getNumberOfDownSuperArcs();
                        // Skip superarcs that are not a leaf
                        bool markLeaf = false;
                        if (leafOnly && numDown != 0) markLeaf = true;
                        size_t arcSize = superArc->regionSize();
                        for (size_t i = 0; i < arcSize; i++) {
                            vertexId = superArc->getRegularNodeId(i);
                            segmentationValues[vertexId] =
                                markLeaf ? maxId : static_cast<float>(superArcId);
                            if (superArcId < min) min = static_cast<float>(superArcId);
                            if (superArcId > max) max = static_cast<float>(superArcId);
                        }
                    }
                    progress(0.8f);

                    // Only critical points are missing now

                    size_t numNodes = tree->getNumberOfNodes();
                    ttk::ftm::idSuperArc superArcId;

                    for (ttk::ftm::idNode nodeId = 0; nodeId < numNodes; ++nodeId) {
                        auto node = tree->getNode(nodeId);
                        auto numDown = node->getNumberOfDownSuperArcs();
                        // Skip superarcs that are not a leaf
                        bool markLeaf = false;
                        if (leafOnly && numDown != 0) markLeaf = true;
                        if (node->getNumberOfUpSuperArcs() >= 1) {
                            // Assign leaf and saddles to their up superArcId (there will only be
                            // one)
                            superArcId = node->getUpSuperArcId(0);
                        } else {
                            // This is the case *only* for the root node, assign it to the first one
                            superArcId = node->getDownSuperArcId(0);
                        }
                        vertexId = node->getVertexId();
                        segmentationValues[vertexId] =
                            markLeaf ? maxId : static_cast<float>(superArcId);
                        if (superArcId < min) min = static_cast<float>(superArcId);
                        if (superArcId > max) max = static_cast<float>(superArcId);
                    }

                    // create a new triangulation based on the old one, but with segmentation as
                    // scalar values
                    auto result = std::make_shared<topology::TriangulationData>(*inportScalars);
                    result->setScalarValues(util::makeBuffer(std::move(segmentationValues)));
                    result->setOffsets(std::move(offsets));
                    if (result->isUniformGrid()) {
                        auto datamapper = result->getDataMapper();
                        datamapper.dataRange = vec2(min, max);
                        datamapper.valueRange = vec2(min, max);
                        result->set(inportScalars->getGridDimensions(),
                                    inportScalars->getGridOrigin(), inportScalars->getGridExtent(),
                                    datamapper);
                    }

                    return result;
                });
    };

    outport_.clear();
    dispatchOne(compute, [this](Result result) {
        outport_.setData(result);
        newResults();
    });
}

}  // namespace inviwo
