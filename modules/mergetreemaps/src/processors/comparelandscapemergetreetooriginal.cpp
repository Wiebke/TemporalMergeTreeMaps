/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2022 Inviwo Foundation
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

#include <inviwo/mergetreemaps/processors/comparelandscapemergetreetooriginal.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo CompareLandscapeMergeTreeToOriginal::processorInfo_{
    "org.inviwo.CompareLandscapeMergeTreeToOriginal",  // Class identifier
    "Compare Landscape Merge Tree To Original",        // Display name
    "Undefined",                                       // Category
    CodeState::Experimental,                           // Code state
    Tags::None,                                        // Tags
};
const ProcessorInfo CompareLandscapeMergeTreeToOriginal::getProcessorInfo() const {
    return processorInfo_;
}

CompareLandscapeMergeTreeToOriginal::CompareLandscapeMergeTreeToOriginal()
    : Processor()
    , sumCorrectCritical_("sumCorrectCritical", "Sum of Correct Critical", 0, 0,
                          std::numeric_limits<size_t>::max(), 1, InvalidationLevel::Valid,
                          PropertySemantics::Text)
    , sumIncorrectCritical_("sumIncorrectCritical", "Sum of Incorrect Critical (should be Reg)", 0,
                            0, std::numeric_limits<size_t>::max(), 1, InvalidationLevel::Valid,
                            PropertySemantics::Text)
    , sumCorrectRegular_("sumCorrectRegular", "Sum of Correct Regular", 0, 0,
                         std::numeric_limits<size_t>::max(), 1, InvalidationLevel::Valid,
                         PropertySemantics::Text)
    , sumIncorrectRegular_("sumIncorrectRegular", "Sum of Incorrect Regular (should be Crit)", 0, 0,
                           std::numeric_limits<size_t>::max(), 1, InvalidationLevel::Valid,
                           PropertySemantics::Text)
    , printIncorrectCritical_("printIncorrectCritical", "Print Incorrect Critical", false)
    , printIncorrectRegular_("printIncorrectRegular", "Print Incorrect Regular", false)
    , timer_("timer", "Eval Time (s)", 0.f, 0.f, std::numeric_limits<float>::max(), 0.001f,
             InvalidationLevel::Valid, PropertySemantics::Text) {

    addPort(inport_);
    addPort(treesInport_);
    addPort(outport_);

    addProperties(sumCorrectCritical_, sumIncorrectCritical_, sumCorrectRegular_,
                  sumIncorrectRegular_, printIncorrectCritical_, printIncorrectRegular_, timer_);
    timer_.setReadOnly(true);
}

void CompareLandscapeMergeTreeToOriginal::process() {
    performanceTimer_.Reset();

    const auto treesData = treesInport_.getData();
    if (!treesData) return;
    auto numTrees = treesData.get()->size();
    const auto landscapeData = inport_.getData();

    bool leavesAreMin = true;

    std::vector<int> sumCorrectCritical(numTrees, 0);
    std::vector<int> sumIncorrectCritical(numTrees, 0);
    std::vector<int> sumCorrectRegular(numTrees, 0);
    std::vector<int> sumIncorrectRegular(numTrees, 0);
    std::vector<int> multiSaddles(numTrees, 0);
    std::vector<int> shortSuperArcs(numTrees, 0);

    for (size_t i = 0; i < numTrees; i++) {
        const topology::ContourTreeData &treeData = *treesData.get()->at(i).get();
        if (treeData.type == topology::TreeType::Join) {
            leavesAreMin = true;
        } else if (treeData.type == topology::TreeType::Split) {
            leavesAreMin = false;
        } else {
            LogProcessorError("Tree type not supported");
            return;
        }
        auto tree = treeData.getTree();
        auto numVertices = tree->getNumberOfVertices();

        const auto landscape = landscapeData->at(i);
        auto vertexIds =
            std::dynamic_pointer_cast<const TemplateColumn<int>>(landscape->getColumn("Vertex ID"));
        if (!vertexIds) return;
        auto mapPosition = std::dynamic_pointer_cast<const TemplateColumn<int>>(
            landscapeData->at(i)->getColumn("Map Position"));
        if (!mapPosition) return;

        auto scalars = landscapeData->at(i)->getColumn("Scalar Value");
        if (!mapPosition) return;

        // Build lookup maps
        std::map<int, int> vertex2pos;
        std::map<int, double> vertex2scalar;
        std::map<int, int> pos2vertex;
        for (size_t r = 0; r < numVertices; r++) {
            int vertexId = vertexIds->get(r);
            int pos = mapPosition->get(r);
            vertex2pos[vertexId] = pos;
            pos2vertex[pos] = vertexId;
            vertex2scalar[vertexId] = scalars->getAsDouble(r);
        }

        auto isCritical = [&vertex2pos, &pos2vertex, &vertex2scalar, numVertices,
                           leavesAreMin](int vertexId) -> bool {
            auto pos = vertex2pos[vertexId];
            if (pos == 0) return true;
            if (pos == numVertices - 1) return false;
            auto scalar = vertex2scalar[vertexId];
            auto vertexIdLeft = pos2vertex[pos - 1];
            auto scalarLeft = vertex2scalar[vertexIdLeft];
            auto vertexIdRight = pos2vertex[pos + 1];
            auto scalarRight = vertex2scalar[vertexIdRight];
            // Value is critical if both neighbors are larger or both neighbors are
            // smaller
            if (scalar > scalarLeft && scalar > scalarRight ||
                scalar < scalarLeft && scalar < scalarRight)
                return true;
            /*if (leavesAreMin) {
                // when leaves are minima, also regards as critical when only one neighbor is
                // strictly larger, but simulation of simplicity would have processed this vertex
                // first to break the tie
                if (scalar >= scalarLeft && scalar > scalarRight && vertexId < vertexIdLeft ||
                    scalar > scalarLeft && scalar >= scalarRight && vertexId < vertexIdRight)
                    return true;
                // when leaves are minima, also regards as critical when only one neighbor is
                // stricly smaller, but simulation of simplicity would have processed the other
                // vertex first to break the tie
                if (scalar <= scalarLeft && scalar < scalarRight && vertexId > vertexIdLeft ||
                    scalar < scalarLeft && scalar <= scalarRight && vertexId > vertexIdRight)
                    return true;
            } else {
                // when leaves are maxima, also regards as critical when only one neighbor is
                // strictly smaller, but simulation of simplicity would have processed this vertex
                // first to break the tie
                if (scalar <= scalarLeft && scalar < scalarRight && vertexId < vertexIdLeft ||
                    scalar < scalarLeft && scalar <= scalarRight && vertexId < vertexIdRight)
                    return true;
                // when leaves are minima, also regards as critical when only one neighbor is
                // stricly larger, but simulation of simplicity would have processed the other
                // vertex first to break the tie
                if (scalar >= scalarLeft && scalar > scalarRight && vertexId > vertexIdLeft ||
                    scalar > scalarLeft && scalar >= scalarRight && vertexId > vertexIdRight)
                    return true;
            }*/
            return false;
        };

        // Iterate over the tree
        size_t numSuperArcs = tree->getNumberOfSuperArcs();
        for (ttk::ftm::idSuperArc superArcId = 0; superArcId < numSuperArcs; superArcId++) {
            auto superArc = tree->getSuperArc(superArcId);
            size_t arcSize = superArc->regionSize();
            if (arcSize <= 2) shortSuperArcs[i]++;
            for (size_t nodeId = 0; nodeId < arcSize; nodeId++) {
                auto vertexId = superArc->getRegularNodeId(nodeId);
                bool isVertexCritical = isCritical(vertexId);
                if (isVertexCritical) {
                    sumIncorrectCritical[i]++;
                    if (printIncorrectCritical_.get()) {
                        LogProcessorWarn("--Vertex ID " << vertexId << " with value "
                                                        << vertex2scalar[vertexId]
                                                        << " at timestep " << i
                                                        << " should be regular, but is critical.");
                        auto pos = vertex2pos[vertexId];
                        if (pos != 0 && pos != numVertices - 1) {
                            auto vertexIdLeft = pos2vertex[pos - 1];
                            bool isVertexLeftCritical = isCritical(vertexIdLeft);
                            auto scalarLeft = vertex2scalar[vertexIdLeft];
                            LogProcessorWarn("The left neighbor is"
                                             << (isVertexLeftCritical ? "critical" : "regular")
                                             << " with value " << scalarLeft << " and vertexID "
                                             << vertexIdLeft << " .");
                            auto vertexIdRight = pos2vertex[pos + 1];
                            bool isVertexRightCritical = isCritical(vertexIdRight);
                            auto scalarRight = vertex2scalar[vertexIdRight];
                            LogProcessorWarn("The right neighbor is "
                                             << (isVertexRightCritical ? "critical" : "regular")
                                             << " with value " << scalarRight << " and vertexID "
                                             << vertexIdRight << " .");
                        }
                    }
                } else {
                    sumCorrectRegular[i]++;
                }
            }
        }
        // Only critical points are missing now

        size_t numNodes = tree->getNumberOfNodes();
        for (ttk::ftm::idNode nodeId = 0; nodeId < numNodes; ++nodeId) {
            auto node = tree->getNode(nodeId);
            auto numDown = node->getNumberOfDownSuperArcs();
            if (numDown > 2) multiSaddles[i]++;
            auto vertexId = node->getVertexId();
            bool isVertexCritical = isCritical(vertexId);
            if (isVertexCritical) {
                sumCorrectCritical[i]++;
            } else {
                sumIncorrectRegular[i]++;
                if (printIncorrectRegular_.get()) {
                    LogProcessorWarn("--Vertex ID " << vertexId << " with value "
                                                    << vertex2scalar[vertexId] << " at timestep "
                                                    << i << " should be critical, but is regular.");
                    auto pos = vertex2pos[vertexId];
                    if (pos != 0 && pos != numVertices - 1) {
                        auto vertexIdLeft = pos2vertex[pos - 1];
                        bool isVertexLeftCritical = isCritical(vertexIdLeft);
                        auto scalarLeft = vertex2scalar[vertexIdLeft];
                        LogProcessorWarn("The left neighbor is "
                                         << (isVertexLeftCritical ? "critical" : "regular")
                                         << " with value " << scalarLeft << " and vertexID "
                                         << vertexIdLeft << " .");
                        auto vertexIdRight = pos2vertex[pos + 1];
                        bool isVertexRightCritical = isCritical(vertexIdLeft);
                        auto scalarRight = vertex2scalar[vertexIdRight];
                        LogProcessorWarn("The right neighbor is "
                                         << (isVertexRightCritical ? "critical" : "regular")
                                         << " with value " << scalarRight << " and vertexID "
                                         << vertexIdRight << " .");
                    }
                }
            }
        }
    }

    sumCorrectCritical_.set(
        std::accumulate(sumCorrectCritical.begin(), sumCorrectCritical.end(), 0));
    sumCorrectRegular_.set(std::accumulate(sumCorrectRegular.begin(), sumCorrectRegular.end(), 0));
    sumIncorrectCritical_.set(
        std::accumulate(sumIncorrectCritical.begin(), sumIncorrectCritical.end(), 0));
    sumIncorrectRegular_.set(
        std::accumulate(sumIncorrectRegular.begin(), sumIncorrectRegular.end(), 0));

    auto dataFrame = std::make_shared<DataFrame>();
    dataFrame->addColumnFromBuffer("Correct Critical",
                                   util::makeBuffer<int>(std::move(sumCorrectCritical)));
    dataFrame->addColumnFromBuffer("Incorrect Critical",
                                   util::makeBuffer<int>(std::move(sumIncorrectCritical)));
    dataFrame->addColumnFromBuffer("Correct Regular",
                                   util::makeBuffer<int>(std::move(sumCorrectRegular)));
    dataFrame->addColumnFromBuffer("Incorrect Regular",
                                   util::makeBuffer<int>(std::move(sumIncorrectRegular)));
    dataFrame->addColumnFromBuffer("Multi-Saddles", util::makeBuffer<int>(std::move(multiSaddles)));
    dataFrame->addColumnFromBuffer("Short SuperArcs",
                                   util::makeBuffer<int>(std::move(shortSuperArcs)));
    dataFrame->updateIndexBuffer();

    outport_.setData(dataFrame);
}

}  // namespace inviwo
