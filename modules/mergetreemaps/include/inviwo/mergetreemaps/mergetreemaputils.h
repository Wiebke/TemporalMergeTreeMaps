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
#pragma once

#include <inviwo/mergetreemaps/mergetreemapsmoduledefine.h>
#include <inviwo/dataframe/datastructures/dataframe.h>
#include <inviwo/topologytoolkit/ports/contourtreeport.h>
#include <inviwo/topologytoolkit/datastructures/triangulationdata.h>
#include <inviwo/mergetreemaps/topologysequenceports.h>
#include <modules/temporaltreemaps/datastructures/treeport.h>
#include <inviwo/core/processors/poolprocessor.h>

namespace inviwo {

namespace mtmutils {

struct TimeStepMapper {

    TimeStepMapper(std::shared_ptr<const topology::TriangulationData> TriangulationForMapping) {

        triangulation = TriangulationForMapping;
        auto gridExtend = triangulation->getGridExtent();
        auto gridOffset = triangulation->getGridOrigin();
        auto gridDims = triangulation->getGridDimensions();

        zOffset = gridOffset.z;
        zSpacing = (gridExtend / vec3(gridDims)).z;
    };

    int getTimeStep(size_t vertexId) const {
        auto pos = triangulation->getPoint(vertexId);
        return static_cast<int>((pos.z - zOffset) / zSpacing);
    }

    vec3 getPoint(size_t vertexId) const { return triangulation->getPoint(vertexId); }

    std::shared_ptr<const topology::TriangulationData> triangulation;
    float zSpacing;
    float zOffset;
};

template <typename ScalarType>
struct Landscape {

    Landscape(size_t Size, bool Positions, size_t Dims) : sizeLandscape(Size) {
        vertexIDs = std::vector<int>(sizeLandscape);
        segmentations = std::vector<int>(sizeLandscape);
        mapPositions = std::vector<int>(sizeLandscape);
        scalars = std::vector<ScalarType>(sizeLandscape);
        if (Positions) {
            if (Dims > 2) {
                posZ = std::vector<float>(sizeLandscape);
            }
            if (Dims > 1) {
                posY = std::vector<float>(sizeLandscape);
            }
            posX = std::vector<float>(sizeLandscape);
        }
    };

    std::shared_ptr<DataFrame> createDataFrame() {
        auto dataframe = std::make_shared<DataFrame>();
        dataframe->addColumnFromBuffer("Vertex ID", util::makeBuffer<int>(std::move(vertexIDs)));
        dataframe->addColumnFromBuffer("Super Arc ID",
                                       util::makeBuffer<int>(std::move(segmentations)));
        dataframe->addColumnFromBuffer("Map Position",
                                       util::makeBuffer<int>(std::move(mapPositions)));
        dataframe->addColumnFromBuffer("Scalar Value",
                                       util::makeBuffer<ScalarType>(std::move(scalars)));
        if (posX.size() == sizeLandscape) {
            dataframe->addColumnFromBuffer("X Position", util::makeBuffer<float>(std::move(posX)));
        }
        if (posY.size() == sizeLandscape) {
            dataframe->addColumnFromBuffer("Y Position", util::makeBuffer<float>(std::move(posY)));
        }
        if (posZ.size() == sizeLandscape) {
            dataframe->addColumnFromBuffer("Z Position", util::makeBuffer<float>(std::move(posZ)));
        }
        dataframe->updateIndexBuffer();
        return dataframe;
    }

    void set(size_t index, int vertexId, int segmentation, int mapPosition, ScalarType scalar,
             vec3 position) {
        vertexIDs[index] = vertexId;
        segmentations[index] = segmentation;
        mapPositions[index] = mapPosition;
        scalars[index] = scalar;
        if (posX.size() == sizeLandscape) {
            posX[index] = position.x;
        }
        if (posY.size() == sizeLandscape) {
            posY[index] = position.y;
        }
        if (posZ.size() == sizeLandscape) {
            posZ[index] = position.z;
        }
    }

    size_t sizeLandscape;
    std::vector<int> vertexIDs;
    std::vector<int> segmentations;
    std::vector<int> mapPositions;
    std::vector<ScalarType> scalars;
    /* Optionally */
    std::vector<float> posX;
    std::vector<float> posY;
    std::vector<float> posZ;
};

/* Different Options by which superArcs can be traversed */
enum class SuperArcOrderMode { None, ById, ByIdReverse, BySize, ByDecision, ByTemporalTree };

namespace {
struct ComputeLandscape {
    template <typename Result, typename Format>
    Result operator()(pool::Progress progress, const topology::ContourTreeData& treeData,
                      const topology::TriangulationData& originalScalars, const size_t timeStep,
                      const bool mapOriginalScalars, const bool useUnaugmentedTreeOnly,
                      const bool includePosition, const size_t dataDims,
                      const SuperArcOrderMode& orderMode, const VertexOrderMap& vertexOrderMap,
                      const std::vector<int>& decisions) {
        // Passing local variables as references, but must copy the input data
        using ScalarType = Format::type;

        /* Extract input */
        auto tree = treeData.getTree();
        auto type = treeData.type;
        const size_t numNodes = tree->getNumberOfNodes();
        const size_t numVertices = tree->getNumberOfVertices();
        const BufferRAM* scalarBuffer;
        if (mapOriginalScalars) {
            scalarBuffer = originalScalars.getScalarValues()->getRepresentation<BufferRAM>();
        } else {
            scalarBuffer =
                treeData.triangulation->getScalarValues()->getRepresentation<BufferRAM>();
        }

        /* Setup of output */
        auto landscape = Landscape<ScalarType>(numVertices, includePosition, dataDims);
        ttk::ftm::idNode rootNodeId = mtmutils::findRoot(tree);

        std::vector<size_t> sizeBelow(numNodes, 0);
        std::map<ttk::ftm::idNode, size_t> orderMap;
        mtmutils::accumulateOrderAndSizes(tree, vertexOrderMap, rootNodeId, sizeBelow, orderMap);

        size_t outputIndex = 0;
        mtmutils::landscapeTraverseTree<ScalarType>(
            tree, type, scalarBuffer, originalScalars, mapOriginalScalars, useUnaugmentedTreeOnly,
            orderMode, sizeBelow, orderMap, decisions, rootNodeId, landscape, outputIndex, 0,
            numVertices - 1, true);

        auto dataframe = landscape.createDataFrame();
        return dataframe;
    }
};

struct ComputeLandscapeSpaceTime {
    template <typename Result, typename Format>
    Result operator()(pool::Progress progress, const topology::ContourTreeData& treeData,
                      const size_t& numTimeSteps, const bool includePositions,
                      const size_t dataDims) {
        // Passing local variables as references, but must copy the input data
        using ScalarType = Format::type;

        /* Extract input */
        auto tree = treeData.getTree();
        auto type = treeData.type;
        const size_t numNodes = tree->getNumberOfNodes();
        const size_t numVertices = tree->getNumberOfVertices();
        const BufferRAM* scalarBuffer =
            treeData.triangulation->getScalarValues()->getRepresentation<BufferRAM>();

        /* Setup of output */
        const int numVerticesPerTimeStep = numVertices / numTimeSteps;
        std::vector<Landscape<ScalarType>> landscapes(
            numTimeSteps,
            Landscape<ScalarType>(numVerticesPerTimeStep, includePositions, dataDims));
        ttk::ftm::idNode rootNodeId = mtmutils::findRoot(tree);

        std::vector<std::vector<size_t>> sizeBelowNode(numTimeSteps,
                                                       std::vector<size_t>(numNodes, 0));
        std::vector<std::vector<size_t>> sizeOnEdge(
            numTimeSteps, std::vector<size_t>(tree->getNumberOfSuperArcs(), 0));
        TimeStepMapper pointMapping(treeData.triangulation);
        mtmutils::accumulateSizesPerTimeStep(tree, pointMapping, numTimeSteps, rootNodeId,
                                             sizeBelowNode, sizeOnEdge);

        std::vector<size_t> outputIndices(numTimeSteps, 0);
        std::vector<int> leftIndices(numTimeSteps, 0);
        std::vector<int> rightIndices(numTimeSteps, numVerticesPerTimeStep - 1);
        std::vector<bool> left(numTimeSteps, true);

        mtmutils::landscapeTraverseTreeSpaceTime<ScalarType>(
            tree, type, scalarBuffer, pointMapping, numTimeSteps, sizeBelowNode, sizeOnEdge,
            rootNodeId, landscapes, outputIndices, leftIndices, rightIndices, left);

        auto output = std::make_shared<DataFrameSequence>(numTimeSteps);

        for (size_t t = 0; t < numTimeSteps; t++) {
            output->at(t) = landscapes[t].createDataFrame();
        }

        return output;
    }
};
}  // namespace

/**
 * \brief Compute the index mapping from temporal tree to contour trees, uses given time step if
 * there is only a single one
 *
 * @param timeStep (optional)
 * @return index mapping mapping from countour tree indices to temporal tree node indices*/

bool IVW_MODULE_MERGETREEMAPS_API getVertexOrderPerTimeStepFromTemporal(
    const kth::TemporalTree& temporalTree, const ContourTreeSequence& contourTrees,
    std::vector<VertexOrderMap>& vertexOrderMapPerTimeStep, size_t timeStep = 0);

/* Find root of a merge tree */
ttk::ftm::idNode IVW_MODULE_MERGETREEMAPS_API findRoot(ttk::ftm::FTMTree_MT* tree);

/* For each node, compute number of nodes below (including itself) and ordering in the layout:
the root includes all nodes, leaves have value one
Assumed that sizeBelow is initialized with the correct size (number of nodes and value 0)*/
std::pair<int, size_t> IVW_MODULE_MERGETREEMAPS_API
accumulateOrderAndSizes(ttk::ftm::FTMTree_MT* tree, const VertexOrderMap& vertexOrderMap,
                        const ttk::ftm::idNode& nodeId, std::vector<size_t>& sizeBelow,
                        std::map<ttk::ftm::idNode, size_t>& orderMap);

void IVW_MODULE_MERGETREEMAPS_API accumulateSizesPerTimeStep(
    ttk::ftm::FTMTree_MT* tree, const TimeStepMapper& pointMapping, const size_t numTimeSteps,
    const ttk::ftm::idNode& nodeId, std::vector<std::vector<size_t>>& sizeBelowNode,
    std::vector<std::vector<size_t>>& sizeOnEdge);

/* Sort superarcs according to the given order */
void IVW_MODULE_MERGETREEMAPS_API landscapeSortSuperArcsOrder(
    ttk::ftm::FTMTree_MT* tree, const std::map<ttk::ftm::idNode, size_t>& orderMap,
    std::vector<ttk::ftm::idSuperArc>& superArcIds);

/* Sort superarcs according to decision (0,1 at each super arc) */
void IVW_MODULE_MERGETREEMAPS_API landscapeSortSuperArcsDecisions(
    ttk::ftm::FTMTree_MT* tree, const ttk::ftm::idSuperArc& idParentArc,
    const std::vector<int>& decisions, std::vector<ttk::ftm::idSuperArc>& superArcIds);

/* Sort superarcs according to the given sizes below */
void IVW_MODULE_MERGETREEMAPS_API
landscapeSortSuperArcsSize(ttk::ftm::FTMTree_MT* tree, const std::vector<size_t>& sizeBelow,
                           std::vector<ttk::ftm::idSuperArc>& superArcIds);

void IVW_MODULE_MERGETREEMAPS_API landscapeSortSuperArcs(
    const SuperArcOrderMode& orderMode, ttk::ftm::FTMTree_MT* tree,
    const std::vector<size_t>& sizeBelow, const ttk::ftm::idSuperArc& idParentArc,
    const std::vector<int>& decisions, const std::map<ttk::ftm::idNode, size_t>& orderMap,
    std::vector<ttk::ftm::idSuperArc>& superArcIds);

template <typename ScalarType>
bool IVW_MODULE_MERGETREEMAPS_API landscapeTraverseSuperArc(
    const ttk::ftm::FTMTree_MT* tree, const topology::TreeType& type, const BufferRAM* scalarBuffer,
    const topology::TriangulationData& originalScalars, bool mapOriginalScalars,
    const ttk::ftm::SuperArc* superArc, Landscape<ScalarType>& landscape, size_t& outputIndex,
    int& xLeft, int& xRight) {

    int assumed_left_after = xLeft + superArc->regionSize() / 2;
    int assumed_right_after = xRight - (superArc->regionSize() + 1) / 2;

    // Alright start on the right (arbitrary choice)
    bool left = false;

    ttk::SimplexId vertexId;
    size_t arcSize = superArc->regionSize();

    /***
        *Now obsolete, mapping original unsimplified data does not make sense
        *keeping code around in case this needs to be shown again later
        *
        if (mapOriginalScalars) {
        std::vector<std::pair<int, ScalarType>> vertexIdsAndScalars(arcSize);
        int segmentation = 0;
        for (size_t i = 0; i < arcSize; i++) {
            vertexId = superArc->getRegularNodeId(i);
            if (i == 0) {
                segmentation = tree->getCorrespondingSuperArcId(vertexId);
            }
            vertexIdsAndScalars[i] = std::make_pair(
                vertexId, static_cast<ScalarType>(scalarBuffer->getAsDouble(vertexId)));
        }
        // Sort vertexIds in the arc according to their scalar and depending on the tree type
        std::sort(
            vertexIdsAndScalars.begin(), vertexIdsAndScalars.end(),
            [&originalScalars, &type](const std::pair<int, ScalarType>& a,
                                      const std::pair<int, ScalarType>& b) -> bool {
                // We want maxima first
                if (type == topology::TreeType::Join) {
                    if (a.second == b.second) {
                        auto posA = originalScalars.getPoint(a.first);
                        auto posB = originalScalars.getPoint(b.first);
                        return std::tie(posA.x, posA.y, posA.z) > std::tie(posB.x, posB.y, posB.z);
                    }
                    return a.second > b.second;
                } else {
                    if (a.second == b.second) {
                        auto posA = originalScalars.getPoint(a.first);
                        auto posB = originalScalars.getPoint(b.first);
                        return std::tie(posA.x, posA.y, posA.z) < std::tie(posB.x, posB.y, posB.z);
                    }
                    return a.second < b.second;
                }
            });

        for (auto vertexIdScalarPair : vertexIdsAndScalars) {
            auto vertexId = vertexIdScalarPair.first;
            // Todo: Should be the same for the entire arc
            auto segmentation = tree->getCorrespondingSuperArcId(vertexId);
            auto mapPosition = left ? xLeft++ : xRight--;
            auto scalar = vertexIdScalarPair.second;
            landscape.set(outputIndex, vertexId, segmentation, mapPosition, scalar,
                          originalScalars.getPoint(vertexId));
            // Alternate between left and right
            left = left ? false : true;
            outputIndex++;
        }
    } else { */
    int segmentation = 0;
    for (size_t i = 0; i < arcSize; i++) {
        if (type == topology::TreeType::Join)
            // Traverse in opposite order, super arc node ids are *always* sorted by value
            vertexId = superArc->getRegularNodeId(arcSize - 1 - i);
        else
            vertexId = superArc->getRegularNodeId(i);
        if (i == 0) {
            // Segmentation is the same for the entire arc, needs to be set only once
            segmentation = tree->getCorrespondingSuperArcId(vertexId);
        }
        auto mapPosition = left ? xLeft++ : xRight--;
        auto scalar = static_cast<ScalarType>(scalarBuffer->getAsDouble(vertexId));
        landscape.set(outputIndex, vertexId, segmentation, mapPosition, scalar,
                      originalScalars.getPoint(vertexId));
        // Alternate between left and right
        left = left ? false : true;
        outputIndex++;
    }
    /*}*/

    if (xLeft != assumed_left_after) {
        std::cout << "xLeft does not have assumed value" << xLeft << "!=" << assumed_left_after;
    }

    if (xRight != assumed_right_after) {
        std::cout << "xRight does not have assumed value" << xRight << "!=" << assumed_right_after;
    }

    return left;
}

template <typename ScalarType>
void IVW_MODULE_MERGETREEMAPS_API landscapeTraverseTree(
    ttk::ftm::FTMTree_MT* tree, const topology::TreeType& type, const BufferRAM* scalarBuffer,
    const topology::TriangulationData& originalScalars, const bool mapOriginalScalars,
    const bool useUnaugmentedTreeOnly, const SuperArcOrderMode& orderMode,
    const std::vector<size_t>& sizeBelow, const std::map<ttk::ftm::idNode, size_t>& orderMap,
    const std::vector<int>& decisions, const ttk::ftm::idNode& nodeId,
    Landscape<ScalarType>& landscape, size_t& outputIndex, int xLeft, int xRight, bool left) {

    auto node = tree->getNode(nodeId);
    auto numDown = node->getNumberOfDownSuperArcs();

    auto vertexId = node->getVertexId();
    // Node in the tree should have unchanged scalar value
    auto scalar = static_cast<ScalarType>(scalarBuffer->getAsDouble(vertexId));
    // Set mapPosition to -1 to signify it is unassigned so far
    int mapPosition = -1;
    // If placing node first, this would be:
    // = left ? xLeft++ : xRight--;
    // But, we need to place the node in between its children and thus only process later
    int segmentation;
    /* Super nodes in the tree get assigned the negative node id in mapping to superArc Id*/
    if (node->getNumberOfUpSuperArcs() >= 1) {
        // Assign leaf and saddles to their up superArcId (there will only be one)
        segmentation = node->getUpSuperArcId(0);
    } else {
        // This is the case *only* for the root node, assign it to the first one
        segmentation = node->getDownSuperArcId(0);
        // Position for root is always on the left, can already be assigned now
        mapPosition = xLeft++;
    }

    // Save outputindex, such that the node will be placed next according to index
    size_t nodeOutputIndex = outputIndex;

    outputIndex++;

    // Leaf
    if (numDown == 0) {
        if (xRight != xLeft) {
            std::cout << "xLeft and xRight should be equal, but are not " << xLeft
                      << "!=" << xRight;
        }
        landscape.set(nodeOutputIndex, vertexId, segmentation, xLeft, scalar,
                      originalScalars.getPoint(vertexId));
        return;
    }

    int xLeftArc = xLeft;

    // Not leaf
    std::vector<ttk::ftm::idSuperArc> superArcIds;
    for (ttk::ftm::idSuperArc i = 0; i < numDown; ++i) {
        auto superArcId = node->getDownSuperArcId(i);
        superArcIds.push_back(superArcId);
    }
    // Decisions are done regarding the up-arcs
    ttk::ftm::idSuperArc upArcId = node->getNumberOfUpSuperArcs() != 0
                                       ? node->getUpSuperArcId(0)
                                       : tree->getNumberOfSuperArcs();
    if (superArcIds.size() > 1)
        mtmutils::landscapeSortSuperArcs(orderMode, tree, sizeBelow, upArcId, decisions, orderMap,
                                         superArcIds);

    size_t arcCounter = 0;
    for (auto superArcId : superArcIds) {
        auto superArc = tree->getSuperArc(superArcId);
        // Needs to take sizeBelow Into Account
        auto downNodeId = superArc->getDownNodeId();
        // xLeftArc und xRightArc are both indices ->
        int xRightArc = xLeftArc + superArc->regionSize() + sizeBelow[downNodeId] - 1;
        int xLeftArcNext = xRightArc + 1;
        // If we are currently processing the first arc, place the supernode afterwards
        // unless we are processing the root arc (mapPosition already assigned)
        if (arcCounter == 0 && mapPosition == -1) {
            mapPosition = xLeftArcNext++;
        }
        // Just a placeholder
        bool left = false;
        if (!useUnaugmentedTreeOnly) {
            // Traverse superarc and update left and right based on its
            left = mtmutils::landscapeTraverseSuperArc<ScalarType>(
                tree, type, scalarBuffer, originalScalars, mapOriginalScalars, superArc, landscape,
                outputIndex, xLeftArc, xRightArc);
        } else {
            bool leftArc = true;
            auto arcSize = superArc->regionSize();
            // Get scalar value of down node
            auto downNode = tree->getNode(downNodeId);
            auto downVertexId = downNode->getVertexId();
            // Node in the tree should have unchanged scalar value
            auto scalarDown = static_cast<ScalarType>(scalarBuffer->getAsDouble(downVertexId));
            // Linearly interpolate between values of the super nodes (scalar for upnode and
            // scalarDown for downNode) scalar is at index 0, the superarc filles 1..arcSize, scalar
            // is at index arcSize+1 this means we take arcSize+1 steps and each step covers
            // 1/(arcSize+1) of the distance between beginning and end
            float increment = 1.0f / static_cast<float>((arcSize + 1));
            auto segmentationRegularNode = superArcId;
            for (size_t i = 0; i < arcSize; i++) {
                auto mapPositionRegularNode = leftArc ? xLeftArc++ : xRightArc--;
                // At i, we take the i+1th step towards the down node
                float fraction = (i + 1) * increment;
                auto scalarRegularNode = (1 - fraction) * scalar + fraction * scalarDown;
                landscape.set(outputIndex, -1, segmentationRegularNode, mapPositionRegularNode,
                              scalarRegularNode, vec3(0, 0, 0));
                // Alternate between left and right
                leftArc = leftArc ? false : true;
                outputIndex++;
            }
            left = leftArc;
        }
        // Reminder: Function above adapts outputIndex, xLeftArc and xRightArc
        mtmutils::landscapeTraverseTree<ScalarType>(
            tree, type, scalarBuffer, originalScalars, mapOriginalScalars, useUnaugmentedTreeOnly,
            orderMode, sizeBelow, orderMap, decisions, downNodeId, landscape, outputIndex, xLeftArc,
            xRightArc, left);
        xLeftArc = xLeftArcNext;
        arcCounter++;
    }

    // Place supernode
    landscape.set(nodeOutputIndex, vertexId, segmentation, mapPosition, scalar,
                  originalScalars.getPoint(vertexId));
}

template <typename ScalarType>
void IVW_MODULE_MERGETREEMAPS_API landscapeTraverseSuperArcSpaceTime(
    const ttk::ftm::FTMTree_MT* tree, const topology::TreeType& type, const BufferRAM* scalarBuffer,
    const TimeStepMapper& pointMapping, const size_t numTimeSteps,
    const ttk::ftm::SuperArc* superArc, std::vector<Landscape<ScalarType>>& landscapes,
    std::vector<size_t>& outputIndices, std::vector<int>& xLeft, std::vector<int>& xRight,
    std::vector<bool>& left) {

    // ToDo: needs to be checked for every timesep
    // int assumed_left_after = xLeft + superArc->regionSize() / 2;
    // int assumed_right_after = xRight - (superArc->regionSize() + 1) / 2;

    // Alright start on the right (arbitrary choice)
    std::vector<bool> leftLocal(numTimeSteps, false);

    ttk::SimplexId vertexId;
    size_t arcSize = superArc->regionSize();

    int segmentation = 0;
    int timeStep = 0;
    for (size_t i = 0; i < arcSize; i++) {
        if (type == topology::TreeType::Join)
            // Traverse in opposite order, super arc node ids are *always* sorted by value
            vertexId = superArc->getRegularNodeId(arcSize - 1 - i);
        else
            vertexId = superArc->getRegularNodeId(i);
        if (i == 0) {
            // Segmentation is the same for the entire arc, needs to be set only once
            segmentation = tree->getCorrespondingSuperArcId(vertexId);
        }
        timeStep = pointMapping.getTimeStep(vertexId);
        auto mapPosition = leftLocal[timeStep] ? xLeft[timeStep]++ : xRight[timeStep]--;
        auto scalar = static_cast<ScalarType>(scalarBuffer->getAsDouble(vertexId));
        landscapes[timeStep].set(outputIndices[timeStep], vertexId, segmentation, mapPosition,
                                 scalar, pointMapping.getPoint(vertexId));
        // Alternate between left and right
        leftLocal[timeStep] = leftLocal[timeStep] ? false : true;
        outputIndices[timeStep]++;
    }

    /*if (xLeft != assumed_left_after) {
        std::cout << "xLeft does not have assumed value" << xLeft << "!=" << assumed_left_after;
    }

    if (xRight != assumed_right_after) {
        std::cout << "xLeft does not have assumed value" << xRight << "!=" << assumed_right_after;
    }*/
}

template <typename ScalarType>
void IVW_MODULE_MERGETREEMAPS_API landscapeTraverseTreeSpaceTime(
    ttk::ftm::FTMTree_MT* tree, const topology::TreeType& type, const BufferRAM* scalarBuffer,
    const TimeStepMapper& pointMapping, const size_t numTimeSteps,
    const std::vector<std::vector<size_t>>& sizeBelowNode,
    std::vector<std::vector<size_t>>& sizeOnEdge, const ttk::ftm::idNode& nodeId,
    std::vector<Landscape<ScalarType>>& landscapes, std::vector<size_t>& outputIndices,
    std::vector<int>& xLeft, std::vector<int>& xRight, std::vector<bool>& left) {

    auto node = tree->getNode(nodeId);
    auto numDown = node->getNumberOfDownSuperArcs();

    auto vertexId = node->getVertexId();
    auto timeStep = pointMapping.getTimeStep(vertexId);
    // Node in the tree should have unchanged scalar value
    auto scalar = static_cast<ScalarType>(scalarBuffer->getAsDouble(vertexId));
    auto mapPosition = left[timeStep] ? xLeft[timeStep]++ : xRight[timeStep]--;
    int segmentation;
    /* Super nodes in the tree get assigned the negative node id in mapping to superArc Id*/
    if (node->getNumberOfUpSuperArcs() >= 1) {
        // Assign leaf and saddles to their up superArcId (there will only be one)
        segmentation = node->getUpSuperArcId(0);
    } else {
        // This is the case *only* for the root node, assign it to the first one
        segmentation = node->getDownSuperArcId(0);
    }

    landscapes[timeStep].set(outputIndices[timeStep], vertexId, segmentation, mapPosition, scalar,
                             pointMapping.getPoint(vertexId));

    outputIndices[timeStep]++;

    // Leaf
    if (numDown == 0) {
        return;
    }

    std::vector<int> xLeftArc = xLeft;

    // Not leaf
    std::vector<ttk::ftm::idSuperArc> superArcIds;
    for (ttk::ftm::idSuperArc i = 0; i < numDown; ++i) {
        auto superArcId = node->getDownSuperArcId(i);
        superArcIds.push_back(superArcId);
    }
    // For now: traverse super Arcs in given order
    for (auto superArcId : superArcIds) {
        auto superArc = tree->getSuperArc(superArcId);
        // Needs to take sizeBelow Into Account
        auto downNodeId = superArc->getDownNodeId();
        // xLeftArc und xRightArc are both indices ->
        std::vector<int> xRightArc = xLeftArc;
        std::vector<int> xLeftArcNext(numTimeSteps);
        for (size_t t = 0; t < numTimeSteps; t++) {
            xRightArc[t] += sizeOnEdge[t][superArcId] + sizeBelowNode[t][downNodeId] - 1;
            xLeftArcNext[t] = xRightArc[t] + 1;
        }
        // Traverse superarc and update left and right based on its
        std::vector<bool> left(numTimeSteps);
        mtmutils::landscapeTraverseSuperArcSpaceTime<ScalarType>(
            tree, type, scalarBuffer, pointMapping, numTimeSteps, superArc, landscapes,
            outputIndices, xLeftArc, xRightArc, left);
        mtmutils::landscapeTraverseTreeSpaceTime<ScalarType>(
            tree, type, scalarBuffer, pointMapping, numTimeSteps, sizeBelowNode, sizeOnEdge,
            downNodeId, landscapes, outputIndices, xLeftArc, xRightArc, left);
        xLeftArc = xLeftArcNext;
    }
}

struct SubTreeInfo {

    SubTreeInfo() : numArcs(0), rootArcIdx(0){};

    SubTreeInfo(size_t NumArcs, ttk::ftm::idSuperArc RootArcIdx)
        : numArcs(NumArcs), rootArcIdx(RootArcIdx) {
        arcSizes = std::vector<size_t>(numArcs);
        treeSizes = std::vector<size_t>(numArcs);
        subTreeArcs = std::vector<std::vector<ttk::ftm::idSuperArc>>(numArcs);
    };

    size_t numArcs;
    ttk::ftm::idSuperArc rootArcIdx;
    std::vector<size_t> arcSizes;
    std::vector<size_t> treeSizes;
    std::vector<std::vector<ttk::ftm::idSuperArc>> subTreeArcs;
};

void accumulateArcs(ttk::ftm::FTMTree_MT* tree, const ttk::ftm::idSuperArc& superArcId,
                    SubTreeInfo& info);

void accumulateArcsTree(ttk::ftm::FTMTree_MT* tree, SubTreeInfo& info);

struct Overlap {

    Overlap() : numSuperArcsFirst(0), numSuperArcsSecond(0){};

    Overlap(size_t NumSuperArcsFirst, size_t NumSuperArcsSecond)
        : numSuperArcsFirst(NumSuperArcsFirst), numSuperArcsSecond(NumSuperArcsSecond) {
        arcArcCount = std::vector<std::vector<size_t>>(
            numSuperArcsFirst, std::vector<size_t>(numSuperArcsSecond, 0.0));
        treeTreeCount = std::vector<std::vector<size_t>>(
            numSuperArcsFirst, std::vector<size_t>(numSuperArcsSecond, 0.0));
    };

    static void computeFraction(const std::vector<size_t>& sizesFirst,
                                const std::vector<size_t>& sizesSecond,
                                const std::vector<std::vector<size_t>>& overlap,
                                std::vector<std::vector<float>>& fraction) {
        auto numFirst = sizesFirst.size();
        auto numSecond = sizesSecond.size();

        fraction = std::vector<std::vector<float>>(numFirst, std::vector<float>(numSecond, 0.0));

        if (overlap.size() < numFirst) return;

        // normalize with number of vertices in the union
        // (sum of sizes minus overlap since that would otherwise be counted twice)
        for (ttk::ftm::idSuperArc i = 0; i < numFirst; i++)
            for (ttk::ftm::idSuperArc j = 0; j < numSecond; j++) {
                if (overlap[i].size() < numSecond) return;
                double overlapCount = static_cast<double>(overlap[i][j]);
                fraction[i][j] = overlapCount / (sizesFirst[i] + sizesSecond[j] - overlapCount);
            }
    }

    void computeArcArcFraction(const std::vector<size_t>& arcSizesFirst,
                               const std::vector<size_t>& arcSizesSecond,
                               std::vector<std::vector<float>>& arcArcFraction) const {
        computeFraction(arcSizesFirst, arcSizesSecond, arcArcCount, arcArcFraction);
    }

    void computeTreeTreeFraction(const std::vector<size_t>& treeSizesFirst,
                                 const std::vector<size_t>& treeSizesSecond,
                                 std::vector<std::vector<float>>& treeTreeFraction) const {
        computeFraction(treeSizesFirst, treeSizesSecond, treeTreeCount, treeTreeFraction);
    }

    void computeTreeTreeCount(const SubTreeInfo& infoFirst, const SubTreeInfo& infoSecond) {
        for (ttk::ftm::idSuperArc i = 0; i < numSuperArcsFirst; i++) {
            const auto& subTreeArcsFirst = infoFirst.subTreeArcs[i];
            for (ttk::ftm::idSuperArc j = 0; j < numSuperArcsSecond; j++) {
                size_t sum = 0;
                const auto& subTreeArcsSecond = infoSecond.subTreeArcs[j];
                for (auto& subTreeArcFirst : subTreeArcsFirst)
                    for (auto& subTreeArcSecond : subTreeArcsSecond) {
                        sum += arcArcCount[subTreeArcFirst][subTreeArcSecond];
                    }
                treeTreeCount[i][j] = sum;
            }
        }
    }

    size_t numSuperArcsFirst;
    size_t numSuperArcsSecond;

    /* overlap between edges/superarcs in two timesteps */
    std::vector<std::vector<size_t>> arcArcCount;
    /* overlap between subtrees in two timesteps */
    std::vector<std::vector<size_t>> treeTreeCount;
};

void computeOverlap1D(const TemplateColumn<int>& superArcLandscapeFirst,
                      const TemplateColumn<int>& mapPositionFirst, const SubTreeInfo& infoFirst,
                      const TemplateColumn<int>& superArcLandscapeSecond,
                      const TemplateColumn<int>& mapPositionSecond, const SubTreeInfo& infoSecond,
                      Overlap& overlap);

void computeOverlapND(const topology::ContourTreeData& treeDataFirst, const SubTreeInfo& infoFirst,
                      const topology::ContourTreeData& treeDataSecond,
                      const SubTreeInfo& infoSecond, Overlap& overlap);

// Prototype for Landscape evaluation in optimization tool
struct LandscapeInfo {

    LandscapeInfo() { limits = std::vector<size_t>(5, 0); }

    size_t getOuterLimMin() const { return limits[0]; };

    size_t getOuterLimMax() const { return limits[1]; };

    size_t getInnerLimMin() const { return limits[2]; };

    size_t getInnerLimMax() const { return limits[3]; };

    size_t getDownNodePos() const { return limits[4]; }

    void setLimits(size_t outerLimMin, size_t outerLimMax, size_t innerLimMin, size_t innerLimMax) {
        IVW_ASSERT(outerLimMin <= outerLimMax,
                   "Minimum is larger than Maximum for outer landscape limit");
        IVW_ASSERT(innerLimMin <= innerLimMax,
                   "Minimum is larger than Maximum for inner landscape limit")
        limits[0] = outerLimMin;
        limits[1] = outerLimMax;
        limits[2] = innerLimMin;
        limits[3] = innerLimMax;
    };

    void setDownNodePos(size_t downNodePos) {
        // downNode Position should be right between innerLimMin and innerLimMax
        IVW_ASSERT(getInnerLimMin() < downNodePos,
                   "Super Node Position is smaller than Minimum for inner landscape limit");
        IVW_ASSERT(getInnerLimMax() > downNodePos,
                   "Super Node Position is larger than Maximum for inner landscape limit")
        limits[4] = downNodePos;
    };

    size_t getSubTreeSize() const { return getOuterLimMax() - getOuterLimMin() + 1; }

    size_t getArcSize() const {
        // Each arc consists of two intervals [outerMin, innerMin], [innerMax, outerMax] and down
        // and the down node position, which can be expressed as an additional interval
        // [downNodePos, downNodePos]
        // The size of each interval [a,b] is b-a+1
        // size of [outerMin, innerMin] = innerMin - innerMin + 1
        size_t size = getInnerLimMin() - getOuterLimMin() + 1;
        // size of [innerMax, outerMax] = outerMax - innerMax + 1
        // size of [downNodePos, downNodePos] = downNodePos - downNodePos + 1 =1
        size += getOuterLimMax() - getInnerLimMax() + 1 + 1;
        return size;
    }

    void computeLimitRoot(const size_t arcSize, const size_t treeSize) {
        // For all cases: If we add number of elements to an index, we have to subtract 1 to get
        // steps. The root arc resembles the entire tree and thus fills all of the landscape
        limits[0] = 0;
        limits[1] = treeSize - 1;
        // Iteration starts at the root node and switches sides for each subsequent node in the
        // arc If the number of nodes is uneven, there will be one more node on the root side
        // as that is where we started
        size_t arcSizeWithoutDownNode = arcSize - 1;
        limits[2] = limits[0] + ((arcSizeWithoutDownNode + 1) / 2 - 1);
        limits[3] = limits[1] - (arcSizeWithoutDownNode / 2 - 1);
    }

    void computeLimit(size_t lastAssigned, const size_t arcSize, const size_t treeSize) {
        // For all cases: If we add number of elements to an index, we have to subtract 1 to get
        // steps
        // !! Changed code to accomodate multiple variables
        // Previously, additional variables const LandscapeInfo& parentLimits, const bool first,
        // We assume a binary tree, i.e. at most 2 down arcs at each inner node, a node can only
        // be first or second limits can be computed from the inner limits of the parent arc,
        // for the first arc, the minimum outer limit depends on the minimum inner limit and the
        // tree size for that arc
        // for the second, the minimum outer limits depends on the maximum outer limit of the
        // first arc - to compute it independently, we instead compute the maximum outer limit
        // in terms of the maximum inner limit of the parent, then the minimum outer limit
        // depends on the tree size for that arc
        /*if (first) {
            limits[0] = parentLimits.getInnerLimMin() + 1;
            limits[1] = limits[0] + (treeSize - 1);
        } else {
            limits[1] = parentLimits.getInnerLimMax() - 1;
            limits[0] = limits[1] - (treeSize - 1);
        }*/
        limits[0] = lastAssigned + 1;
        limits[1] = limits[0] + (treeSize - 1);
        size_t arcSizeWithoutDownNode = arcSize - 1;
        // Independently of the order, each arc starts being traversed at the maximum (to fit
        // with overall starting at the minimum for the root ), thus in case of an uneven arc
        // size, one more node will be placed on the minimum side
        limits[2] = limits[0] + (arcSizeWithoutDownNode / 2 - 1);
        limits[3] = limits[1] - ((arcSizeWithoutDownNode + 1) / 2 - 1);
    }

    std::vector<size_t> limits;

    size_t static overlapInterval(size_t minA, size_t maxA, size_t minB, size_t maxB) {
        // Cases for overlap of interval A [minA, maxA] with interval B [minB, maxB]
        // one is fully before the other -> no overlap
        // A ----xxxxx------ [4,8]
        // B ---------xxxxxx [9,15]
        // -> oS = max(4,9) = 9, oE = min(8,15) = 8, return max(0, 8-9+1) = 0
        // one is partially within the other -> overlap = intersection
        // A ----xxxxx------ [4,8]
        // B -----xxx------- [5,7]
        // -> oS = max(4,5) = 5, oE = min(8,7) = 7, return max(0, 7-5+1) = 3
        // one is partially within the other -> overlap = intersection
        // A ----xxxxx------ [4,8]
        // B --xxxxxx------- [2,7]
        // -> oS = max(4,2) = 4, oE = min(8,7) = 7, return max(0, 7-4+1) = 4
        int overlapStart = static_cast<int>(std::max(minA, minB));
        int overlapEnd = static_cast<int>(std::min(maxA, maxB));
        return std::max(0, overlapEnd - overlapStart + 1);
    }

    size_t static overLapCountTrees(const LandscapeInfo& first, const LandscapeInfo& second) {
        // essentially computation of overlap of two intervals [minA, maxA], [minB, maxB]
        return overlapInterval(first.getOuterLimMin(), first.getOuterLimMax(),
                               second.getOuterLimMin(), second.getOuterLimMax());
    };

    float static overLapFractionTrees(const LandscapeInfo& first, const LandscapeInfo& second) {
        size_t overlap = LandscapeInfo::overLapCountTrees(first, second);
        size_t sizeFirst = first.getSubTreeSize();
        size_t sizeSecond = second.getSubTreeSize();
        // Normalize by union (sum of sizes - overlap, overlap would be counted twice otherwise)
        return overlap / static_cast<float>(sizeFirst + sizeSecond - overlap);
    };

    size_t static overLapCountArcs(const LandscapeInfo& first, const LandscapeInfo& second) {
        // Each arc consists of two intervals [outerMin, innerMin], [innerMax, outerMax]
        // and the down node position, which can be expressed as an additional interval
        // [downNodePos, downNodePos]
        // So for the overlap we need to compute 3*3 interval intersections
        size_t overlap = 0;
        // overlap for intervals first.mins and second.mins
        overlap += overlapInterval(first.getOuterLimMin(), first.getInnerLimMin(),
                                   second.getOuterLimMin(), second.getInnerLimMin());
        // overlap for intervals first.mins and second.maxs
        overlap += overlapInterval(first.getOuterLimMin(), first.getInnerLimMin(),
                                   second.getInnerLimMax(), second.getOuterLimMax());
        // overlap for intervals first.mins and second.downNode
        overlap += overlapInterval(first.getOuterLimMin(), first.getInnerLimMin(),
                                   second.getDownNodePos(), second.getDownNodePos());
        // overlap for intervals first.maxs and second.mins
        overlap += overlapInterval(first.getInnerLimMax(), first.getOuterLimMax(),
                                   second.getOuterLimMin(), second.getInnerLimMin());
        // overlap for intervals first.maxs and second.maxs
        overlap += overlapInterval(first.getInnerLimMax(), first.getOuterLimMax(),
                                   second.getInnerLimMax(), second.getOuterLimMax());
        // overlap for intervals first.mins and second.downNode
        overlap += overlapInterval(first.getInnerLimMax(), first.getOuterLimMax(),
                                   second.getDownNodePos(), second.getDownNodePos());
        // overlap for intervals first.downNode and second.mins
        overlap += overlapInterval(first.getDownNodePos(), first.getDownNodePos(),
                                   second.getOuterLimMin(), second.getInnerLimMin());
        // overlap for intervals first.downNode and second.maxs
        overlap += overlapInterval(first.getDownNodePos(), first.getDownNodePos(),
                                   second.getInnerLimMax(), second.getOuterLimMax());
        // overlap for intervals first.downNode and second.downNode
        overlap += overlapInterval(first.getDownNodePos(), first.getDownNodePos(),
                                   second.getDownNodePos(), second.getDownNodePos());

        return overlap;
    }

    float static overLapFractionArcs(const LandscapeInfo& first, const LandscapeInfo& second) {
        size_t overlap = LandscapeInfo::overLapCountArcs(first, second);
        size_t sizeFirst = first.getArcSize();
        size_t sizeSecond = second.getArcSize();
        // Normalize by union (sum of sizes - overlap, overlap would be counted twice otherwise)
        return overlap / static_cast<float>(sizeFirst + sizeSecond - overlap);
    }
};
void accumulateLimits(ttk::ftm::FTMTree_MT* tree, const SubTreeInfo& info,
                      const SuperArcOrderMode& orderMode,
                      const std::map<ttk::ftm::idNode, size_t>& orderMap,
                      const std::vector<int>& decisions, const ttk::ftm::idSuperArc& superArcId,
                      const size_t& lastAssigned, std::vector<LandscapeInfo>& landscapes);

void accumulateLimitsTree(ttk::ftm::FTMTree_MT* tree, const SubTreeInfo& info,
                          const SuperArcOrderMode& orderMode,
                          const std::map<ttk::ftm::idNode, size_t>& orderMap,
                          const std::vector<int>& decisions,
                          std::vector<LandscapeInfo>& landscapes);

float objectiveForSeveralArcs(const SubTreeInfo& infoFromPrev,
                              const std::vector<LandscapeInfo>& landscapeFromPrev,
                              const Overlap& overlapND, const bool useCounts,
                              const std::vector<ttk::ftm::idSuperArc>& superArcIds,
                              const std::vector<LandscapeInfo>& landscapes);

float decideGreedy(ttk::ftm::FTMTree_MT* tree, const SubTreeInfo& infoFromPrev,
                   const std::vector<LandscapeInfo>& landscapeFromPrev, const SubTreeInfo& info,
                   const Overlap& overlapND, const ttk::ftm::idSuperArc& superArcId,
                   const bool useCounts, const bool chooseWorse, std::vector<int>& decisions,
                   std::vector<LandscapeInfo>& landscape);

float decideGreedyTree(ttk::ftm::FTMTree_MT* tree, const SubTreeInfo& infoFromPrev,
                       const std::vector<LandscapeInfo>& landscapeFromPrev, const SubTreeInfo& info,
                       const Overlap& overlapND, const bool useCounts, const bool chooseWorse,
                       std::vector<int>& decisions, std::vector<LandscapeInfo>& landscape);

void computeOverlap1D(const std::vector<LandscapeInfo>& superArcLandscapesFirst,
                      const SubTreeInfo& infoFirst,
                      const std::vector<LandscapeInfo>& superArcLandscapesSecond,
                      const SubTreeInfo& infoSecond, Overlap& overlap,
                      bool checkOverLapCountTrees = false);

}  // namespace mtmutils

}  // namespace inviwo
