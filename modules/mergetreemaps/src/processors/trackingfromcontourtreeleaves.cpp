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

#include <inviwo/mergetreemaps/processors/trackingfromcontourtreeleaves.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo TrackingFromContourTreeLeaves::processorInfo_{
    "org.inviwo.TrackingFromContourTreeLeaves",  // Class identifier
    "Tracking From Contour Tree Leaves",         // Display name
    "Topology",                                  // Category
    CodeState::Experimental,                     // Code state
    Tags::None,                                  // Tags
};
const ProcessorInfo TrackingFromContourTreeLeaves::getProcessorInfo() const {
    return processorInfo_;
}

TrackingFromContourTreeLeaves::TrackingFromContourTreeLeaves()
    : Processor()
    , outport_("outport")
    , trackingOutport_("tracking")
    , minOverlapFraction_("minOverlapFraction", "Minimum Overlap Fraction", 0.4, 0.0001, 1.0,
                          0.0001)
    , forceZTranslation_("forceZTranslation", "Force Z-Translation")
    , zTranslation_("zTranslation", "Z-Translation", 1, 0)
    , colors_("colors", "Colors") {

    addPort(treesInport_);
    addPort(outport_);
    addPort(trackingOutport_);
    addProperties(minOverlapFraction_, forceZTranslation_, zTranslation_, colors_);
    zTranslation_.visibilityDependsOn(forceZTranslation_, [](const auto &p) { return p.get(); });
}

void TrackingFromContourTreeLeaves::process() {

    const auto treesData = treesInport_.getData();
    if (!treesData) return;
    auto numTrees = treesData.get()->size();
    LogProcessorInfo(numTrees);
    if (numTrees < 2) return;

    if (treesInport_.isChanged()) {

        subTreeInfos_ = std::vector<mtmutils::SubTreeInfo>(numTrees);
        nDOverlap_ = std::vector<mtmutils::Overlap>(numTrees - 1);

        // Pre-compute all subtree infos
        for (size_t i = 0; i < numTrees; i++) {
            const topology::ContourTreeData &treeData = *treesData.get()->at(i).get();
            mtmutils::accumulateArcsTree(treeData.getTree(), subTreeInfos_[i]);
        }

        // Compute overlap
        for (size_t i = 0; i < numTrees - 1; i++) {
            const topology::ContourTreeData &treeDataFirst = *treesData.get()->at(i).get();
            const mtmutils::SubTreeInfo &infoFirst = subTreeInfos_[i];
            const topology::ContourTreeData &treeDataSecond = *treesData.get()->at(i + 1).get();
            const mtmutils::SubTreeInfo &infoSecond = subTreeInfos_[i + 1];
            mtmutils::computeOverlapND(treeDataFirst, infoFirst, treeDataSecond, infoSecond,
                                       nDOverlap_[i]);
        }
    }

    // Prepare output
    auto outputMesh = std::make_shared<Mesh>();
    auto triangulation = treesData.get()->at(0)->triangulation;
    vec3 offsetFirst = triangulation->getOffset();
    // outputMesh->setBasis(triangulation->getBasis());
    // outputMesh->setOffset(offsetFirst);
    outputMesh->setWorldMatrix(triangulation->getWorldMatrix());
    auto outputDf = std::make_shared<DataFrame>();

    std::vector<vec3> vertices;
    std::vector<vec4> colors;
    // Join tree -> one max that is root, all leaves are mins
    // Split tree -> one min that is root, all leaves are maxs
    const auto &trackedColor = treesData.get()->at(0)->type == topology::TreeType::Join
                                   ? colors_.getColor2D(0)
                                   : colors_.getColor2D(2);
    std::vector<uint32_t> indicesLines;

    std::vector<float> matchingCost;

    std::vector<int> timescalars_1;
    std::vector<ttk::SimplexId> vertexIds_1;

    std::vector<int> timescalars_2;
    std::vector<ttk::SimplexId> vertexIds_2;
    // Maps per timestep
    std::vector<std::map<ttk::SimplexId, std::vector<ttk::SimplexId>>> edgesForward(numTrees);
    std::vector<std::map<ttk::SimplexId, std::vector<ttk::SimplexId>>> edgesBackward(numTrees);

    for (size_t i = 0; i < numTrees - 1; i++) {
        const mtmutils::SubTreeInfo &infoFirst = subTreeInfos_[i];
        const mtmutils::SubTreeInfo &infoSecond = subTreeInfos_[i + 1];
        const mtmutils::Overlap &currentNDOverlap = nDOverlap_[i];
        const auto &treeFirst = treesData.get()->at(i)->getTree();
        const auto &treeSecond = treesData.get()->at(i + 1)->getTree();
        const auto &triangulationFirst = treesData.get()->at(i)->triangulation;
        const auto &triangulationSecond = treesData.get()->at(i + 1)->triangulation;

        // Overlap fraction normalized by subtree size for nD
        std::vector<std::vector<float>> nDOverlapFraction;
        currentNDOverlap.computeArcArcFraction(infoFirst.treeSizes, infoSecond.treeSizes,
                                               nDOverlapFraction);
        for (size_t subTreeIdFirst = 0; subTreeIdFirst < infoFirst.numArcs; subTreeIdFirst++) {
            // Skip non-leaves (subTree contains more than the superArc itself
            if (infoFirst.subTreeArcs[subTreeIdFirst].size() > 1) continue;
            auto superArcFirst = treeFirst->getSuperArc(subTreeIdFirst);
            auto downNodeIdFirst = superArcFirst->getDownNodeId();
            auto downNodeFirst = treeFirst->getNode(downNodeIdFirst);
            auto vertexIdFirst = downNodeFirst->getVertexId();

            for (size_t subTreeIdSecond = 0; subTreeIdSecond < infoSecond.numArcs;
                 subTreeIdSecond++) {
                // Skip non-leaves (subTree contains more than the superArc itself
                if (infoSecond.subTreeArcs[subTreeIdSecond].size() > 1) continue;

                // Skip if the overlap is below the minimum overlap fraction
                const auto &overlapfraction = nDOverlapFraction[subTreeIdFirst][subTreeIdSecond];
                if (overlapfraction < minOverlapFraction_.get()) continue;

                // If we have not yet skipped, than we did find a correspondance!
                auto superArcSecond = treeSecond->getSuperArc(subTreeIdSecond);
                auto downNodeIdSecond = superArcSecond->getDownNodeId();
                auto downNodeSecond = treeSecond->getNode(downNodeIdSecond);
                auto vertexIdSecond = downNodeSecond->getVertexId();

                vertices.push_back(triangulationFirst->getPoint(vertexIdFirst));
                if (forceZTranslation_) vertices.back().z += zTranslation_ * (i);
                indicesLines.push_back(static_cast<uint32_t>(vertices.size() - 1));
                colors.push_back(trackedColor);
                vertices.push_back(triangulationSecond->getPoint(vertexIdSecond));
                if (forceZTranslation_) vertices.back().z += zTranslation_ * (i + 1);
                indicesLines.push_back(static_cast<uint32_t>(vertices.size() - 1));
                colors.push_back(trackedColor);

                vertexIds_1.push_back(vertexIdFirst);
                timescalars_1.push_back(i);
                vertexIds_2.push_back(vertexIdSecond);
                timescalars_2.push_back(i + 1);
                matchingCost.push_back(1.0f - overlapfraction);

                // Helpers for component identification
                edgesForward[i][vertexIdFirst].push_back(vertexIdSecond);
                edgesBackward[i + 1][vertexIdSecond].push_back(vertexIdFirst);
            }
        }
    }

    // Add Buffers to the mesh
    outputMesh->addBuffer(Mesh::BufferInfo(BufferType::PositionAttrib),
                          util::makeBuffer(std::move(vertices)));
    outputMesh->addBuffer(Mesh::BufferInfo(BufferType::ColorAttrib),
                          util::makeBuffer(std::move(colors)));

    if (!indicesLines.empty()) {
        outputMesh->addIndices(Mesh::MeshInfo(DrawType::Lines, ConnectivityType::None),
                               util::makeIndexBuffer(std::move(indicesLines)));
    }

    const int criticalType = treesData.get()->at(0)->type == topology::TreeType::Join ? 0 : 3;
    std::vector<int> pointTypeScalars_1;
    pointTypeScalars_1.resize(vertexIds_1.size());
    std::fill(pointTypeScalars_1.begin(), pointTypeScalars_1.end(), criticalType);
    std::vector<int> pointTypeScalars_2;
    pointTypeScalars_2.resize(vertexIds_2.size());
    std::fill(pointTypeScalars_2.begin(), pointTypeScalars_2.end(), criticalType);

    // Figure out component Ids
    int currentId = 0;
    std::vector<int> componentIds;
    std::map<ttk::SimplexId, int> latestComponentForVertexId;
    componentIds.resize(vertexIds_1.size());
    for (size_t t = 0; t < vertexIds_1.size(); t++) {
        const auto &vertexIdStart = vertexIds_1[t];
        const auto &vertexIdEnd = vertexIds_2[t];
        const auto &i = timescalars_1[t];
        bool startNewComponent = true;
        // If number of backward edges is larger than one then this vertex has two predecessors, a
        // new (merged) track is started here
        auto mergedBeforeDetector = edgesBackward[i].find(vertexIdStart);
        if (mergedBeforeDetector != edgesBackward[i].end() &&
            mergedBeforeDetector->second.size() == 1) {
            // If number of forward edges is larger than one than starting vertex has more than one
            // successor, each successor gets a new id
            auto splittingHereDetector = edgesForward[i].find(vertexIdStart);
            if (splittingHereDetector != edgesForward[i].end() &&
                splittingHereDetector->second.size() == 1) {
            startNewComponent = false;
            // We confirmed one to one correspondance, so vertexIdEnd must have been just updated and is now vertexIdStart
            const auto &lastId = latestComponentForVertexId[vertexIdStart];
            latestComponentForVertexId[vertexIdEnd] = lastId;
            componentIds[t] = lastId;
            }
        }
        // start a new component
        if (startNewComponent) {
            componentIds[t] = currentId;
            latestComponentForVertexId[vertexIdEnd] = currentId;
            currentId++;
        }
    }

    // Add buffers to the dataframe
    outputDf->addColumnFromBuffer("Vertex Id Start", util::makeBuffer<int>(std::move(vertexIds_1)));
    outputDf->addColumnFromBuffer("Critical Type Start",
                                  util::makeBuffer<int>(std::move(pointTypeScalars_1)));
    outputDf->addColumnFromBuffer("Time Start", util::makeBuffer<int>(std::move(timescalars_1)));
    outputDf->addColumnFromBuffer("Vertex Id End", util::makeBuffer<int>(std::move(vertexIds_2)));
    outputDf->addColumnFromBuffer("Critical Type End",
                                  util::makeBuffer<int>(std::move(pointTypeScalars_2)));
    outputDf->addColumnFromBuffer("Time End", util::makeBuffer<int>(std::move(timescalars_2)));
    outputDf->addColumnFromBuffer("Cost", util::makeBuffer<float>(std::move(matchingCost)));
    outputDf->addColumnFromBuffer("Component Id", util::makeBuffer<int>(std::move(componentIds)));

    outputDf->updateIndexBuffer();

    outport_.setData(outputMesh);
    trackingOutport_.setData(outputDf);
}

}  // namespace inviwo
