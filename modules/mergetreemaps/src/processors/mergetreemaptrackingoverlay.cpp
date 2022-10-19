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

#include <inviwo/mergetreemaps/processors/mergetreemaptrackingoverlay.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo MergeTreeMapTrackingOverlay::processorInfo_{
    "org.inviwo.MergeTreeMapTrackingOverlay",  // Class identifier
    "Merge Tree Map Tracking Overlay",         // Display name
    "Merge Tree Maps",                         // Category
    CodeState::Experimental,                   // Code state
    Tags::None,                                // Tags
};
const ProcessorInfo MergeTreeMapTrackingOverlay::getProcessorInfo() const { return processorInfo_; }

MergeTreeMapTrackingOverlay::MergeTreeMapTrackingOverlay()
    : Processor()
    , positionColumn_("position", "Position Columns", dataFramePort_, false, 1)
    , vertexColumn_("color", "Vertex Columns", dataFramePort_, false, 2)
    , trackingColor_("trackingColor", "Tracking Color", vec4(0.0f, 0.0f, 0.0f, 1.0f), vec4(0.0f),
                     vec4(1.0f), vec4(0.01f), InvalidationLevel::InvalidOutput,
                     PropertySemantics::Color)
    , treeType_("treeType", "Tree Type",
                {
                    {"join", "Join Tree (Minima Leaves)", topology::TreeType::Join},
                    {"split", "Split Tree (Maxima Leaves)", topology::TreeType::Split},
                    {"contour", "Contour Tree", topology::TreeType::Contour}
                    //{"joinAndSplit", "Join and Split", topology::TreeType::JoinAndSplit}},
                    // FIXME: for some reason TTK does not support this properly.
                    // The resulting tree has no data
                })
    , timer_("timer", "Eval Time (s)", 0.f, 0.f, std::numeric_limits<float>::max(), 0.001f,
             InvalidationLevel::Valid, PropertySemantics::Text) {
    addPort(inport_);
    addPort(dataFramePort_);
    addPort(trackingInport_);
    addPort(outport_);

    addProperties(treeType_, positionColumn_, vertexColumn_, trackingColor_, timer_);
    timer_.setReadOnly(true);
}

void MergeTreeMapTrackingOverlay::process() {
    performanceTimer_.Reset();
    const auto dfFrames = inport_.getData();
    const auto modelFrame = dataFramePort_.getData();
    const auto type = treeType_.get();
    if (!dfFrames || !modelFrame) return;
    size_t numTimeSteps = dfFrames->size();
    if (numTimeSteps < 1) return;
    size_t numVertices = dfFrames->at(0)->getNumberOfRows();
    auto outputMesh = std::make_shared<Mesh>();

    std::vector<vec3> vertices;
    std::vector<vec4> colors;
    // Join tree -> one max that is root, all leaves are mins
    // Split tree -> one min that is root, all leaves are maxs
    const auto& trackedColor = trackingColor_.get();
    std::vector<uint32_t> indicesLines;

    auto tracking = trackingInport_.getData();
    size_t rows = tracking->getNumberOfRows();
    size_t columns = tracking->getNumberOfColumns();

    if (columns != 9) {
        return;
    }

    vertices.reserve(rows * 2);
    colors.reserve(rows * 2);

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

    const auto getPositionIndexFromVertex =
        [numVertices, vertexColumnIdx = vertexColumn_.get(), posColumnIdx = positionColumn_.get()](
            const DataFrame& dfForTimeStep, int vertexId) -> int {
        auto posColumn = dfForTimeStep.getColumn(posColumnIdx);
        auto vertexColumn = dfForTimeStep.getColumn(vertexColumnIdx);
        // Find the row that contains the vertexId
        for (size_t rowIndex = 0; rowIndex < numVertices; rowIndex++) {
            if (std::stoi(vertexColumn->getAsString(rowIndex)) == vertexId) {
                return std::stoi(posColumn->getAsString(rowIndex));
            }
        }
        return -1;
    };

    for (size_t r(0); r < rows; r++) {

        ttk::CriticalType cpType = ttk::CriticalType(std::stoi(columnCpTypeStart->getAsString(r)));

        // Join tree -> all leaves are minima
        // Split tree -> all leaves are maxima
        // Disregard the respective other
        if ((type == topology::TreeType::Join && cpType != ttk::CriticalType::Local_minimum) ||
            (type == topology::TreeType::Split && cpType != ttk::CriticalType::Local_maximum)) {
            continue;
        }

        auto tStart = std::stoi(columnTimeStart->getAsString(r));
        if (tStart < 0 || tStart >= numTimeSteps) {
            continue;
        }
        auto vertexIdStart = std::stoi(columnIdStart->getAsString(r));
        auto posStart = getPositionIndexFromVertex(*dfFrames->at(tStart), vertexIdStart);
        if (posStart < 0) continue;

        auto tEnd = std::stoi(columnTimeEnd->getAsString(r));
        if (tEnd < 0 || tEnd >= numTimeSteps) {
            continue;
        }
        auto vertexIdEnd = std::stoi(columnIdEnd->getAsString(r));
        auto posEnd = getPositionIndexFromVertex(*dfFrames->at(tEnd), vertexIdEnd);
        if (posEnd < 0) continue;

        // Add track to the mesh
        vertices.push_back(vec3(static_cast<float>(tStart) / (numTimeSteps - 1),
                                static_cast<float>(posStart) / (numVertices - 1), 0.f));
        colors.push_back(trackedColor);
        indicesLines.push_back(static_cast<uint32_t>(vertices.size() - 1));
        vertices.push_back(vec3(static_cast<float>(tEnd) / (numTimeSteps - 1),
                                static_cast<float>(posEnd) / (numVertices - 1), 0.f));
        colors.push_back(trackedColor);
        indicesLines.push_back(static_cast<uint32_t>(vertices.size() - 1));
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

    outport_.setData(outputMesh);
    timer_.set(performanceTimer_.ElapsedTimeAndReset());
}

}  // namespace inviwo
