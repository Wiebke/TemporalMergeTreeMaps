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

#include <inviwo/mergetreemaps/processors/trackingqualityestimator.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo TrackingQualityEstimator::processorInfo_{
    "org.inviwo.TrackingQualityEstimator",  // Class identifier
    "Tracking Quality Estimator",           // Display name
    "Tracking",                             // Category
    CodeState::Experimental,                // Code state
    Tags::None,                             // Tags
};
const ProcessorInfo TrackingQualityEstimator::getProcessorInfo() const { return processorInfo_; }

TrackingQualityEstimator::TrackingQualityEstimator()
    : Processor()
    , colorByDistance_("colorByDistance", "Color By Distance", false)
    , distanceType_("distanceType", "Distance Type")
    , tf_("transferFunction", "Distance Colors")
    , outputGeometricMeansInMesh_("outputGeometricMeansInMesh", "Geometric Mean Tracks", false)
    , accumulatedCritDistance_("accumulatedCritDistance", "Avg Acc Crit Distance", 0.0f, 0.0f,
                               std::numeric_limits<float>::max(), 0.01f,
                               inviwo::InvalidationLevel::InvalidOutput, PropertySemantics::Text)
    , accumulatedGeoMeanDistance_("accumulatedGeoMeanDistance", "Avg Acc Geo Mean Distance", 0.0f,
                                  0.0f, std::numeric_limits<float>::max(), 0.01f,
                                  inviwo::InvalidationLevel::InvalidOutput, PropertySemantics::Text)
    , accumulatedNonOverlapDistance_(
          "accumulatedNonOverlapDistance", "Avg Acc Non-Overlap Distance", 0.0f, 0.0f,
          std::numeric_limits<float>::max(), 0.01f, inviwo::InvalidationLevel::InvalidOutput,
          PropertySemantics::Text)
    , timer_("timer", "Eval Time (s)", 0.f, 0.f, std::numeric_limits<float>::max(), 0.001f,
             InvalidationLevel::Valid, PropertySemantics::Text) {

    addPort(treesInport_);
    addPort(segmentationsInport_);
    addPort(trackingInport_);

    addPort(tracksOutport_);

    addProperties(colorByDistance_, distanceType_, tf_, outputGeometricMeansInMesh_,
                  accumulatedCritDistance_, accumulatedGeoMeanDistance_,
                  accumulatedNonOverlapDistance_, timer_);
    timer_.setReadOnly(true);

    distanceType_.addOption("criticalP", "Euclid CP", 0);
    distanceType_.addOption("geometricMean", "Euclid Geometric Mean", 1);
    distanceType_.addOption("matching", "Tracking Cost", 2);
    distanceType_.setSelectedIndex(0);
}

void TrackingQualityEstimator::process() {
    performanceTimer_.Reset();
    const auto segmentations = segmentationsInport_.getData();
    const auto trees = treesInport_.getData();
    if (segmentations->size() != trees->size() || segmentations->size() < 0 || trees->size() < 0) {
        LogProcessorError("Mismatch between trees and segmentations or empty sequences.") return;
    }
    const auto type = trees->at(0)->type;
    size_t numTimeSteps = trees->size();
    // Number of total vertices per time step, same for every time step
    size_t numVertices = segmentations->at(0)->getPoints().size();

    // Compute geometric means of leaf components
    std::vector<std::vector<vec3>> geometricMeanPositions{numTimeSteps, std::vector<vec3>(0)};
    std::vector<std::vector<size_t>> sizeOfArea{
        std::vector<std::vector<size_t>>(numTimeSteps, std::vector<size_t>(0))};

    for (size_t t = 0; t < numTimeSteps; t++) {
        auto& triangulation = segmentations->at(t);
        auto scalarValues = triangulation->getScalarValues()->getRepresentation<BufferRAM>();
        auto& points = triangulation->getPoints();

        auto& geometricMeanPositionsT = geometricMeanPositions[t];
        auto& sizeOfAreaT = sizeOfArea[t];

        for (size_t v = 0; v < numVertices; v++) {
            size_t superArcId = scalarValues->getAsDouble(v);
            vec3 position = points[v];
            if (geometricMeanPositionsT.size() < superArcId + 1) {
                geometricMeanPositionsT.resize(superArcId + 1, vec3(0));
                sizeOfAreaT.resize(superArcId + 1, 0);
            }
            geometricMeanPositionsT[superArcId] += position;
            sizeOfAreaT[superArcId]++;
        }

        for (size_t superArcId = 0; superArcId < geometricMeanPositionsT.size(); superArcId++) {
            geometricMeanPositionsT[superArcId] /= sizeOfAreaT[superArcId];
        }
    }

    auto outputMesh = std::make_shared<Mesh>();

    std::vector<vec3> vertices;
    std::vector<vec4> colors;
    std::vector<uint32_t> indicesLines;

    auto tracking = trackingInport_.getData();
    size_t rows = tracking->getNumberOfRows();
    size_t columns = tracking->getNumberOfColumns();

    if (columns != 9) {
        return;
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
    // 7 - Cost (To accumulate)
    std::shared_ptr<const Column> columnCost = tracking->getColumn(7);
    // 8 - Component Id (To count number of tracks)
    std::shared_ptr<const Column> columnComponentId = tracking->getColumn(8);

    std::set<int> componentIds;
    vec3 minPosition = segmentations->at(0)->getPoint(0);
    vec3 maxPosition = segmentations->at(0)->getPoint(numVertices - 1);
    float maxDistance = glm::distance(minPosition, maxPosition);

    // Initialize accumulation variables
    float critDistance = 0.0f;
    float geoMeanDistance = 0.0f;
    // TODO: Other tracking method may have different costs
    float cost = 0.0f;

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
        auto tEnd = std::stoi(columnTimeEnd->getAsString(r));
        if (tEnd < 0 || tEnd >= numTimeSteps) {
            continue;
        }
        auto vertexIdEnd = std::stoi(columnIdEnd->getAsString(r));

        // Insert component Id into the mesh
        componentIds.insert(std::stoi(columnComponentId->getAsString(r)));

        size_t superArcIdStart = segmentations->at(tStart)
                                     ->getScalarValues()
                                     ->getRepresentation<BufferRAM>()
                                     ->getAsDouble(vertexIdStart);
        size_t superArcIdEnd =
            segmentations->at(tEnd)->getScalarValues()->getRepresentation<BufferRAM>()->getAsDouble(
                vertexIdEnd);

        vec3 geometricMeanStart = geometricMeanPositions[tStart][superArcIdStart];
        vec3 geometricMeanEnd = geometricMeanPositions[tEnd][superArcIdEnd];
        vec3 criticalPosStart = segmentations->at(tStart)->getPoints()[vertexIdStart];
        vec3 criticalPosEnd = segmentations->at(tEnd)->getPoints()[vertexIdEnd];

        float critDistanceCurrent = glm::distance(criticalPosStart, geometricMeanEnd) / maxDistance;
        critDistance += critDistanceCurrent;
        float geoMeanDistanceCurrent =
            glm::distance(geometricMeanStart, criticalPosEnd) / maxDistance;
        geoMeanDistance += geoMeanDistanceCurrent;
        float matchingCost = columnCost->getAsDouble(r);
        cost += matchingCost;

        // Add track to the mesh
        if (outputGeometricMeansInMesh_) {
            vertices.push_back(geometricMeanStart);
            vertices.push_back(geometricMeanEnd);
        } else {
            vertices.push_back(criticalPosStart);
            vertices.push_back(criticalPosEnd);
        }

        float distance = 0.0f;

        switch (distanceType_.get()) {
            case 0:
                distance = critDistanceCurrent;
                break;
            case 1:
                distance = geoMeanDistanceCurrent;
                break;
            case 2:
                distance = matchingCost;
                break;
            default:
                break;
        }

        vec4 colorCurrent = tf_.get().sample(distance);
        colors.push_back(colorCurrent);
        colors.push_back(colorCurrent);
        indicesLines.push_back(static_cast<uint32_t>(vertices.size() - 1));
        indicesLines.push_back(static_cast<uint32_t>(vertices.size() - 2));
    }

    // Average accumulated values by number of tracks
    accumulatedCritDistance_.set(critDistance / componentIds.size());
    accumulatedGeoMeanDistance_.set(geoMeanDistance / componentIds.size());
    accumulatedNonOverlapDistance_.set(cost / componentIds.size());

    // Add Buffers to the mesh
    outputMesh->addBuffer(Mesh::BufferInfo(BufferType::PositionAttrib),
                          util::makeBuffer(std::move(vertices)));
    outputMesh->addBuffer(Mesh::BufferInfo(BufferType::ColorAttrib),
                          util::makeBuffer(std::move(colors)));

    if (!indicesLines.empty()) {
        outputMesh->addIndices(Mesh::MeshInfo(DrawType::Lines, ConnectivityType::None),
                               util::makeIndexBuffer(std::move(indicesLines)));
    }

    tracksOutport_.setData(outputMesh);
    timer_.set(performanceTimer_.ElapsedTimeAndReset());
}

}  // namespace inviwo
