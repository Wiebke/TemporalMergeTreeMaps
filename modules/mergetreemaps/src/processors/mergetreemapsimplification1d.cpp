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

#include <inviwo/mergetreemaps/processors/mergetreemapsimplification1d.h>
#include <modules/base/algorithm/dataminmax.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo MergeTreeMapSimplification1D::processorInfo_{
    "org.inviwo.MergeTreeMapSimplification1D",  // Class identifier
    "Merge Tree Map Simplification 1D",         // Display name
    "Merge Tree Maps",                          // Category
    CodeState::Experimental,                    // Code state
    Tags::None,                                 // Tags
};
const ProcessorInfo MergeTreeMapSimplification1D::getProcessorInfo() const {
    return processorInfo_;
}

MergeTreeMapSimplification1D::MergeTreeMapSimplification1D()
    : Processor()
    , positionColumn_("position", "Position Columns", inport_, false, 1)
    , scalarColumn_("color", "Color Columns", inport_, false, 2)
    , epsilon_("eps", "Epsilon", 0.0001f, 0.0f, 10.0f, 1e-10f, InvalidationLevel::InvalidOutput,
               PropertySemantics::Text) {

    addPort(inport_);
    addPort(outport_);

    addProperties(positionColumn_, scalarColumn_, epsilon_);
}

float MergeTreeMapSimplification1D::distanceToLine(const vec3& point, const vec3& lineP1,
                                                   const vec3& lineP2) {

    float lineLength = glm::distance(vec2{lineP1.x, lineP1.y}, vec2{lineP2.x, lineP2.y});
    mat3 cords;
    cords[0] = vec3{point.x, point.y, 1};
    cords[1] = vec3{lineP1.x, lineP1.y, 1};
    cords[2] = vec3{lineP2.x, lineP2.y, 1};
    return std::abs(glm::determinant(cords)) / lineLength;
}

std::vector<vec3> MergeTreeMapSimplification1D::simplify(const std::vector<vec3>& points,
                                                         float epsilon) {
    // Find the point with the maximum distance
    float maxDistance = 0;
    size_t maxIndex = 0;
    size_t startIndex = 0;
    const vec3& startPoint = points[startIndex];
    size_t endIndex = points.size() - 1;
    const vec3& endPoint = points[endIndex];

    // Find point that is furthest away
    for (size_t idx = startIndex + 1; idx < endIndex; idx++) {
        float dist = distanceToLine(points[idx], startPoint, endPoint);
        if (dist > maxDistance) {
            maxIndex = idx;
            maxDistance = dist;
        }
    }
    std::vector<vec3> simplifiedPoints;

    if (maxDistance > epsilon) {
		// Vector init with [first, last) -> need to add + 1 to last index
        const std::vector<vec3> firstHalfPoints(points.cbegin() + startIndex,
                                                points.cbegin() + maxIndex + 1);
        auto firstHalfSimplified = simplify(firstHalfPoints, epsilon);
        const std::vector<vec3> secondHalfPoints(points.cbegin() + maxIndex,
                                                 points.cbegin() + endIndex + 1);
        auto secondHalfSimplified = simplify(secondHalfPoints, epsilon);

        // Will have at least two points, end()-1 is fine (used to not duplicate maxIndex point
        simplifiedPoints.insert(simplifiedPoints.end(), firstHalfSimplified.begin(),
                                firstHalfSimplified.end() - 1);
        simplifiedPoints.insert(simplifiedPoints.end(), secondHalfSimplified.begin(),
                                secondHalfSimplified.end());
    } else {
        simplifiedPoints.push_back(startPoint);
        simplifiedPoints.push_back(endPoint);
    }

    return simplifiedPoints;
}

void MergeTreeMapSimplification1D::process() {

    // Place position and scalar column into vector of vec3 with (x,y,original Index)
    auto df = inport_.getData();
    size_t numRows = df->getNumberOfRows();
    if (numRows < 2) return;
    std::vector<vec3> points(numRows);
    auto posIdx = positionColumn_.get();
    auto posColumn = df->getColumn(posIdx)->getBuffer()->getRepresentation<BufferRAM>();
    auto scalarIdx = scalarColumn_.get();
    auto scalarColumn = df->getColumn(scalarIdx)->getBuffer()->getRepresentation<BufferRAM>();

    auto range = util::bufferMinMax(scalarColumn);

    for (size_t idx = 0; idx < numRows; idx++) {
        double pos = posColumn->getAsDouble(idx);
        double value = scalarColumn->getAsDouble(idx);
        // normalize x to range (-> function covers square)
        // points[idx] = vec3{pos / numRows * (range.second.x - range.first.x), value,
        // static_cast<float>(idx)};
        points[idx] = vec3{pos, value, static_cast<float>(idx)};
    }

    // Sort vector according to x
    std::sort(points.begin(), points.end(), [](const vec3& a, const vec3& b) { return a.x < b.x; });

    // Simplify recursivly
    std::vector<vec3> simplified = simplify(points, epsilon_.get());

    // Reassemble into dataframe
    std::vector<int> positions;
    positions.reserve(simplified.size());
    std::vector<float> scalarValues;
    scalarValues.reserve(simplified.size());

    size_t idx = 0;
    for (const auto& value : simplified) {
        // revert normalization: / numRows * (range.second.x - range.first.x)
        // positions.push_back(
        //    static_cast<int>(std::round(value.x * numRows / (range.second.x - range.first.x))));
        positions.push_back(value.x);
        scalarValues.push_back(value.y);
        idx++;
    }

    auto resultDf = std::make_shared<DataFrame>();
    resultDf->addColumnFromBuffer(df->getHeader(posIdx),
                                  util::makeBuffer<int>(std::move(positions)));
    resultDf->addColumnFromBuffer(df->getHeader(scalarIdx),
                                  util::makeBuffer<float>(std::move(scalarValues)));
    resultDf->updateIndexBuffer();

    outport_.setData(resultDf);
}

}  // namespace inviwo
