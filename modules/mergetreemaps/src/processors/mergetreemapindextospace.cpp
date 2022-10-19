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

#include <inviwo/mergetreemaps/processors/mergetreemapindextospace.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo MergeTreeMapIndexToSpace::processorInfo_{
    "org.inviwo.MergeTreeMapIndexToSpace",  // Class identifier
    "Merge Tree Map Index To Space",        // Display name
    "Undefined",                            // Category
    CodeState::Experimental,                // Code state
    Tags::None,                             // Tags
};
const ProcessorInfo MergeTreeMapIndexToSpace::getProcessorInfo() const { return processorInfo_; }

MergeTreeMapIndexToSpace::MergeTreeMapIndexToSpace()
    : PoolProcessor()
    , vertexIdColumn_("vertexId", "Vertex Id Column", mtmInport_, false, 0)
    , positionColumn_("position", "Position Column", mtmInport_, false, 1) {

    addPort(inport_);
    addPort(mtmInport_);
    addPort(outport_);

    addProperties(vertexIdColumn_, positionColumn_);
}

void MergeTreeMapIndexToSpace::process() {
    const auto inportScalars = inport_.getData();
    auto df = mtmInport_.getData();
    size_t numRows = df->getNumberOfRows();
    if (inportScalars->getTriangulation().getNumberOfVertices() != numRows) {
        LogProcessorError("");
        return;
    }
    auto vertexIdx = vertexIdColumn_.get();
    auto vertexIdColumn = df->getColumn(vertexIdx)->getBuffer()->getRepresentation<BufferRAM>();
    auto posIdx = positionColumn_.get();
    auto posColumn = df->getColumn(posIdx)->getBuffer()->getRepresentation<BufferRAM>();

    using Result = std::shared_ptr<topology::TriangulationData>;

    auto compute = [inportScalars, vertexIdColumn, posColumn](pool::Stop stop,
                                                              pool::Progress progress) -> Result {
        return inportScalars->getScalarValues()
            ->getRepresentation<BufferRAM>()
            ->dispatch<std::shared_ptr<topology::TriangulationData>, dispatching::filter::Scalars>(
                [inportScalars, vertexIdColumn, posColumn, stop,
                 progress](const auto buffer) -> Result {
                    if (stop) return nullptr;
                    using ValueType = util::PrecisionValueType<decltype(buffer)>;

                    progress(0.1f);

                    auto numVertices = buffer->getDataContainer().size();
					// Initialize with -1 for we can see vertex ids not set
                    std::vector<float> segmentationValues(numVertices, -1.0f);
                    std::vector<int> offsets(inportScalars->getOffsets());

                    // We already checked before that the dataframe columns have the same size as
                    // the triangultion

                    for (size_t r = 0; r < numVertices; r++) {
                        float pos = posColumn->getAsDouble(r);
                        int vertexId = vertexIdColumn->getAsDouble(r);
                        if (vertexId >= 0 && vertexId < numVertices) {
                            segmentationValues[vertexId] = pos;
                        }
                    }

                    // create a new triangulation based on the old one, but with segmentation as
                    // scalar values
                    auto result = std::make_shared<topology::TriangulationData>(*inportScalars);
                    result->setScalarValues(util::makeBuffer(std::move(segmentationValues)));
                    result->setOffsets(std::move(offsets));
                    if (result->isUniformGrid()) {
                        auto datamapper = result->getDataMapper();
                        datamapper.dataRange = vec2(0, numVertices - 1);
                        datamapper.valueRange = vec2(0, numVertices - 1);
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
