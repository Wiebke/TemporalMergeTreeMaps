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

#include <inviwo/mergetreemaps/processors/contourtreesforsequence.h>

#include <warn/push>
#include <warn/ignore/all>
#include <ttk/core/base/topologicalSimplification/TopologicalSimplification.h>
#include <ttk/core/base/ftmTree/FTMTree_MT.h>
#include <warn/pop>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo ContourTreesForSequence::processorInfo_{
    "org.inviwo.ContourTreesForSequence",  // Class identifier
    "Contour Trees For Sequence",          // Display name
    "Topology",                            // Category
    CodeState::Experimental,               // Code state
    Tags::None,                            // Tags
};
const ProcessorInfo ContourTreesForSequence::getProcessorInfo() const { return processorInfo_; }

ContourTreesForSequence::ContourTreesForSequence()
    : PoolProcessor()
    , scalarsInport_("scalars")
    , outport_("contourtrees")
    , treeType_("treeType", "Tree Type",
                {
                    {"join", "Join Tree (Minima Leaves)", topology::TreeType::Join},
                    {"split", "Split Tree (Maxima Leaves)", topology::TreeType::Split},
                    {"contour", "Contour Tree", topology::TreeType::Contour}
                    //{"joinAndSplit", "Join and Split", topology::TreeType::JoinAndSplit}},
                    // FIXME: for some reason TTK does not support this properly.
                    // The resulting tree has no data
                },
                2)
    , threadCount_("threadCount", "Number of Threads", 1, 1, 100)
    , segmentation_("segmentation", "Segmentation", true)
    , normalization_("normalization", "Normalization", false)
    , timer_("timer", "Eval Time (s)", 0.f, 0.f, std::numeric_limits<float>::max(), 0.001f,
             InvalidationLevel::Valid, PropertySemantics::Text) {

    addPort(scalarsInport_);
    addPort(outport_);

    addProperty(treeType_);
    addProperty(threadCount_);
    addProperty(segmentation_);
    addProperty(normalization_);

    addProperty(timer_);
    timer_.setReadOnly(true);
}

void ContourTreesForSequence::process() {

    performanceTimer_.Reset();

    const auto scalarData = scalarsInport_.getData();

    const auto threadCount = threadCount_.get();
    const auto treeType = treeType_.get();
    const auto segmentation = segmentation_.get();
    const auto normalization = normalization_.get();

    using Result = std::shared_ptr<topology::ContourTreeData>;

    const auto computeTreeJob = [](const int threadCount, const topology::TreeType treeType,
                                   const bool segmentation, const bool normalization,
                                   const topology::TriangulationData triangulation) {
        return [triangulation, threadCount, treeType, segmentation, normalization](
                   pool::Stop, pool::Progress progress) -> Result {
            // Lamda function to compute contour tree based on scalar value buffer
            auto computeTree = [triangulation, threadCount, treeType, segmentation,
                                normalization](const auto buffer) {
                using ValueType = util::PrecisionValueType<decltype(buffer)>;
                using PrimitiveType = typename DataFormat<ValueType>::primitive;

                std::vector<int> offsets(triangulation.getOffsets());

                auto treeData = std::make_shared<topology::ContourTreeData>();
                auto tree = std::make_shared<ttk::ftm::FTMTree>();
                treeData->tree = tree;
                treeData->type = treeType;

                tree->setThreadNumber(threadCount);
                tree->setupTriangulation(
                    const_cast<ttk::Triangulation*>(&triangulation.getTriangulation()));
                // tree->setDebugLevel(0);
                tree->setVertexScalars(buffer->getDataContainer().data());
                tree->setVertexSoSoffsets(offsets.data());
                tree->setTreeType(static_cast<int>(treeType));
                tree->setSegmentation(segmentation);
                tree->setNormalizeIds(normalization);

                tree->build<PrimitiveType, ttk::SimplexId>();
                return treeData;
            };

            return triangulation.getScalarValues()
                ->getRepresentation<BufferRAM>()
                ->dispatch<Result, dispatching::filter::Scalars>(computeTree);
        };
    };

    std::vector<std::function<Result(pool::Stop, pool::Progress progress)>> jobs;
    for (size_t i = 0; i < scalarData.get()->size(); i++) {
        jobs.push_back(computeTreeJob(threadCount, treeType, segmentation, normalization,
                                      *scalarData.get()->at(i).get()));
    }

    outport_.setData(nullptr);
    dispatchMany(jobs, [this](std::vector<Result> result) {
        trees_ = result;
        // Add triangulation pointer to each contour tree
        for (size_t i = 0; i < result.size(); i++) {
            auto triangulation = scalarsInport_.getData()->at(i);
            result[i]->triangulation = triangulation;
        }
        outport_.setData(std::make_shared<ContourTreeSequence>(trees_));
        newResults();
        timer_.set(performanceTimer_.ElapsedTimeAndReset());
    });
}

}  // namespace inviwo
