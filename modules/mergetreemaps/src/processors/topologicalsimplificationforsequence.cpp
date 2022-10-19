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

#include <inviwo/mergetreemaps/processors/topologicalsimplificationforsequence.h>

#include <warn/push>
#include <warn/ignore/all>
#include <ttk/core/base/topologicalSimplification/TopologicalSimplification.h>
#include <warn/pop>
#include <inviwo/core/util/formats.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo TopologicalSimplificationForSequence::processorInfo_{
    "org.inviwo.TopologicalSimplificationForSequence",  // Class identifier
    "Topological Simplification For Sequence",          // Display name
    "Topology",                                         // Category
    CodeState::Experimental,                            // Code state
    Tags::None,                                         // Tags
};
const ProcessorInfo TopologicalSimplificationForSequence::getProcessorInfo() const {
    return processorInfo_;
}

TopologicalSimplificationForSequence::TopologicalSimplificationForSequence()
    : PoolProcessor()
    , scalarsInport_("scalarsIn")
    , pdInport_("persistence")
    , scalarsOutport_("scalarsOut")
    , mode_("mode", "Critical Points Removal Mode",
            {
                {"removeBelow", "Below Threshold", 0},
                {"removeAbove", "Above Threhold", 1},
                {"keepMost", "Keep N Most Persistent", 2},
                {"keepLeast", "Keep N Least Persistent", 3},
            },
            0)
    , threshold_("threshold", "Threshold", 0.0f, 0.0f, 1000.0f)
    , numKeepPoints_("numKeepPoints", "N", 30, 2, 10000)
    , timer_("timer", "Eval Time (s)", 0.f, 0.f, std::numeric_limits<float>::max(), 0.001f,
             InvalidationLevel::Valid, PropertySemantics::Text) {

    addPort(scalarsInport_);
    addPort(pdInport_);
    addPort(scalarsOutport_);

    addProperty(mode_);
    addProperty(threshold_);
    addProperty(numKeepPoints_);

    threshold_.visibilityDependsOn(mode_, [](const auto& p) { return p.get() <= 1; });
    numKeepPoints_.visibilityDependsOn(mode_, [](const auto& p) { return p.get() > 1; });

    pdInport_.onChange([this]() {
        if (pdInport_.hasData()) {
            // Adjust max value to highest persistence value (will always be positive)
            float currentMax = -1.0;
            for (auto pd : *pdInport_.getData()) {
                auto endIt = std::end(*pd);
                auto maxIt = std::max_element(
                    std::begin(*pd), endIt,
                    [](const auto& a, const auto& b) { return std::get<4>(a) < std::get<4>(b); });
                if (maxIt != endIt && std::get<4>(*maxIt) > currentMax) {
                    currentMax = std::get<4>(*maxIt);
                }
            }
            threshold_.setMaxValue(currentMax);
        }
    });

    addProperty(timer_);
    timer_.setReadOnly(true);
}

void TopologicalSimplificationForSequence::process() {

    performanceTimer_.Reset();

    // Save input and properties needed to calculate simplification on local variables
    const auto scalarData = scalarsInport_.getData();
    const auto persistenceDiagrams = pdInport_.getData();
    const auto threshold = threshold_.get();
    const auto mode = mode_.get();

    using Result = std::shared_ptr<topology::TriangulationData>;

    const auto computeSimplificationJob = [](const size_t mode, const float threshold,
                                             const size_t numKeepPoints,
                                             const topology::TriangulationData triangulation,
                                             const topology::PersistenceDiagramData diagramData) {
        return [mode, threshold, numKeepPoints, triangulation, diagramData](
                   pool::Stop stop, pool::Progress progress) -> Result {
            // Lambda for topological simplification based on scalar value buffer
            auto simplify = [mode, threshold, numKeepPoints, triangulation, diagramData, stop,
                             progress](const auto buffer) -> Result {
                if (stop) return nullptr;
                using ValueType = util::PrecisionValueType<decltype(buffer)>;

                progress(0.1f);

                float currentMax = -1.0;
                size_t maxIndex = 0;
                float currentMin = std::numeric_limits<float>::max();
                size_t minIndex = 0;

                // select most/least persistent critical point pairs according to either threshold
                // or number of desired critical points
                std::vector<int> authorizedCriticalPoints;

                if (mode <= 1) {
                    // select most/least persistent critical point pairs
                    bool invert = (mode != 0);

                    for (size_t i = 0; i < diagramData.size(); i++) {
                        auto persistence = std::get<4>(diagramData[i]);
                        if ((persistence >= threshold) != invert) {
                            authorizedCriticalPoints.push_back(std::get<0>(diagramData[i]));
                            authorizedCriticalPoints.push_back(std::get<2>(diagramData[i]));
                        }
                        if (persistence > currentMax) {
                            currentMax = persistence;
                            maxIndex = i;
                        }
                        if (persistence < currentMin) {
                            currentMin = persistence;
                            minIndex = i;
                        }
                    }

                    // Keep at least one pair of points
                    if (authorizedCriticalPoints.empty()) {
                        if (mode == 0) {
                            // Keep most persistant pair
                            authorizedCriticalPoints.push_back(std::get<0>(diagramData[maxIndex]));
                            authorizedCriticalPoints.push_back(std::get<2>(diagramData[maxIndex]));
                        } else {
                            // Keep least persistant pair
                            authorizedCriticalPoints.push_back(std::get<0>(diagramData[minIndex]));
                            authorizedCriticalPoints.push_back(std::get<2>(diagramData[minIndex]));
                        }
                    }
                } else {
                    // Sort diagram indices according to persistance
                    std::vector<size_t> pairIndices(diagramData.size());
                    std::iota(std::begin(pairIndices), std::end(pairIndices), 0);
                    std::sort(pairIndices.begin(), pairIndices.end(),
                              [&diagramData](const size_t& a, const size_t& b) -> bool {
                                  return std::get<4>(diagramData[a]) < std::get<4>(diagramData[b]);
                              });
                    size_t numPairs = diagramData.size();
                    // We can keep at most the number of existing pairs
                    size_t maxNumPairs = std::min(numPairs, numKeepPoints / 2);
                    size_t pairIndex;
                    for (size_t i = 0; i < maxNumPairs; i++) {
                        if (mode == 2) {
                            // Keep most persistant pairs (will be at the end of sorted pairIndices)
                            pairIndex = pairIndices[numPairs - 1 - i];

                        } else {
                            // Keep least persistant pairs (will be at the beginning or sorted
                            // pairIndices)
                            pairIndex = pairIndices[i];
                        }
                        authorizedCriticalPoints.push_back(std::get<0>(diagramData[pairIndex]));
                        authorizedCriticalPoints.push_back(std::get<2>(diagramData[pairIndex]));
                    }
                }

                if (stop) return nullptr;

                progress(0.2f);

                // create a copy of the data values, nth component will be overwritten by
                // simplification
                auto simplifiedDataValues = buffer->getDataContainer();

                std::vector<int> offsets(triangulation.getOffsets());
                if (!authorizedCriticalPoints.empty()) {
                    // perform topological simplification
                    ttk::TopologicalSimplification simplification;
                    simplification.setupTriangulation(
                        const_cast<ttk::Triangulation*>(&triangulation.getTriangulation()));
                    simplification.setInputScalarFieldPointer(buffer->getDataContainer().data());
                    simplification.setInputOffsetScalarFieldPointer(offsets.data());
                    simplification.setOutputScalarFieldPointer(simplifiedDataValues.data());
                    simplification.setOutputOffsetScalarFieldPointer(offsets.data());
                    simplification.setConstraintNumber(
                        static_cast<int>(authorizedCriticalPoints.size()));
                    simplification.setVertexIdentifierScalarFieldPointer(
                        authorizedCriticalPoints.data());

                    int retVal =
                        simplification.execute<typename DataFormat<ValueType>::primitive, int>();
                    if (retVal < 0) {
                        throw TTKException("Error computing ttk::TopologicalSimplification",
                                           IVW_CONTEXT_CUSTOM("TopologicalSimplification"));
                    }
                }

                progress(0.8f);

                // create a new triangulation based on the old one, but with new scalar values
                auto result = std::make_shared<topology::TriangulationData>(triangulation);
                result->setScalarValues(util::makeBuffer(std::move(simplifiedDataValues)));
                result->setOffsets(std::move(offsets));

                progress(0.99f);

                return result;
            };

            return triangulation.getScalarValues()
                ->getEditableRepresentation<BufferRAM>()
                ->dispatch<Result, dispatching::filter::Scalars>(simplify);
        };
    };

    std::vector<std::function<Result(pool::Stop, pool::Progress progress)>> jobs;
    for (size_t i = 0; i < scalarData.get()->size(); i++) {
        jobs.push_back(computeSimplificationJob(mode_.get(), threshold_.get(), numKeepPoints_.get(),
                                                *scalarData.get()->at(i),
                                                *persistenceDiagrams.get()->at(i)));
    }

    scalarsOutport_.setData(nullptr);
    dispatchMany(jobs, [this](std::vector<Result> result) {
        simplified_ = result;
        scalarsOutport_.setData(std::make_shared<TriangulationSequence>(simplified_));
        newResults();
        timer_.set(performanceTimer_.ElapsedTimeAndReset());
    });
}

}  // namespace inviwo
