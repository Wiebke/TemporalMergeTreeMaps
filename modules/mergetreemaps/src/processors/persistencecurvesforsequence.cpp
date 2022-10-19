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

#include <inviwo/mergetreemaps/processors/persistencecurvesforsequence.h>

#include <warn/push>
#include <warn/ignore/all>
#include <ttk/core/base/persistenceCurve/PersistenceCurve.h>
#include <warn/pop>

#include <tuple>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo PersistenceCurvesForSequence::processorInfo_{
    "org.inviwo.PersistenceCurvesForSequence",  // Class identifier
    "Persistence Curves For Sequence",          // Display name
    "Topology",                                 // Category
    CodeState::Experimental,                    // Code state
    Tags::None,                                 // Tags
};
const ProcessorInfo PersistenceCurvesForSequence::getProcessorInfo() const {
    return processorInfo_;
}

PersistenceCurvesForSequence::PersistenceCurvesForSequence()
    : PoolProcessor()
    , scalarsInport_("scalars")
    , outport_("diagrams")
    , timer_("timer", "Eval Time (s)", 0.f, 0.f, std::numeric_limits<float>::max(), 0.001f,
             InvalidationLevel::Valid, PropertySemantics::Text) {

    addPort(scalarsInport_);
    addPort(outport_);

    addProperty(timer_);
    timer_.setReadOnly(true);
}

void PersistenceCurvesForSequence::process() {
    performanceTimer_.Reset();

    // Save input and properties needed to calculate ttk contour tree to local variables
    const auto scalarData = scalarsInport_.getData();

    using Result = std::shared_ptr<DataFrame>;

    const auto computeCurveJob = [](const topology::TriangulationData& triangulation) {
        return [triangulation](pool::Stop, pool::Progress progress) -> Result {
            // Lambda for diagram computation based on scalar value buffer
            auto computePersistanceCurve = [triangulation, progress](const auto buffer) -> Result {
                using ValueType = util::PrecisionValueType<decltype(buffer)>;
                using PrimitiveType = typename DataFormat<ValueType>::primitive;

                std::vector<int> offsets(triangulation.getOffsets());

                // Computing the persistence curve
                ttk::PersistenceCurve curve;
                std::vector<std::pair<PrimitiveType, ttk::SimplexId>> outputCurve;
                curve.setupTriangulation(
                    const_cast<ttk::Triangulation*>(&triangulation.getTriangulation()));
                curve.setInputScalars(buffer->getDataContainer().data());
                curve.setInputOffsets(offsets.data());
                curve.setOutputCTPlot(&outputCurve);

                int retVal = curve.execute<PrimitiveType, int>();
                if (retVal < 0) {
                    throw TTKException("Error computing ttk::PersistenceCurve");
                }

                // convert result of ttk::PersistenceCurve into a DataFrame
                auto dataFrame = std::make_shared<DataFrame>();

                std::vector<PrimitiveType> persistence;
                std::vector<unsigned int> count;
                persistence.reserve(outputCurve.size());
                count.reserve(outputCurve.size());
                for (const auto& p : outputCurve) {
                    persistence.emplace_back(p.first);
                    count.emplace_back(static_cast<unsigned int>(p.second));
                }

                dataFrame->addColumnFromBuffer(
                    "Persistence", util::makeBuffer<PrimitiveType>(std::move(persistence)));
                dataFrame->addColumnFromBuffer("Number of CP Pairs",
                                               util::makeBuffer<unsigned int>(std::move(count)));
                dataFrame->updateIndexBuffer();

                return dataFrame;
            };

            return triangulation.getScalarValues()
                ->getEditableRepresentation<BufferRAM>()
                ->dispatch<Result, dispatching::filter::Scalars>(computePersistanceCurve);
        };
    };

    std::vector<std::function<Result(pool::Stop, pool::Progress progress)>> jobs;
    for (size_t i = 0; i < scalarData.get()->size(); i++) {
        jobs.push_back(computeCurveJob(*scalarData.get()->at(i)));
    }

    outport_.setData(nullptr);
    dispatchMany(jobs, [this](std::vector<Result> result) {
        curves_ = result;
        outport_.setData(std::make_shared<DataFrameSequence>(curves_));
        newResults();
        timer_.set(performanceTimer_.ElapsedTimeAndReset());
    });
}

}  // namespace inviwo
