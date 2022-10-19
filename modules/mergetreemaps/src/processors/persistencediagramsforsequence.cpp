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

#include <inviwo/mergetreemaps/processors/persistencediagramsforsequence.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo PersistenceDiagramsForSequence::processorInfo_{
    "org.inviwo.PersistenceDiagramForSequence",  // Class identifier
    "Persistence Diagrams For Sequence",         // Display name
    "Topology",                                  // Category
    CodeState::Experimental,                     // Code state
    Tags::None,                                  // Tags
};
const ProcessorInfo PersistenceDiagramsForSequence::getProcessorInfo() const {
    return processorInfo_;
}

PersistenceDiagramsForSequence::PersistenceDiagramsForSequence()
    : PoolProcessor()
    , scalarsInport_("scalars")
    , outport_("diagrams")
    , computeSaddleConnectors_{"computeSaddleConnectors", "Compute Saddle Connectors", false}
    , timer_("timer", "Eval Time (s)", 0.f, 0.f, std::numeric_limits<float>::max(), 0.001f,
             InvalidationLevel::Valid, PropertySemantics::Text) {

    addPort(scalarsInport_);
    addPort(outport_);

    addProperties(computeSaddleConnectors_);

    addProperty(timer_);
    timer_.setReadOnly(true);
}

namespace {

struct ComputePersistanceDiagram {
    template <typename Result, typename Format>
    Result operator()(pool::Progress progress, bool computeSaddleConnectors,
                      const topology::TriangulationData triangulation) {

        using dataType = Format::type;

        using DiagramOutput =
            std::vector<std::tuple<ttk::SimplexId, ttk::CriticalType, ttk::SimplexId,
                                   ttk::CriticalType, dataType, ttk::SimplexId>>;

        std::vector<int> offsets(triangulation.getOffsets());
        DiagramOutput output;
        ttk::PersistenceDiagram diagram;

        diagram.setComputeSaddleConnectors(computeSaddleConnectors);
        diagram.setOutputCTDiagram(&output);
        diagram.setupTriangulation(
            const_cast<ttk::Triangulation*>(&triangulation.getTriangulation()));
        diagram.setInputScalars(
            triangulation.getScalarValues()->getRepresentation<BufferRAM>().getData());
        diagram.setInputOffsets(offsets.data());

        int retVal = diagram.execute<dataType, int>();
        if (retVal != 0) {
            throw TTKException("Error computing ttk::PersistenceDiagram");
        }

        // convert diagram output to topology::PersistenceDiagramData, i.e. float
        auto diagramOutput = std::make_shared<topology::PersistenceDiagramData>();
        diagramOutput->reserve(output.size());
        for (auto& elem : output) {
            diagramOutput->emplace_back(std::make_tuple(
                std::get<0>(elem), std::get<1>(elem), std::get<2>(elem), std::get<3>(elem),
                static_cast<float>(std::get<4>(elem)), std::get<5>(elem)));
        }

        return diagramOutput;
    }
};
}  // namespace

void PersistenceDiagramsForSequence::process() {

    performanceTimer_.Reset();

    // Save input and properties needed to calculate ttk contour tree to local variables
    const auto scalarData = scalarsInport_.getData();
    const bool computeSaddleConnectors = computeSaddleConnectors_.get();

    using Result = std::shared_ptr<topology::PersistenceDiagramData>;

    const auto computeDiagramJob = [](const bool computeSaddleConnectors,
                                      const topology::TriangulationData triangulation) {
        return [computeSaddleConnectors, triangulation](pool::Stop,
                                                        pool::Progress progress) -> Result {
            // Lambda for diagram computation based on scalar value buffer
            auto computePersistanceDiagram = [computeSaddleConnectors, triangulation,
                                              progress](const auto buffer) -> Result {
                using ValueType = util::PrecisionValueType<decltype(buffer)>;
                using DiagramOutput =
                    std::vector<std::tuple<ttk::SimplexId, ttk::CriticalType, ttk::SimplexId,
                                           ttk::CriticalType, ValueType, ttk::SimplexId>>;

                std::vector<int> offsets(triangulation.getOffsets());
                DiagramOutput output;
                ttk::PersistenceDiagram diagram;

                diagram.setComputeSaddleConnectors(computeSaddleConnectors);
                diagram.setOutputCTDiagram(&output);
                diagram.setupTriangulation(
                    const_cast<ttk::Triangulation*>(&triangulation.getTriangulation()));
                diagram.setInputScalars(buffer->getDataContainer().data());
                diagram.setInputOffsets(offsets.data());

                int retVal = diagram.execute<typename DataFormat<ValueType>::primitive, int>();
                if (retVal != 0) {
                    throw TTKException("Error computing ttk::PersistenceDiagram");
                }

                // convert diagram output to topology::PersistenceDiagramData, i.e. float
                auto diagramOutput = std::make_shared<topology::PersistenceDiagramData>();
                diagramOutput->reserve(output.size());
                for (auto& elem : output) {
                    diagramOutput->emplace_back(std::make_tuple(
                        std::get<0>(elem), std::get<1>(elem), std::get<2>(elem), std::get<3>(elem),
                        static_cast<float>(std::get<4>(elem)), std::get<5>(elem)));
                }

                return diagramOutput;
            };

            return triangulation.getScalarValues()
                ->getEditableRepresentation<BufferRAM>()
                ->dispatch<Result, dispatching::filter::Scalars>(computePersistanceDiagram);
        };
    };

    std::vector<std::function<Result(pool::Stop, pool::Progress progress)>> jobs;
    for (size_t i = 0; i < scalarData.get()->size(); i++) {
        jobs.push_back(computeDiagramJob(computeSaddleConnectors, *scalarData.get()->at(i)));
    }

    outport_.setData(nullptr);
    dispatchMany(jobs, [this](std::vector<Result> result) {
        diagrams_ = result;
        outport_.setData(std::make_shared<PersistenceDiagramSequence>(diagrams_));
        newResults();
        timer_.set(performanceTimer_.ElapsedTimeAndReset());
    });
}

}  // namespace inviwo
