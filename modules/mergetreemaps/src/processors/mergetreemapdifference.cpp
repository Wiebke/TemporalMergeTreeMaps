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

#include <inviwo/mergetreemaps/processors/mergetreemapdifference.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo MergeTreeMapDifference::processorInfo_{
    "org.inviwo.MergeTreeMapDifference",  // Class identifier
    "Merge Tree Map Difference",          // Display name
    "Merge Tree Maps",                    // Category
    CodeState::Experimental,              // Code state
    Tags::None,                           // Tags
};
const ProcessorInfo MergeTreeMapDifference::getProcessorInfo() const { return processorInfo_; }

MergeTreeMapDifference::MergeTreeMapDifference()
    : Processor()
    , positionColumn_("position", "Position Columns", dataFramePort_, false, 1)
    , valueColumn_("value", "Value Columns", dataFramePort_, false, 2)
    , outputMode_("outputMode", "Output Mode")
    , timer_("timer", "Eval Time (s)", 0.f, 0.f, std::numeric_limits<float>::max(), 0.001f,
             InvalidationLevel::Valid, PropertySemantics::Text) {

    addPort(inport_);
    addPort(dataFramePort_);
    addPort(secondInport_);

    addPort(outport_);

    addProperties(positionColumn_, valueColumn_, outputMode_, timer_);

    outputMode_.addOption("diff", "Difference", 0);
    outputMode_.addOption("diffSquared", "Difference^2", 1);
    outputMode_.addOption("diffRmse", "sqrt(Difference^2)", 2);
    outputMode_.setSelectedIndex(0);
    outputMode_.setCurrentStateAsDefault();
}

void MergeTreeMapDifference::process() {
    performanceTimer_.Reset();
    const auto dfFrames = inport_.getData();
    const auto dfFramesSecond = secondInport_.getData();
    const auto modelFrame = dataFramePort_.getData();
    if (!dfFrames || !modelFrame || !dfFramesSecond) return;
    size_t numTimeSteps = dfFrames->size();
    if (numTimeSteps < 1) return;
    if (dfFramesSecond->size() != numTimeSteps) {
        LogProcessorError("Number of timesteps does not match in input dataframes.");
        return;
    }

    size_t dataDims = dfFrames->at(0)->getNumberOfRows();
    std::vector<double> valuesInFirst(dataDims);
    std::vector<double> valuesInSecond(dataDims);
    auto posIdx = positionColumn_.get();
    auto valueIdx = valueColumn_.get();

    auto outputDfs = std::make_shared<DataFrameSequence>(DataFrameSequence(numTimeSteps));

    for (size_t t = 0; t < numTimeSteps; t++) {
        auto currentDfFirst = dfFrames->at(t);
        if (currentDfFirst->getNumberOfRows() != dataDims) {
            LogProcessorError("First dataframe at time t=" << t << "has incorrect size.");
            return;
        }
        auto posColumnFirst =
            currentDfFirst->getColumn(posIdx)->getBuffer()->getRepresentation<BufferRAM>();
        auto valueColumnFirst =
            currentDfFirst->getColumn(valueIdx)->getBuffer()->getRepresentation<BufferRAM>();
        auto currentDfSecond = dfFramesSecond->at(t);
        if (currentDfSecond->getNumberOfRows() != dataDims) {
            LogProcessorError("Second dataframe at time t=" << t << "has incorrect size.");
            return;
        }
        auto posColumnSecond =
            currentDfSecond->getColumn(posIdx)->getBuffer()->getRepresentation<BufferRAM>();
        auto valueColumnSecond =
            currentDfSecond->getColumn(valueIdx)->getBuffer()->getRepresentation<BufferRAM>();
        // Check that sizes and column names match in both dataframes
        if (currentDfFirst->getHeader(valueIdx) != currentDfSecond->getHeader(valueIdx)) {
            LogProcessorError("Headers for position column do not match: "
                              << currentDfFirst->getHeader(posIdx)
                              << "!= " << currentDfSecond->getHeader(posIdx) << ".");
            return;
        }
        if (currentDfFirst->getHeader(valueIdx) != currentDfSecond->getHeader(valueIdx)) {
            LogProcessorError("Headers for value column do not match: "
                              << currentDfFirst->getHeader(valueIdx)
                              << "!= " << currentDfSecond->getHeader(valueIdx) << ".");
            return;
        }

        // Note: positions do not need to be sorted, thus we need to build a map first before
        // computing the difference
        for (size_t dIndex = 0; dIndex < dataDims; dIndex++) {
            int posFirst = static_cast<int>(posColumnFirst->getAsDouble(dIndex));
            double valueFirst = valueColumnFirst->getAsDouble(dIndex);
            if (posFirst < 0 || posFirst >= dataDims) {
                LogProcessorError(
                    "Positions in position columns are assumed to be integers between 0 and "
                    << dataDims << "but value " << posFirst << "was encountered.");
                return;
            }
            valuesInFirst[posFirst] = valueFirst;
            int posSecond = static_cast<int>(posColumnSecond->getAsDouble(dIndex));
            double valueSecond = valueColumnSecond->getAsDouble(dIndex);
            if (posSecond < 0 || posSecond >= dataDims) {
                LogProcessorError(
                    "Positions in position columns are assumed to be integers between 0 and "
                    << dataDims << "but value " << posSecond << "was encountered.");
                return;
            }
            valuesInSecond[posSecond] = valueSecond;
        }

        std::vector<float> valueDifference(dataDims);
        for (size_t dIndex = 0; dIndex < dataDims; dIndex++) {
            double diff = valuesInFirst[dIndex] - valuesInSecond[dIndex];

            switch (outputMode_.get()) {
                case 0:
                    valueDifference[dIndex] = diff;
                    break;
                case 1:
                    valueDifference[dIndex] = diff * diff;
                    break;
                case 2:
                    valueDifference[dIndex] = sqrt(diff * diff);
                    break;
                default:
                    valueDifference[dIndex] = diff;
                    break;
            }
        }
        outputDfs->at(t) = std::make_shared<DataFrame>();
        auto &dataframe = outputDfs->at(t);
        dataframe->addColumnFromBuffer("Scalar Difference",
                                       util::makeBuffer<float>(std::move(valueDifference)));
        dataframe->updateIndexBuffer();
    }

    timer_.set(performanceTimer_.ElapsedTimeAndReset());
    outport_.setData(outputDfs);
}

}  // namespace inviwo
