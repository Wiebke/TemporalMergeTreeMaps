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

#include <inviwo/mergetreemaps/processors/mergetreemap.h>
#include <modules/base/algorithm/dataminmax.h>
#include <inviwo/core/datastructures/buffer/bufferramprecision.h>
#include <inviwo/core/datastructures/buffer/bufferram.h>
#include <inviwo/core/datastructures/image/layerram.h>
#include <inviwo/core/datastructures/image/layerramprecision.h>
#include <inviwo/core/util/zip.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo MergeTreeMap::processorInfo_{
    "org.inviwo.MergeTreeMap",  // Class identifier
    "Merge Tree Map",           // Display name
    "Merge Tree Maps",          // Category
    CodeState::Experimental,    // Code state
    Tags::None,                 // Tags
};
const ProcessorInfo MergeTreeMap::getProcessorInfo() const { return processorInfo_; }

MergeTreeMap::MergeTreeMap()
    : Processor()
    , outport_("outport")
    // Image cannot be larger than maximum texture in GL
    , maxDataDimSize_("maxdataDimSize", "Max Data Dim Size", 4096, 8, 16384, 1,
                      InvalidationLevel::InvalidOutput, PropertySemantics::Text)
    , imageDataDimSize_("imageDataDimSize", "Result Data Dim Size", 128, 1, 16384, 1,
                        InvalidationLevel::Valid, PropertySemantics::Text)
    , timestepWidth_("timestepWidth", "Time Step Width", 10, 1, 16384, 1,
                     InvalidationLevel::InvalidOutput, PropertySemantics::Text)
    , imageTimeDimSize_("imageTimeDimSize", "Result Time Dim Size", 128, 1, 16384, 1,
                        InvalidationLevel::Valid, PropertySemantics::Text)
    , nearestNeighborInterpolation_("nearestNeighborInterpolation", "Use NN interpolation", false)
    , restrictPositionRange_("restrictPositionRange", "Restrict Position Range", false)
    , excludeFirstX_("excludeFirstX", "Exclude First X Positions", 0, 0,
                     std::numeric_limits<int>::max(), 1, InvalidationLevel::InvalidOutput,
                     PropertySemantics::Text)
    , excludeFirstXPercentage_("excludeFirstXPercentage", "% First Excluded", 0.0f, 0.0f, 100.0,
                               1e-10, InvalidationLevel::Valid, PropertySemantics::Text)
    , excludeLastX_("excludeLastX", "Exclude Last X Positions", 0, 0,
                    std::numeric_limits<int>::max(), 1, InvalidationLevel::InvalidOutput,
                    PropertySemantics::Text)
    , excludeLastXPercentage_("excludeLastXPercentage", "% Last Excluded", 0.0f, 0.0f, 100.0, 1e-10,
                              InvalidationLevel::Valid, PropertySemantics::Text)
    , timeDirection_("timeDirection", "Time Direction")
    , positionColumn_("position", "Position Columns", dataFramePort_, false, 1)
    , colorColumn_("color", "Color Columns", dataFramePort_, false, 2)
    , colorColumnContainsColorStrings_("directColorString", "Contains Color String", false)
    , transferFunction_("tf", "Transfer Function")
    , useCustomDataRange_("useCustomRange", "Use Custom Range", false)
    , customDataRange_("customDataRange", "Custom Data Range", 0.0, 1.0,
                       std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max(),
                       0.01, 0.0, InvalidationLevel::InvalidOutput, PropertySemantics::Text)
    , dataRange_("dataRange", "Output Range", 0.0, 1.0, std::numeric_limits<double>::lowest(),
                 std::numeric_limits<double>::max(), 0.01, 0.0, InvalidationLevel::Valid,
                 PropertySemantics::Text)
    , interpolation_("interpolation", "Interpolation")
    , timer_("timer", "Eval Time (s)", 0.f, 0.f, std::numeric_limits<float>::max(), 0.001f,
             InvalidationLevel::Valid, PropertySemantics::Text) {
    addPort(inport_);
    addPort(dataFramePort_);
    addPort(outport_);
    addProperties(maxDataDimSize_, imageDataDimSize_, timestepWidth_, imageTimeDimSize_,
                  nearestNeighborInterpolation_, restrictPositionRange_, excludeFirstX_,
                  excludeFirstXPercentage_, excludeLastX_, excludeLastXPercentage_, positionColumn_,
                  colorColumn_);

    imageDataDimSize_.setReadOnly(true);
    imageTimeDimSize_.setReadOnly(true);
    excludeFirstXPercentage_.setReadOnly(true);
    excludeLastXPercentage_.setReadOnly(true);
    excludeFirstX_.visibilityDependsOn(restrictPositionRange_,
                                       [](const auto& p) { return p.get(); });
    excludeFirstXPercentage_.visibilityDependsOn(restrictPositionRange_,
                                       [](const auto& p) { return p.get(); });
    excludeLastX_.visibilityDependsOn(restrictPositionRange_,
                                       [](const auto& p) { return p.get(); });
    excludeLastXPercentage_.visibilityDependsOn(restrictPositionRange_,
                                       [](const auto& p) { return p.get(); });



    timeDirection_.addOption("tLeftRight", "t on x, left to right", 0);
    timeDirection_.addOption("tUp", "t on y, upwards", 1);
    timeDirection_.addOption("tDown", "t on y, downwards", 2);
    timeDirection_.setSelectedValue(0);
    timeDirection_.setCurrentStateAsDefault();

    // Prevent updateDataRange to crash based on empty option vector
    // (will be overwritten one a dataframe is connected to the port)
    colorColumn_.addOption("dummy", "Dummy", 0);

    addProperty(useCustomDataRange_);
    addProperty(customDataRange_);
    addProperty(dataRange_);
    addProperty(transferFunction_);

    interpolation_.addOption("repeat", "Repeat", 0);
    interpolation_.addOption("linear", "Linear", 1);
    addProperty(interpolation_);

    auto updateDataRange = [this]() {
        if (inport_.hasData() && inport_.getData()->size() > 0 && dataFramePort_.hasData()) {
            const auto dfFrames = inport_.getData();
            auto idx = colorColumn_.get();
            double rangeMin = std::numeric_limits<double>::max();
            double rangeMax = std::numeric_limits<double>::lowest();
            for (size_t i = 0; i < inport_.getData()->size(); i++) {
                auto currentDf = dfFrames->at(i);
                // In case the dataframe changed, but the columns have not updated yet
                if (!currentDf || idx >= currentDf->getNumberOfColumns()) return;
                auto col = currentDf->getColumn(idx);
                auto buffer =
                    currentDf->getColumn(idx)->getBuffer()->getRepresentation<BufferRAM>();
                auto range = util::bufferMinMax(buffer);
                if (range.first.x < rangeMin) rangeMin = range.first.x;
                if (range.second.x > rangeMax) rangeMax = range.second.x;
            }
            dataRange_.set(dvec2(rangeMin, rangeMax));
        } else {
            // reset data range, no data
            dataRange_.set(dvec2(0.0, 1.0));
        }
        if (!useCustomDataRange_.get()) {
            auto range = dataRange_.get();
            rangeMin_ = range.x;
            rangeMax_ = range.y;
        }
    };
    dataRange_.setReadOnly(true);
    inport_.onChange(updateDataRange);
    colorColumn_.onChange(updateDataRange);

    useCustomDataRange_.onChange([&]() {
        customDataRange_.setReadOnly(!useCustomDataRange_.get());
        dataRange_.setReadOnly(useCustomDataRange_.get());
        if (useCustomDataRange_.get()) {
            auto range = customDataRange_.get();
            rangeMin_ = range.x;
            rangeMax_ = range.y;
        } else {
            auto range = dataRange_.get();
            rangeMin_ = range.x;
            rangeMax_ = range.y;
        }
    });
    customDataRange_.setReadOnly(!useCustomDataRange_.get());
    customDataRange_.onChange([&]() {
        auto range = customDataRange_.get();
        rangeMin_ = range.x;
        rangeMax_ = range.y;
    });

    addProperty(timer_);
}

void MergeTreeMap::process() {
    performanceTimer_.Reset();
    const auto dfFrames = inport_.getData();
    const auto modelFrame = dataFramePort_.getData();
    if (!dfFrames || !modelFrame) return;
    size_t numTimeSteps = dfFrames->size();
    if (numTimeSteps < 1) return;
    size_t tDims = timestepWidth_ * numTimeSteps;

    size_t dataDims = dfFrames->at(0)->getNumberOfRows();
    size_t fullDomain = dataDims;
    size_t startingIndex = restrictPositionRange_.get()
                               ? std::min(static_cast<size_t>(excludeFirstX_.get()), dataDims - 1)
                               : 0;
    excludeFirstXPercentage_.set(static_cast<float>(excludeFirstX_.get()) / dataDims);
    size_t endingIndex =
        restrictPositionRange_.get()
            ? static_cast<size_t>(std::max(static_cast<int>(dataDims - 1) - excludeLastX_.get(),
                                           static_cast<int>(startingIndex)))
            : dataDims - 1;
    excludeLastXPercentage_.set(static_cast<float>(excludeLastX_.get()) / dataDims);

    dataDims = endingIndex - startingIndex + 1;
    LogProcessorInfo("Covering " << dataDims << " of the full domain of " << fullDomain)

        size_t subSampleFactor = 1;
    if (dataDims > maxDataDimSize_.get()) {
        subSampleFactor = dataDims / maxDataDimSize_ + (dataDims % maxDataDimSize_ != 0);
        LogProcessorInfo("Subsample Factor " << subSampleFactor);
    }

    // Round up to ensure the last sample makes it in
    size_t finalDataDims = dataDims / subSampleFactor + (dataDims % subSampleFactor != 0);
    imageDataDimSize_.set(finalDataDims);
    imageTimeDimSize_.set(tDims);
    int timeDirection = timeDirection_.get();

    // ToDo: Altnerative Subsampling: Average, keep topology...
    std::shared_ptr<Image> mtm;
    if (timeDirection == 0) {
        mtm = std::make_shared<Image>(size2_t{tDims, finalDataDims}, DataVec4Float32::get());
    } else {
        mtm = std::make_shared<Image>(size2_t{finalDataDims, tDims}, DataVec4Float32::get());
    }

    // Recover full dataDims again
    dataDims = fullDomain;

    auto imgRam = mtm->getColorLayer()->getEditableRepresentation<LayerRAM>();

    if (interpolation_.get() == 0 || timestepWidth_ == 1) {
        for (size_t t = 0; t < numTimeSteps; t++) {
            // Get the current dataframe and it's selected columns
            auto currentDf = dfFrames->at(t);
            if (!currentDf) {
                LogProcessorError("Not getting input. Aborting.");
                return;
            }
            auto posIdx = positionColumn_.get();
            auto posColumn =
                currentDf->getColumn(posIdx)->getBuffer()->getRepresentation<BufferRAM>();
            auto colorIdx = colorColumn_.get();
            auto colorColumn =
                currentDf->getColumn(colorIdx)->getBuffer()->getRepresentation<BufferRAM>();

            for (size_t dIndex = 0; dIndex < dataDims; dIndex++) {
                // TODO: This must be crazy slow
                int pos = static_cast<int>(posColumn->getAsDouble(dIndex));
                if (pos < startingIndex || pos > endingIndex) continue;
                double colorValue = colorColumn->getAsDouble(dIndex);
                double normalizedColorValue = (colorValue - rangeMin_) / (rangeMax_ - rangeMin_);
                vec4 color = transferFunction_.get().sample(normalizedColorValue);
                size_t endIndex = (t + 1) * timestepWidth_.get();
                // Repeat this for timestepwidth pixels
                for (size_t tIndex = t * timestepWidth_.get(); tIndex < endIndex; tIndex++) {
                    if (pos % subSampleFactor == 0) {
                        int subSampledPos = (pos - startingIndex) / subSampleFactor;
                        if (subSampledPos < 0 || subSampledPos >= finalDataDims) {
                            LogProcessorWarn("Skipping datavalue at t="
                                             << tIndex << ", pos=" << pos
                                             << " because it is out of range.");
                            continue;
                        }
                        if (timeDirection == 0) {
                            // time on x, left to right
                            imgRam->setFromDVec4({tIndex, subSampledPos}, color);

                        } else if (timeDirection == 1) {
                            // time on y upwards
                            imgRam->setFromDVec4({subSampledPos, tIndex}, color);
                        } else {
                            // time on y downwards
                            imgRam->setFromDVec4({subSampledPos, tDims - 1 - tIndex}, color);
                        }
                    }
                }
            }
        }
    } else if (interpolation_.get() == 1) {

        std::vector<double> values1(finalDataDims);
        std::vector<double> values2(finalDataDims);
        bool newOnFirst = false;

        for (size_t t = 0; t < numTimeSteps - 1; t++) {
            // Get the current and next dataframe and it's selected columns, build a map for
            // y-position and value, necessary because positions in the dataframe or not in order
            auto posIdx = positionColumn_.get();
            auto colorIdx = colorColumn_.get();
            if (t == 0) {
                auto currentDf = dfFrames->at(t);
                auto posColumn =
                    currentDf->getColumn(posIdx)->getBuffer()->getRepresentation<BufferRAM>();
                auto colorColumn =
                    currentDf->getColumn(colorIdx)->getBuffer()->getRepresentation<BufferRAM>();
                for (size_t dIndex = 0; dIndex < dataDims; dIndex++) {
                    int pos = static_cast<int>(posColumn->getAsDouble(dIndex));
                    if (pos < startingIndex || pos > endingIndex) continue;
                    double colorValue = colorColumn->getAsDouble(dIndex);
                    if (pos % subSampleFactor == 0) {
                        int subSampledPos = (pos - startingIndex) / subSampleFactor;
                        if (subSampledPos < 0 || subSampledPos >= finalDataDims) {
                            LogProcessorWarn("Skipping datavalue at t="
                                             << t << ", pos=" << pos
                                             << " because it is out of range.");
                            continue;
                        }
                        values1[subSampledPos] = colorValue;
                    }
                }
            }

            auto currentDfNext = dfFrames->at(t + 1);
            auto posColumnNext =
                currentDfNext->getColumn(posIdx)->getBuffer()->getRepresentation<BufferRAM>();
            auto colorColumnNext =
                currentDfNext->getColumn(colorIdx)->getBuffer()->getRepresentation<BufferRAM>();

            for (size_t dIndex = 0; dIndex < dataDims; dIndex++) {
                int pos = static_cast<int>(posColumnNext->getAsDouble(dIndex));
                if (pos < startingIndex || pos > endingIndex) continue;
                double colorValue = colorColumnNext->getAsDouble(dIndex);
                if (pos % subSampleFactor == 0) {
                    int subSampledPos = (pos - startingIndex) / subSampleFactor;
                    if (subSampledPos < 0 || subSampledPos >= finalDataDims) {
                        LogProcessorWarn("Skipping datavalue at t="
                                         << t << ", pos=" << pos << " because it is out of range.");
                        continue;
                    }
                    if (newOnFirst) {
                        values1[subSampledPos] = colorValue;
                    } else {
                        values2[subSampledPos] = colorValue;
                    }
                }
            }

            // timeStepWidth is at least 2 here
            size_t halfWidth = timestepWidth_.get() / 2;

            for (size_t dIndex = 0; dIndex < finalDataDims; dIndex++) {
                size_t startIndex = t * timestepWidth_.get() + halfWidth;
                size_t endIndex = (t + 1) * timestepWidth_.get() + halfWidth;
                // Fill the first half of the first time step by repeating the color value
                if (t == 0) {
                    double colorValue = values1[dIndex];
                    double normalizedColorValue =
                        (colorValue - rangeMin_) / (rangeMax_ - rangeMin_);
                    vec4 color = transferFunction_.get().sample(normalizedColorValue);
                    for (size_t tIndex = 0; tIndex < startIndex; tIndex++) {
                        if (timeDirection == 0) {
                            // time on x, left to right
                            imgRam->setFromDVec4({tIndex, dIndex}, color);
                        } else if (timeDirection == 1) {
                            // time on y upwards
                            imgRam->setFromDVec4({dIndex, tIndex}, color);
                        } else {
                            // time on y downwards
                            imgRam->setFromDVec4({dIndex, tDims - 1 - tIndex}, color);
                        }
                    }
                }
                // Fill the last half of the last time step by repeating the color value
                // (last index is numTimeSteps-1, so last prev one is numTimeSteps-2
                if (t == numTimeSteps - 2) {
                    double colorValue = newOnFirst ? values1[dIndex] : values2[dIndex];
                    double normalizedColorValue =
                        (colorValue - rangeMin_) / (rangeMax_ - rangeMin_);
                    vec4 color = transferFunction_.get().sample(normalizedColorValue);
                    for (size_t tIndex = endIndex; tIndex < tDims; tIndex++) {
                        if (timeDirection == 0) {
                            // time on x, left to right
                            imgRam->setFromDVec4({tIndex, dIndex}, color);
                        } else if (timeDirection == 1) {
                            // time on y upwards
                            imgRam->setFromDVec4({dIndex, tIndex}, color);
                        } else {
                            // time on y downwards
                            imgRam->setFromDVec4({dIndex, tDims - 1 - tIndex}, color);
                        }
                    }
                }

                double firstValue = newOnFirst ? values2[dIndex] : values1[dIndex];
                double secondValue = newOnFirst ? values1[dIndex] : values2[dIndex];

                for (size_t tIndex = startIndex; tIndex < endIndex; tIndex++) {
                    double interpolateT =
                        (tIndex - startIndex) / (static_cast<double>(timestepWidth_.get()) - 1.0);
                    double colorValue =
                        (1 - interpolateT) * firstValue + interpolateT * secondValue;
                    double normalizedColorValue =
                        (colorValue - rangeMin_) / (rangeMax_ - rangeMin_);
                    vec4 color = transferFunction_.get().sample(normalizedColorValue);
                    if (timeDirection == 0) {
                        // time on x, left to right
                        imgRam->setFromDVec4({tIndex, dIndex}, color);
                    } else if (timeDirection == 1) {
                        // time on y upwards
                        imgRam->setFromDVec4({dIndex, tIndex}, color);
                    } else {
                        // time on y downwards
                        imgRam->setFromDVec4({dIndex, tDims - 1 - tIndex}, color);
                    }
                }
            }

            newOnFirst = !newOnFirst;
        }
    }

    if (nearestNeighborInterpolation_.get()) imgRam->setInterpolation(InterpolationType::Nearest);

    size_t upSampleFactor = 1;
    if (dataDims < maxDataDimSize_.get()) {
        upSampleFactor = maxDataDimSize_ / dataDims;
        if (upSampleFactor > 1) {
            LogProcessorWarn(
                "Upsampling causing additional iteration over data, should only be used for paper "
                "export, when using repeat mode, otherwise use bilinear upscaling! Factor: "
                << upSampleFactor);
        }
        finalDataDims = dataDims * upSampleFactor;
        imageDataDimSize_.set(finalDataDims);
        std::shared_ptr<Image> mtmUpsampled;
        if (timeDirection == 0) {
            mtmUpsampled =
                std::make_shared<Image>(size2_t{tDims, finalDataDims}, DataVec4Float32::get());
        } else {
            mtmUpsampled =
                std::make_shared<Image>(size2_t{finalDataDims, tDims}, DataVec4Float32::get());
        }
        auto imgRamUpsampled = mtmUpsampled->getColorLayer()->getEditableRepresentation<LayerRAM>();
        for (size_t t = 0; t < tDims; t++) {
            for (size_t d = 0; d < dataDims; d++) {
                for (size_t i = 0; i < upSampleFactor; i++) {
                    size_t dIndex = d * upSampleFactor + i;
                    if (timeDirection == 0) {
                        // time on x, left to right
                        imgRamUpsampled->setFromDVec4({t, dIndex}, imgRam->getAsDVec4({t, d}));
                    } else {
                        // time on y
                        imgRamUpsampled->setFromDVec4({dIndex, t}, imgRam->getAsDVec4({d, t}));
                    }
                }
            }
        }
        if (nearestNeighborInterpolation_.get())
            imgRamUpsampled->setInterpolation(InterpolationType::Nearest);
        mtm = mtmUpsampled;
    }

    outport_.setData(mtm);
    timer_.set(performanceTimer_.ElapsedTimeAndReset());
}

}  // namespace inviwo
