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

#include <inviwo/mergetreemaps/processors/dataframetoimage.h>
#include <modules/base/algorithm/dataminmax.h>
#include <inviwo/core/datastructures/buffer/bufferramprecision.h>
#include <inviwo/core/datastructures/buffer/bufferram.h>
#include <inviwo/core/datastructures/image/layerram.h>
#include <inviwo/core/datastructures/image/layerramprecision.h>
#include <inviwo/core/util/zip.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo DataFrameToImage::processorInfo_{
    "org.inviwo.DataFrameToImage",  // Class identifier
    "Data Frame To Image",          // Display name
    "Plotting",                     // Category
    CodeState::Experimental,        // Code state
    Tags::None,                     // Tags
};
const ProcessorInfo DataFrameToImage::getProcessorInfo() const { return processorInfo_; }

DataFrameToImage::DataFrameToImage()
    : Processor()
    , dimensionMultipliers_("dimMultipliers", "Dim Multipliers", size2_t(4, 4), size2_t(1, 1),
                            size2_t(256, 256), size2_t(1, 1), InvalidationLevel::InvalidOutput,
                            PropertySemantics::Text)
    , dimensions_("dims", "Output Dimensions", size2_t(256, 256), size2_t(1, 1),
                  size2_t(10000, 10000), size2_t(1, 1), InvalidationLevel::InvalidOutput,
                  PropertySemantics::Text)
    , nearestNeighborInterpolation_("nearestNeighborInterpolation", "Use NN interpolation", false)
    , mode_("imageExportMode", "Mode")
    , column_("color", "Color Columns", inport_, false, 1)
    , padLeftRight_("padLeftRight", "Pad Left/Right by 1/2", false)
    , padUntilAtLeastDims_("padRightBottom", "Pad Bottom/Right until Pad Dims", false)
    , paddedDimensions_("padDims", "Padded Dimensions", size2_t(256, 256), size2_t(1, 1),
                        size2_t(10000, 10000), size2_t(1, 1), InvalidationLevel::InvalidOutput,
                        PropertySemantics::Text)
    , padColor_("padColor", "Pad Color", vec4(1), vec4(0), vec4(1), vec4(0.1f),
                InvalidationLevel::InvalidOutput, PropertySemantics::Color)
    , transferFunction_("tf", "Transfer Function")
    , useCustomDataRange_("useCustomRange", "Use Custom Range", false)
    , customDataRange_("customDataRange", "Custom Data Range", 0.0, 1.0,
                       std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max(),
                       0.01, 0.0, InvalidationLevel::InvalidOutput, PropertySemantics::Text)
    , dataRange_("dataRange", "Output Range", 0.0, 1.0, std::numeric_limits<double>::lowest(),
                 std::numeric_limits<double>::max(), 0.01, 0.0, InvalidationLevel::Valid,
                 PropertySemantics::Text)
    , keepMinRangeAt0_("keepMinAt0", "Keep Minimum Range at 0", false) {

    addPort(inport_);
    addPort(outport_);
    addProperties(dimensionMultipliers_, dimensions_, nearestNeighborInterpolation_, mode_, column_,
                  padLeftRight_, padUntilAtLeastDims_, paddedDimensions_, padColor_,
                  transferFunction_, useCustomDataRange_, customDataRange_, dataRange_,
                  keepMinRangeAt0_);

    mode_.addOption("fullDF", "Full DataFrame", 0);
    mode_.addOption("columnOnly", "Selected Column as Row", 1);
    mode_.setSelectedIndex(0);
    mode_.setCurrentStateAsDefault();

    padLeftRight_.visibilityDependsOn(mode_, [](const auto& p) { return p.get() == 1; });
    padUntilAtLeastDims_.visibilityDependsOn(mode_, [](const auto& p) { return p.get() == 0; });
    column_.visibilityDependsOn(mode_, [](const auto& p) { return p.get() == 1; });

    // Prevent updateDataRange to crash based on empty option vector
    // (will be overwritten one a dataframe is connected to the port)
    column_.addOption("dummy", "Dummy", 0);

    dimensions_.setReadOnly(true);

    auto updateDims = [this]() {
        if (inport_.hasData()) {
            const auto df = inport_.getData();
            size_t numRows = df->getNumberOfRows();
            auto multipliers = dimensionMultipliers_.get();
            if (mode_.get() == 0) {
                size_t numColumns = df->getNumberOfColumns() - 1;
                dimensions_.set({numColumns * multipliers.x, numRows * multipliers.y});
            } else {
                if (padLeftRight_.get()) numRows++;
                dimensions_.set({numRows * multipliers.x, multipliers.y});
            }
        }
    };
    dimensionMultipliers_.onChange(updateDims);

    auto updateDataRangeAndDims = [this, updateDims]() {
        if (inport_.hasData()) {
            const auto df = inport_.getData();
            if (mode_.get() == 0) {
                double rangeMin = std::numeric_limits<double>::max();
                double rangeMax = std::numeric_limits<double>::lowest();
                size_t numColumns = df->getNumberOfColumns();
                // Exclude index column
                for (size_t c = 1; c < numColumns; c++) {
                    if (df->getColumn(c)->getBuffer()) {
                        auto buffer = df->getColumn(c)->getBuffer()->getRepresentation<BufferRAM>();
                        auto range = util::bufferMinMax(buffer);
                        if (range.first.x < rangeMin) rangeMin = range.first.x;
                        if (range.second.x > rangeMax) rangeMax = range.second.x;
                    }
                }
                dataRange_.set(dvec2(rangeMin, rangeMax));
            } else {
                if (column_.get() < df->getNumberOfColumns()) {
                    if (column_.getBuffer()) {
                        auto buffer = column_.getBuffer()->getRepresentation<BufferRAM>();
                        auto range = util::bufferMinMax(buffer);
                        dataRange_.set(dvec2(range.first.x, range.second.x));
                    }
                }
            }

        } else {
            // reset data range, no data
            dataRange_.set(dvec2(0.0, 1.0));
        }
        if (!useCustomDataRange_.get()) {
            auto range = dataRange_.get();
            rangeMin_ = range.x;
            rangeMax_ = range.y;
        }
        if (keepMinRangeAt0_.get()) rangeMin_ = 0.0;
        updateDims();
    };
    dataRange_.setReadOnly(true);
    inport_.onChange(updateDataRangeAndDims);
    column_.onChange(updateDataRangeAndDims);
    padLeftRight_.onChange(updateDataRangeAndDims);

    useCustomDataRange_.onChange([&]() {
        customDataRange_.setReadOnly(!useCustomDataRange_.get());
        if (useCustomDataRange_.get()) {
            auto range = customDataRange_.get();
            rangeMin_ = range.x;
            rangeMax_ = range.y;
        } else {
            auto range = dataRange_.get();
            rangeMin_ = range.x;
            rangeMax_ = range.y;
        }
        if (keepMinRangeAt0_.get()) rangeMin_ = 0.0;
    });
    customDataRange_.setReadOnly(!useCustomDataRange_.get());
    customDataRange_.onChange([&]() {
        auto range = customDataRange_.get();
        rangeMin_ = range.x;
        rangeMax_ = range.y;
        if (keepMinRangeAt0_.get()) rangeMin_ = 0.0;
    });
}

void DataFrameToImage::process() {
    const auto df = inport_.getData();
    if (!df) return;
    // Number of columns includes index column, which we exclude here
    size_t numColumns = df->getNumberOfColumns() - 1;
    size_t numRows = df->getNumberOfRows();

    // Empty dataframe
    if (numColumns == 0 || numRows == 0) return;
    // Selected column is outside of range (from deserialization?)
    if (mode_.get() == 1 && column_.get() >= (numColumns + 1)) return;

    auto multiplier = dimensionMultipliers_.get();
    size_t xDims = numColumns * multiplier.x;
    size_t yDims = numRows * multiplier.y;

    size2_t paddedDims = paddedDimensions_.get();
    size_t numColumnsPadded = numColumns;
    size_t numRowsPadded = numRows;

    // Column as Pixel Row in X, y dim is 1, xDim is number of rows (+1 with padding)
    if (mode_.get() == 1) {
        xDims = numRows * multiplier.x;
        if (padLeftRight_.get()) xDims += multiplier.x;
        yDims = 1 * multiplier.y;
    } else if (mode_.get() == 0 && padUntilAtLeastDims_.get()) {
        if (paddedDims[0] < numColumns) {
            LogProcessorWarn(
                "Padded dims for columns are smalled than actual dims, proceeding with actual "
                "ones.")
        } else {
            numColumnsPadded = std::max(paddedDims[0], numColumns);
            xDims = numColumnsPadded * multiplier.x;
        }
        if (paddedDims[1] < numRows) {
            LogProcessorWarn(
                "Padded dims for rows are smalled than actual dims, proceeding with actual ones.")
        } else {
            numRowsPadded = std::max(paddedDims[1], numRows);
            yDims = numRowsPadded * multiplier.y;
        }
    }

    // Create output
    auto heatmap = std::make_shared<Image>(size2_t{xDims, yDims}, DataVec4Float32::get());
    auto imgRam = heatmap->getColorLayer()->getEditableRepresentation<LayerRAM>();

    if (mode_.get() == 0) {
        for (size_t c = 0; c < numColumnsPadded; c++) {
            const BufferRAM* currColumn;
            if (c < numColumns) {
                currColumn = df->getColumn(c + 1)->getBuffer()->getRepresentation<BufferRAM>();
            }
            for (size_t r = 0; r < numRowsPadded; r++) {
                vec4 color;
                if (c < numColumns && r < numRows && currColumn) {
                    // TODO: This must be crazy slow
                    // Iterate from the back as image indices start from the lower left corner
                    double val = currColumn->getAsDouble(numRows - 1 - r);
                    double normalizedValue = (val - rangeMin_) / (rangeMax_ - rangeMin_);
                    color = transferFunction_.get().sample(normalizedValue);
                } else {
                    color = padColor_.get();
                }

                // Repeat this for timestepwidth pixels
                size_t xEnd = (c + 1) * multiplier.x;
                size_t yEnd = (r + 1) * multiplier.y;
                for (size_t xIndex = c * multiplier.x; xIndex < xEnd; xIndex++)
                    for (size_t yIndex = r * multiplier.y; yIndex < yEnd; yIndex++)
                        imgRam->setFromDVec4({xIndex, yIndex}, color);
            }
        }
    } else {
        auto currColumn = column_.getBuffer()->getRepresentation<BufferRAM>();
        size_t paddingOneSize = padLeftRight_ ? multiplier.x / 2 : 0;
        for (size_t r = 0; r < numRows; r++) {
            // Iterate in order
            double val = currColumn->getAsDouble(r);
            double normalizedValue = (val - rangeMin_) / (rangeMax_ - rangeMin_);
            vec4 color = transferFunction_.get().sample(normalizedValue);
            size_t xEnd = paddingOneSize + (r + 1) * multiplier.x;
            // Each value has height of multiplier.y and width of multiplier.
            for (size_t xIndex = paddingOneSize + r * multiplier.x; xIndex < xEnd; xIndex++)
                for (size_t yIndex = 0; yIndex < multiplier.y; yIndex++)
                    imgRam->setFromDVec4({xIndex, yIndex}, color);
        }
        if (padLeftRight_) {
            vec4 color = padColor_.get();
            for (size_t yIndex = 0; yIndex < multiplier.y; yIndex++) {
                // Pad in the beginning
                for (size_t xIndex = 0; xIndex < paddingOneSize; xIndex++)
                    imgRam->setFromDVec4({xIndex, yIndex}, color);
                // Pad in the end
                for (size_t xIndex = paddingOneSize + numRows * multiplier.x; xIndex < xDims;
                     xIndex++)
                    imgRam->setFromDVec4({xIndex, yIndex}, color);
            }
        }
    }

    // Set nearest Neighbor interpolation
    if (nearestNeighborInterpolation_.get()) imgRam->setInterpolation(InterpolationType::Nearest);

    outport_.setData(heatmap);
}

}  // namespace inviwo
