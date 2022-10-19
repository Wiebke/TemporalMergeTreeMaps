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

#include <inviwo/mergetreemaps/processors/generatedatatargetlevelset.h>
#include <inviwo/mergetreemaps/mergetreemapsmodule.h>
#include <inviwo/core/util/filesystem.h>
#include <modules/base/algorithm/dataminmax.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo GenerateDataTargetLevelSet::processorInfo_{
    "org.inviwo.GenerateDataTargetLevelSet",  // Class identifier
    "Generate Data Target Level Set",         // Display name
    "Data Generation",                        // Category
    CodeState::Experimental,                  // Code state
    Tags::None,                               // Tags
};
const ProcessorInfo GenerateDataTargetLevelSet::getProcessorInfo() const { return processorInfo_; }

GenerateDataTargetLevelSet::GenerateDataTargetLevelSet()
    : Processor()
    , outport_("outport")
    , segmentationOutport_("segmentationOutport")
    // Common properties
    , dimensions_("dims", "Dimensions", size2_t(128), size2_t(1), size2_t(1024))
    , xRange_("xRange", "X Range", -2, 2, -10, 10)
    , yRange_("yRange", "Y Range", -2, 2, -10, 10)
    , offset_("offset", "Offset", 0, -100, 10, 0.001f)
    , scaleFactor_("scaleFactor", "Scale Factor", 1, -10, 10, 0.001f)
    , useCustomDataRange_("useCustomRange", "Use Custom Range", false)
    , customDataRange_("customDataRange", "Custom Data Range", 0.0, 1.0,
                       std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max(),
                       0.01, 0.0, InvalidationLevel::InvalidOutput, PropertySemantics::Text)
    , dataRange_("dataRange", "Output Range", 0.0, 1.0, std::numeric_limits<double>::lowest(),
                 std::numeric_limits<double>::max(), 0.01, 0.0, InvalidationLevel::Valid,
                 PropertySemantics::Text)
    , function_("functionType", "Function")
    // Level set properties
    , levelSet_("levelSet", "(Sub/Super) Level Set")
    , levelSetComparison_("levelSetComparison", "Comparison")
    , levelSetThreshold_("levelSetThreshold", "Level Set Threshold", 0.0f, -100.0f, 100, 0.0001f)
    , levelSetSize_("levelSetSize", "Total Size at Threshold", 0, 0,
                    std::numeric_limits<size_t>::max())
    , levelSetThreshold2_("levelSetThreshold2", "Level Set Threshold 2", 0.0f, -100.0f, 100, 0.0001f)
    , levelSetSize2_("levelSetSize2", "Total Size at Threshold 2", 0, 0,
                    std::numeric_limits<size_t>::max())
    // Gauss properties
    , gauss_("gauss", "Gaussian")
    , gaussCov_("gaussCov", "Covariance (Sigma)", mat2(1, 0, 0, 1), mat2(0), mat2(10), mat2(0.01),
                InvalidationLevel::InvalidOutput, PropertySemantics::Text)
    , gaussMu_("gaussMu", "Mean (mu)", vec2(0, 0), vec2(-10, -10), vec2(10, 10))
    // Skewed normal properties
    , skewedNormal_("skewedNormal", "Skewed Normal")
    , skewedNormalShape_("skewedNormalShape", "Shape (alpha)", vec2(0, 0), vec2(-10, -10),
                         vec2(10, 10))
    , skewedNormalCov_("skewedNormalCov", "Covariance (Sigma)", mat2(1, 0, 0, 1), mat2(0), mat2(10),
                       mat2(0.01), InvalidationLevel::InvalidOutput, PropertySemantics::Text)
    , skewedNormalScale_("skewedNormalScale", "Scale (omega)", 1.f, 0.00001f, 10.f)
    , skewedNormalLocation_("skewedNormalLoc", "Location (xi)", vec2(0, 0), vec2(-10, -10),
                            vec2(10, 10))
    , delta_("delta", "Delta Sequence")
    , deltaEpsilon_("deltaEpsilon", "Epsilon", 1.f, 0.00001f, 10.f, 0.00001f)
    , ring_("ring", "Gaussian Ring")
    , ringEpicenter_("ringEpicenter", "Epicenter (c)", vec2(0, 0), vec2(-10, -10), vec2(10, 10))
    , ringPeak_("ringPeak", "Peak (p)", 1.f, 0.00001f, 10.f, 0.00001f)
    , ringMu_("ringMu", "Median Radius (mu)", 1.f, 0.00001f, 10.f, 0.00001f)
    , ringStd_("ringStd", "Std Radius (omega)", 1.f, 0.00001f, 10.f, 0.00001f)
    , script_(InviwoApplication::getPtr()->getModuleByType<MergeTreeMapsModule>()->getPath(
                  ModulePath::Scripts) +
              "/testFunctions.py") {
    addPort(outport_);
    addPort(segmentationOutport_);

    addProperties(dimensions_, xRange_, yRange_, offset_, scaleFactor_, useCustomDataRange_,
                  customDataRange_, dataRange_, function_, levelSet_, gauss_, skewedNormal_, delta_,
                  ring_);
    util::hide(gauss_, skewedNormal_, delta_, ring_);
    dataRange_.setReadOnly(true);

    function_.addOption("gaussian", "Gaussian", FunctionType::Gaussian);
    function_.addOption("skewedNormal", "Skewed Normal", FunctionType::SkewedNormal);
    function_.addOption("deltaSequence", "Delta Sequence", FunctionType::DeltaSeq);
    function_.addOption("ackley", "Ackley Function", FunctionType::Ackley);
    function_.addOption("ridge", "Single Ridge", FunctionType::Ridge);
    function_.addOption("himmelblau", "Himmelblau", FunctionType::Himmelblau);
    function_.addOption("ring", "Gaussian Ring", FunctionType::Ring);
    function_.addOption("prototype", "Prototype", FunctionType::Prototype);
    
    levelSet_.addProperties(levelSetComparison_, levelSetThreshold_, levelSetSize_,
                            levelSetThreshold2_, levelSetSize2_);

    levelSetComparison_.addOption("smaller", "f < Threshold (Join Tree)", ComparisonType::SMALLER);
    levelSetComparison_.addOption("smallerEqual", "f <= Threshold (Join Tree)",
                                  ComparisonType::SMALLEREQUAL);
    levelSetComparison_.addOption("greaterEqual", "f >= Threshold (Split Tree)",
                                  ComparisonType::GREATEREQUAL);
    levelSetComparison_.addOption("greater", "f > Threshold (Split Tree)", ComparisonType::GREATER);
    levelSetSize_.setSemantics(PropertySemantics::Text);
    levelSetSize_.setReadOnly(true);
    levelSetSize2_.setSemantics(PropertySemantics::Text);
    levelSetSize2_.setReadOnly(true);

    gauss_.addProperties(gaussCov_, gaussMu_);
    gauss_.visibilityDependsOn(function_,
                               [](const auto& p) { return p.get() == FunctionType::Gaussian; });

    skewedNormal_.addProperties(skewedNormalShape_, skewedNormalCov_, skewedNormalScale_,
                                skewedNormalLocation_);
    skewedNormal_.visibilityDependsOn(
        function_, [](const auto& p) { return p.get() == FunctionType::SkewedNormal; });

    delta_.addProperties(deltaEpsilon_);
    delta_.visibilityDependsOn(function_,
                               [](const auto& p) { return p.get() == FunctionType::DeltaSeq; });

    ring_.addProperties(ringEpicenter_, ringPeak_, ringMu_, ringStd_);
    ring_.visibilityDependsOn(function_,
                              [](const auto& p) { return p.get() == FunctionType::Ring; });
}

void GenerateDataTargetLevelSet::process() {
    auto dims = dimensions_.get();
    auto sizeX = dims.x;
    auto sizeY = dims.y;
    auto vol = std::make_shared<Volume>(size3_t(sizeX, sizeY, 1), DataFloat32::get());
    auto volSegmentation = std::make_shared<Volume>(size3_t(sizeX, sizeY, 1), DataFloat32::get());

    auto locals = pybind11::globals();
    // Input
    // Needs to specify:
    // size_x, size_y,
    // range_x, range_y,
    locals["size_x"] = pybind11::cast(sizeX);
    locals["size_y"] = pybind11::cast(sizeY);
    locals["range_x"] = pybind11::cast(xRange_.get());
    locals["range_y"] = pybind11::cast(yRange_.get());
    locals["offset"] = pybind11::cast(offset_.get());
    locals["scaleFactor"] = pybind11::cast(scaleFactor_.get());

    locals["function"] = pybind11::cast(static_cast<int>(function_.get()));

    locals["comparison"] = pybind11::cast(static_cast<int>(levelSetComparison_.get()));
    locals["threshold"] = pybind11::cast(levelSetThreshold_.get());
    locals["threshold2"] = pybind11::cast(levelSetThreshold2_.get());

    locals["vol"] = pybind11::cast(vol.get());
    locals["volSeg"] = pybind11::cast(volSegmentation.get());

    if (function_.get() == FunctionType::Gaussian) {
        locals["cov"] = pybind11::cast(gaussCov_.get());
        locals["loc"] = pybind11::cast(gaussMu_.get());
    } else if (function_.get() == FunctionType::SkewedNormal) {
        locals["shape"] = pybind11::cast(skewedNormalShape_.get());
        locals["cov"] = pybind11::cast(skewedNormalCov_.get());
        locals["loc"] = pybind11::cast(skewedNormalLocation_.get());
        locals["scale"] = pybind11::cast(skewedNormalScale_.get());
    } else if (function_.get() == FunctionType::DeltaSeq) {
        locals["epsilon"] = pybind11::cast(deltaEpsilon_.get());
    } else if (function_.get() == FunctionType::Ring) {
        locals["loc"] = pybind11::cast(ringEpicenter_.get());
        locals["peak"] = pybind11::cast(ringPeak_.get());
        locals["muRadius"] = pybind11::cast(ringMu_.get());
        locals["stdRadius"] = pybind11::cast(ringStd_.get());
    }

    try {
        script_.run(locals, [&](pybind11::dict dict) {
            levelSetSize_.set(pybind11::cast<int>(dict["levelSetSize"]));
            levelSetSize2_.set(pybind11::cast<int>(dict["levelSetSize2"]));
        });
    } catch (std::exception& e) {
        LogError(e.what())
    }

    auto range = util::volumeMinMax(vol->getRepresentation<VolumeRAM>());
    // Use customRange as placeholder for actual data range
    vec2 customRange = vec2(range.first.x, range.second.x);
    dataRange_.set(customRange);
    if (useCustomDataRange_) {
        // Overwrite with property setting: Only relevant for colormap, does not change data
        customRange = customDataRange_.get();
    }
    vol->dataMap_.dataRange = customRange;
    vol->dataMap_.valueRange = customRange;

	// Either: No component (0), first component (1) or second component (2)
    volSegmentation->dataMap_.dataRange = vec2(0.0, 2.0);
    volSegmentation->dataMap_.valueRange = vec2(0.0, 2.0);
    volSegmentation->getEditableRepresentation<VolumeRAM>()->setInterpolation(
        InterpolationType::Nearest);

    outport_.setData(vol);
    segmentationOutport_.setData(volSegmentation);
}

}  // namespace inviwo
