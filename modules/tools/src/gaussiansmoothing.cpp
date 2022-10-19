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

#include <modules/tools/gaussiansmoothing.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo GaussianSmoothing::processorInfo_{
    "org.inviwo.GaussianSmoothing",  // Class identifier
    "Gaussian Smoothing",            // Display name
    "Volume Operation",              // Category
    CodeState::Experimental,         // Code state
    Tags::None,                      // Tags
};
const ProcessorInfo GaussianSmoothing::getProcessorInfo() const { return processorInfo_; }

GaussianSmoothing::GaussianSmoothing()
    : Processor()
    , inport_("inport")
    , outport_("outport")
    , kernelRadius_("kernelRadius", "Kernel Radious", 1, 0, 1024)
    , kernelSize_("kernelSize", "Kernel size", 3, 1, 2028, 1, InvalidationLevel::Valid)
    , sigma_("sigma", "Sigma", 1.f, 0.001f, 10.f, 0.001f)
    , boundaryHandling_("boundaryHandling", "Boundary Handling")
    , smoothOverSequence_("smoothOverSeq", "Smooth over (nD+T)", false)
    , iterations_("iterations", "Iterations", 1, 1, 32)
    , timer_("timer", "Eval Time (s)", 0.f, 0.f, std::numeric_limits<float>::max(), 0.001f,
             InvalidationLevel::Valid, PropertySemantics::Text)
    {

    addPort(inport_);
    addPort(outport_);

    addProperties(kernelRadius_, kernelSize_, sigma_, boundaryHandling_, smoothOverSequence_,
                  iterations_);

    sigma_.onChange([&]() {
        int kernelSize90 = static_cast<int>(sigma_.get() * 2 * 1.645f);
        int kernelSize95 = static_cast<int>(sigma_.get() * 2 * 1.960f);
        int kernelSize99 = static_cast<int>(sigma_.get() * 2 * 2.576f);
        // https://de.wikipedia.org/wiki/Normalverteilung
        LogInfo("Optimizal kernelSize for sigma " << sigma_.get() << " is: "
                                                  << "\n\t90%: " << kernelSize90 << "\n\t95%: "
                                                  << kernelSize95 << "\n\t99%: " << kernelSize99);
    });

    kernelRadius_.onChange([&]() { kernelSize_.set(kernelRadius_ * 2 + 1); });
    kernelSize_.setSemantics(PropertySemantics::Text);
    kernelSize_.setReadOnly(true);

    boundaryHandling_.addOption("kernelCrop", "Kernal Crop", BoundaryHandling::KernelCrop);
    boundaryHandling_.addOption("zeroPad", "Zero Pad", BoundaryHandling::ZeroPad);
    boundaryHandling_.addOption("repear", "Repeat", BoundaryHandling::Repeat);
    boundaryHandling_.addOption("mirror", "Mirrorp", BoundaryHandling::Mirror);
    boundaryHandling_.setSelectedIndex(0);
    boundaryHandling_.setCurrentStateAsDefault();

	addProperty(timer_);
}

void GaussianSmoothing::process() {

    performanceTimer_.Reset();

    const auto inputSequence = inport_.getData();

    if (!inputSequence) return;

    auto outputSequence = std::make_shared<VolumeSequence>(inputSequence->size());

    // Set up the volumes by copying the input
    for (size_t i = 0; i < inputSequence->size(); i++) {
        outputSequence->at(i) = std::make_shared<Volume>(*inputSequence->at(i));
    }

    for (size_t iter = 0; iter < iterations_.get(); iter++) {
        for (size_t i = 0; i < inputSequence->size(); i++) {
            const Volume& inputVolume = *(outputSequence->at(i));
            auto outputVolume =
                std::make_shared<Volume>(inputVolume.getDimensions(), inputVolume.getDataFormat());
            GaussianSmoothing::smoothSingleVolume(
                inputVolume, static_cast<int>(kernelRadius_.get()), sigma_.get(),
                boundaryHandling_.get(), *outputVolume);
            // Next input is this output
            outputSequence->at(i) = outputVolume;
        }
        if (smoothOverSequence_.get()) {
            GaussianSmoothing::smoothVolumeSequence(static_cast<int>(kernelRadius_.get()),
                                                    sigma_.get(), boundaryHandling_.get(),
                                                    *outputSequence);
            LogProcessorInfo(
                outputSequence->at(0)->getRepresentation<VolumeRAM>()->getAsDouble({0, 0, 0}));
        }
    }

    // Copy meta data from the input
    for (size_t i = 0; i < inputSequence->size(); i++) {
        auto outputVolume = outputSequence->at(i);
        const Volume& inputVolume = *(inputSequence->at(i));
        outputVolume->setBasis(inputVolume.getBasis());
        outputVolume->setOffset(inputVolume.getOffset());
        outputVolume->dataMap_.dataRange = inputVolume.dataMap_.dataRange;
        outputVolume->dataMap_.valueRange = inputVolume.dataMap_.valueRange;
        outputVolume->dataMap_.valueUnit = inputVolume.dataMap_.valueUnit;
        outputVolume->copyMetaDataFrom(inputVolume);
    }

    outport_.setData(outputSequence);
    timer_.set(performanceTimer_.ElapsedTimeAndReset());
}

void GaussianSmoothing::fillKernel(const int kernelRadius, const float sigma,
                                   std::vector<double>& kernel) {
    // 1D Kernel with given kernelRadius
    kernel.resize(kernelRadius * 2 + 1);
    double kernelSum = 0.0;
    // Compute Gaussian that is centered at kernel[r]
    for (int k = -kernelRadius; k <= kernelRadius; k++) {
        kernel[k + kernelRadius] = 1.0 / (sigma * sqrt(2 * M_PI)) * exp(-0.5 * pow(k / sigma, 2));
        kernelSum += kernel[k + kernelRadius];
    }
    // Normalize so that elements sum to 1
    for (int k = -kernelRadius; k <= kernelRadius; k++) {
        kernel[k + kernelRadius] /= kernelSum;
    }
}

void GaussianSmoothing::smoothSingleVolume(const Volume& vol, const int kernelRadius,
                                           const float sigma,
                                           const BoundaryHandling boundaryHandling,
                                           Volume& newVol) {

    size3_t dims = vol.getDimensions();
    const auto vr = vol.getRepresentation<VolumeRAM>();

    if (newVol.getDimensions() != dims) {
        newVol = Volume(dims, vol.getDataFormat());
    }
    auto nvr = newVol.getEditableRepresentation<VolumeRAM>();

    Volume intermediateVol(dims, vol.getDataFormat());
    auto ivr = intermediateVol.getEditableRepresentation<VolumeRAM>();

    std::vector<double> kernel(kernelRadius * 2 + 1);
    fillKernel(kernelRadius, sigma, kernel);

    auto smoothInDim = [&](const size3_t idx, const size_t d, const VolumeRAM* vrRead,
                           VolumeRAM* vrWrite) {
        double value = 0.0;
        double kernelSum = 0.0;
        for (int k = -kernelRadius; k <= kernelRadius; k++) {
            int currX = static_cast<int>(idx.x) + (d == 0) * k;
            int currY = static_cast<int>(idx.y) + (d == 1) * k;
            int currZ = static_cast<int>(idx.z) + (d == 2) * k;
            // Inside
            if (currX >= 0 && currX < dims.x && currY >= 0 && currY < dims.y && currZ >= 0 &&
                currZ < dims.z) {
                value += kernel[k + kernelRadius] * vrRead->getAsDouble({currX, currY, currZ});
                kernelSum += kernel[k + kernelRadius];
            } else {
                switch (boundaryHandling) {
                    case BoundaryHandling::Repeat:
                        currX = std::max(std::min(0, currX), static_cast<int>(dims.x) - 1);
                        currY = std::max(std::min(0, currY), static_cast<int>(dims.y) - 1);
                        currZ = std::max(std::min(0, currZ), static_cast<int>(dims.z) - 1);
                        value +=
                            kernel[k + kernelRadius] * vrRead->getAsDouble({currX, currY, currZ});
                        kernelSum += kernel[k + kernelRadius];
                        break;
                    case BoundaryHandling::ZeroPad:
                        // value stays unchanged (+0)
                        kernelSum += kernel[k + kernelRadius];
                        break;
                    case BoundaryHandling::Mirror:
                        // (dims.x - 1) - (currX - dims.x)
                        if (currX > dims.x - 1) currX = 2 * static_cast<int>(dims.x) - currX - 1;
                        if (currX < 0) currX = -currX + 1;
                        // (dims.y - 1) - (currY - dims.Y)
                        if (currY > dims.y - 1) currY = 2 * static_cast<int>(dims.y) - currY - 1;
                        if (currY < 0) currY = -currY + 1;
                        // (dims.z - 1) - (currZ - dims.z)
                        if (currZ > dims.z - 1) currZ = 2 * static_cast<int>(dims.z) - currZ - 1;
                        if (currZ < 0) currZ = -currZ + 1;
                        value +=
                            kernel[k + kernelRadius] * vrRead->getAsDouble({currX, currY, currZ});
                        kernelSum += kernel[k + kernelRadius];
                        break;
                    default:
                        break;
                }
            }
        }
        vrWrite->setFromDouble(idx, value / kernelSum);
    };

    size3_t idx;
    // Smoothing in x, set values in nvr, get values from vr
    for (idx.z = 0; idx.z < dims.z; idx.z++) {
        for (idx.y = 0; idx.y < dims.y; idx.y++) {
            for (idx.x = 0; idx.x < dims.x; idx.x++) {
                smoothInDim(idx, 0, vr, nvr);
            }
        }
    }

    // smoothing in y, set values in ivr, get values from nvr
    for (idx.z = 0; idx.z < dims.z; idx.z++) {
        for (idx.y = 0; idx.y < dims.y; idx.y++) {
            for (idx.x = 0; idx.x < dims.x; idx.x++) {
                smoothInDim(idx, 1, nvr, ivr);
            }
        }
    }

    // smoothing in z, set values in nvr, get values from ivr
    for (idx.z = 0; idx.z < dims.z; idx.z++) {
        for (idx.y = 0; idx.y < dims.y; idx.y++) {
            for (idx.x = 0; idx.x < dims.x; idx.x++) {
                smoothInDim(idx, 2, ivr, nvr);
            }
        }
    }
}

void GaussianSmoothing::smoothVolumeSequence(const int kernelRadius, const float sigma,
                                             const BoundaryHandling boundaryHandling,
                                             VolumeSequence& seq) {
    if (seq.size() < 1) return;
    size3_t dims = seq.at(0)->getDimensions();
    auto dataFormat = seq.at(0)->getDataFormat();
    size_t numSteps = seq.size();

    std::vector<double> kernel(kernelRadius * 2 + 1);
    fillKernel(kernelRadius, sigma, kernel);

    VolumeSequence outputSequence(numSteps);

    // Create intermediate output
    for (size_t t = 0; t < numSteps; t++) {
        outputSequence.at(t) = std::make_shared<Volume>(dims, dataFormat);
    }

    size3_t idx;
    for (idx.z = 0; idx.z < dims.z; idx.z++) {
        for (idx.y = 0; idx.y < dims.y; idx.y++) {
            for (idx.x = 0; idx.x < dims.x; idx.x++) {
                for (int t = 0; t < numSteps; t++) {
                    double value = 0.0;
                    double kernelSum = 0.0;
                    for (int k = -kernelRadius; k <= kernelRadius; k++) {
                        int currT = t + k;
                        if (boundaryHandling == BoundaryHandling::Repeat) {
                            currT = std::max(std::min(0, t), static_cast<int>(numSteps) - 1);
                        } else if (boundaryHandling == BoundaryHandling::Mirror) {
                            if (currT > numSteps - 1)
                                currT = std::min(0, 2 * static_cast<int>(numSteps) - currT - 1);
                            if (currT < 0)
                                currT = std::max(-currT + 1, static_cast<int>(numSteps) - 1);
                        }

                        // currT has either been adapted to be between 0 and numSteps-1 or will be
                        // skipped here
                        if (currT >= 0 && currT < numSteps) {
                            auto vr = seq.at(currT)->getRepresentation<VolumeRAM>();
                            value += kernel[k + kernelRadius] * vr->getAsDouble(idx);
                            kernelSum += kernel[k + kernelRadius];
                            // For zero padding
                        } else if (boundaryHandling != BoundaryHandling::KernelCrop)
                            kernelSum += kernel[k + kernelRadius];
                    }
                    auto nvr = outputSequence.at(t)->getEditableRepresentation<VolumeRAM>();
                    nvr->setFromDouble(idx, value / kernelSum);
                }
            }
        }
    }

    // Set to output
    for (size_t t = 0; t < numSteps; t++) {
        seq.at(t) = outputSequence.at(t);
    }
}

}  // namespace inviwo
