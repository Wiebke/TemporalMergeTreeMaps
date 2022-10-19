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

#pragma once

#include <modules/tools/toolsmoduledefine.h>
#include <inviwo/core/processors/processor.h>
#include <inviwo/core/properties/ordinalproperty.h>
#include <inviwo/core/ports/imageport.h>
#include <modules/tools/performancetimer.h>

namespace inviwo {

/** \docpage{org.inviwo.GaussianSmoothing, Gaussian Smoothing}
 * ![](org.inviwo.GaussianSmoothing.png?classIdentifier=org.inviwo.GaussianSmoothing)
 * Applies Gaussian smoothing to a sequence of volumes. Volumes can represent both 2D or 3D data.
 *
 * ### Inports
 *   * __inport__ Input volume sequence.
 *
 * ### Outports
 *   * __outport__ Output volume sequence.
 *
 * ### Properties
 *   * __kernelsize__ Kernel size in each dimension.
 *   * __sigma__ Sigma of the Gaussian used to generate the kernel.
 *   * __iterations__ Number of times smoothing is applied.
 */
class IVW_MODULE_TOOLS_API GaussianSmoothing : public Processor {
public:
    GaussianSmoothing();
    virtual ~GaussianSmoothing() = default;

    enum class BoundaryHandling : unsigned char {
        KernelCrop = 0,  // Renormalize the kernel based on number of values used
        ZeroPad,         // Set everything outside to zero
        Repeat,          // Repeat border value
        Mirror
    };

    virtual void process() override;

    virtual const ProcessorInfo getProcessorInfo() const override;
    static const ProcessorInfo processorInfo_;

    static void GaussianSmoothing::fillKernel(const int kernelRadius, const float sigma,
                                              std::vector<double>& kernel);

    static void GaussianSmoothing::smoothSingleVolume(const Volume& vol, const int kernelRadius,
                                                      const float sigma,
                                                      const BoundaryHandling boundaryHandling,
                                                      Volume& newVol);

    static void GaussianSmoothing::smoothVolumeSequence(const int kernelRadius, const float sigma,
                                                        const BoundaryHandling boundaryHandling,
                                                        VolumeSequence& seq);

private:
    VolumeSequenceInport inport_;
    VolumeSequenceOutport outport_;

    /** Kernel radius in each dimension **/
    IntSizeTProperty kernelRadius_;

    /** Total kernel size to be used for the current volume **/
    IntSizeTProperty kernelSize_;

    /** Sigma used for generating the Gaussian Kernel **/
    FloatProperty sigma_;

    /** Type of boundary handling **/
    TemplateOptionProperty<BoundaryHandling> boundaryHandling_;

    /** Sigma used for generating the Gaussian Kernel **/
    BoolProperty smoothOverSequence_;

    /** Number of times to repeat the smoothing **/
    IntSizeTProperty iterations_;

    /* Performance Timer  */
    kth::PerformanceTimer performanceTimer_;

    /* Property for the timer */
    FloatProperty timer_;
};

}  // namespace inviwo
