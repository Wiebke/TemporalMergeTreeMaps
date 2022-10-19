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

#include <inviwo/mergetreemaps/mergetreemapsmoduledefine.h>
#include <inviwo/core/processors/processor.h>
#include <inviwo/dataframe/datastructures/dataframe.h>
#include <inviwo/core/ports/meshport.h>
#include <inviwo/topologytoolkit/properties/topologycolorsproperty.h>
#include <inviwo/core/properties/ordinalproperty.h>
#include <inviwo/core/properties/boolproperty.h>
#include <inviwo/core/properties/optionproperty.h>
#include <inviwo/core/properties/transferfunctionproperty.h>
#include <inviwo/mergetreemaps/topologysequenceports.h>
#include <inviwo/mergetreemaps/mergetreemaputils.h>
#include <modules/tools/performancetimer.h>

namespace inviwo {

/** \docpage{org.inviwo.TrackingQualityEstimator, Tracking Quality Estimator}
 * ![](org.inviwo.TrackingQualityEstimator.png?classIdentifier=org.inviwo.TrackingQualityEstimator)
 * Explanation of how to use the processor.
 *
 * ### Inports
 *   * __<Inport1>__ <description>.
 *
 * ### Outports
 *   * __<Outport1>__ <description>.
 *
 * ### Properties
 *   * __<Prop1>__ <description>.
 *   * __<Prop2>__ <description>
 */
class IVW_MODULE_MERGETREEMAPS_API TrackingQualityEstimator : public Processor {
public:
    TrackingQualityEstimator();
    virtual ~TrackingQualityEstimator() = default;

    virtual void process() override;

    virtual const ProcessorInfo getProcessorInfo() const override;
    static const ProcessorInfo processorInfo_;

private:
    /** Not sure yet if these are needed **/
    ContourTreeSequenceInport treesInport_{"contourTrees"};

    // Needed to traverse segmentation
    TriangulationSequenceInport segmentationsInport_{"segmentations"};

    // Tracks from vertex to vertex, where vertices are critical points
    DataFrameInport trackingInport_{"inputTracks"};

    // Tracks colored by distance
    MeshOutport tracksOutport_{"outputTracks"};

    // Color of the mesh segments
    BoolProperty colorByDistance_;

    // Type of distance to use for color
    TemplateOptionProperty<int> distanceType_;

    // Transfor function for distance
    TransferFunctionProperty tf_;

    // Put out tracks at geometric mean of area instead of critical point
    BoolProperty outputGeometricMeansInMesh_;

    // Accumulated Euclidian distance between critical points per track
    FloatProperty accumulatedCritDistance_;

    // Accumulated Euclidian distance between geometric means per track
    FloatProperty accumulatedGeoMeanDistance_;

    /* Accumulated Non-overlap distance per track
       A/B + B/A = A+B - 2*(A \cap B)
       A \cup B = A+B - 2*(A \cap B) */
    FloatProperty accumulatedNonOverlapDistance_;

    /* Performance Timer  */
    kth::PerformanceTimer performanceTimer_;

    /* Property for the timer */
    FloatProperty timer_;
};

}  // namespace inviwo
