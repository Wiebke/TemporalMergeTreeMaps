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
#include <inviwo/core/processors/poolprocessor.h>
#include <inviwo/core/properties/ordinalproperty.h>
#include <inviwo/core/properties/boolproperty.h>
#include <inviwo/core/properties/stringproperty.h>
#include <inviwo/core/properties/listproperty.h>
#include <inviwo/topologytoolkit/ports/triangulationdataport.h>
#include <inviwo/mergetreemaps/topologysequenceports.h>
#include <inviwo/core/properties/transferfunctionproperty.h>
#include <modules/tools/performancetimer.h>
#include <random>

namespace inviwo {

/**
 * \ingroup properties
 * A CompositeProperty holding information about a component (either tracked leaf or superArc) and
 * its timestep
 */
class IVW_MODULE_MERGETREEMAPS_API ComponentProperty : public CompositeProperty {
public:
    virtual std::string getClassIdentifier() const override;
    static const std::string classIdentifier;
    ComponentProperty(std::string identifier, std::string displayName,
                      InvalidationLevel invalidationLevel = InvalidationLevel::InvalidResources,
                      PropertySemantics semantics = PropertySemantics::Default);
    ComponentProperty(const ComponentProperty& rhs);
    virtual ComponentProperty* clone() const override;
    virtual ~ComponentProperty() = default;

    BoolProperty syncWithVolSliceSample_;
    FloatVec4Property volSliceSample_;
    IntSizeTProperty componentId_;
    IntSizeTProperty timeStep_;

private:
    auto props();
};

/** \docpage{org.inviwo.SegmentationsFromContourTrees, Segmentations From Contour Trees}
 * ![](org.inviwo.SegmentationsFromContourTrees.png?classIdentifier=org.inviwo.SegmentationsFromContourTrees)
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
class IVW_MODULE_MERGETREEMAPS_API SegmentationsFromContourTrees : public PoolProcessor,
                                                                   public PropertyOwnerObserver {
public:
    SegmentationsFromContourTrees();
    virtual ~SegmentationsFromContourTrees() = default;

    void generateColorMap(const int minComponentId, const int maxComponentId);
    virtual void process() override;

    virtual const ProcessorInfo getProcessorInfo() const override;
    static const ProcessorInfo processorInfo_;

    virtual void onDidAddProperty(Property* property, size_t index) override;
    virtual void onDidRemoveProperty(Property* property, size_t index) override;

private:
    ContourTreeSequenceInport treesInport_{"contourTrees"};
    TriangulationSequenceInport inport_{"triangulations"};
    // Tracks from vertex to vertex, where vertices are critical points
    DataFrameInport trackingInport_{"inputTracks"};

    TriangulationSequenceOutport outport_{"segmentations"};

    TriangulationSequence segmentations_;

    /* Type of tree: */
    StringProperty treeType_;

    BoolProperty leafOnly_;

    BoolProperty trackingInsteadOfSUperArcs_;

    BoolProperty selectedComponentsOnly_;
    ListProperty selectedComponentIds_;

    FloatVec4Property colorNonLeaf_;
    FloatVec4Property colorNonTracked_;

    IntProperty colorSeed_;

    TransferFunctionProperty segmentationsColorMap_;

    /* Performance Timer  */
    kth::PerformanceTimer performanceTimer_;

    /* Property for the timer */
    FloatProperty timer_;

    mutable std::mt19937 randomGen;
    mutable std::uniform_real_distribution<float> randomDis;
};

}  // namespace inviwo
