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

#include <inviwo/mergetreemaps/mergetreemapsmodule.h>
#include <inviwo/mergetreemaps/processors/comparelandscapemergetreetooriginal.h>
#include <inviwo/mergetreemaps/processors/contourtreeforspacetimetolandscape.h>
#include <inviwo/mergetreemaps/processors/contourtreesforsequence.h>
#include <inviwo/mergetreemaps/processors/contourtreesequenceelementselector.h>
#include <inviwo/mergetreemaps/processors/contourtreesequencetolandscape.h>
#include <inviwo/mergetreemaps/processors/contourtreetolandscape.h>
#include <inviwo/mergetreemaps/processors/dataframesequenceelementselector.h>
#include <inviwo/mergetreemaps/processors/dataframetoimage.h>
#include <inviwo/mergetreemaps/processors/decisionssource.h>
#include <inviwo/mergetreemaps/processors/decisionswriter.h>
#include <inviwo/mergetreemaps/processors/evaluatemergetreemap.h>
#include <inviwo/mergetreemaps/processors/generatedatatargetlevelset.h>
#include <inviwo/mergetreemaps/processors/generatemovingskewednormals.h>
#include <inviwo/mergetreemaps/processors/mergetreemap.h>
#include <inviwo/mergetreemaps/processors/mergetreemapdifference.h>
#include <inviwo/mergetreemaps/processors/mergetreemapindextospace.h>
#include <inviwo/mergetreemaps/processors/mergetreemapsimplification1d.h>
#include <inviwo/mergetreemaps/processors/mergetreemaptrackingoverlay.h>
#include <inviwo/mergetreemaps/processors/optimizemergetreemap.h>
#include <inviwo/mergetreemaps/processors/optimizemergetreemapgreedy.h>
#include <inviwo/mergetreemaps/processors/orderingfromspacetimecontourtree.h>
#include <inviwo/mergetreemaps/processors/persistencecurvesforsequence.h>
#include <inviwo/mergetreemaps/processors/persistencediagramsforsequence.h>
#include <inviwo/mergetreemaps/processors/segmentationfromcontourtree.h>
#include <inviwo/mergetreemaps/processors/segmentationsfromcontourtrees.h>
#include <inviwo/mergetreemaps/processors/topologicalsimplificationforsequence.h>
#include <inviwo/mergetreemaps/processors/trackingfromcontourtreeleaves.h>
#include <inviwo/mergetreemaps/processors/trackingqualityestimator.h>
#include <inviwo/mergetreemaps/processors/treegeneratefromtrackedcontourtrees.h>
#include <inviwo/mergetreemaps/processors/triangulationsequenceelementselector.h>
#include <inviwo/mergetreemaps/processors/triangulationtovolumeforsequence.h>
#include <inviwo/mergetreemaps/processors/volumetotriangulationforsequence.h>
#include <inviwo/mergetreemaps/topologysequenceports.h>
#include <inviwo/mergetreemaps/decisionsreader.h>

namespace inviwo {

MergeTreeMapsModule::MergeTreeMapsModule(InviwoApplication* app)
    : InviwoModule(app, "MergeTreeMaps")
    , pythonFolderObserver_{app, getPath() + "/python/processors", *this} {
    // Add a directory to the search path of the Shadermanager
    // ShaderManager::getPtr()->addShaderSearchPath(getPath(ModulePath::GLSL));

    // Register objects that can be shared with the rest of inviwo here:

    // Processors
    registerProcessor<CompareLandscapeMergeTreeToOriginal>();
    registerProcessor<ContourTreeForSpaceTimeToLandscape>();
    registerProcessor<ContourTreesForSequence>();
    registerProcessor<ContourTreeSequenceElementSelector>();
    registerProcessor<ContourTreeSequenceToLandscape>();
    registerProcessor<ContourTreeToLandscape>();
    registerProcessor<DataFrameSequenceElementSelector>();
    registerProcessor<DataFrameToImage>();
    registerProcessor<DecisionsSource>();
    registerProcessor<DecisionsWriter>();
    registerProcessor<EvaluateMergeTreeMap>();
    registerProcessor<GenerateDataTargetLevelSet>();
    registerProcessor<GenerateMovingSkewedNormals>();
    registerProcessor<MergeTreeMap>();
    registerProcessor<MergeTreeMapDifference>();
    registerProcessor<MergeTreeMapIndexToSpace>();
    registerProcessor<MergeTreeMapSimplification1D>();
    registerProcessor<MergeTreeMapTrackingOverlay>();
    registerProcessor<OptimizeMergeTreeMap>();
    registerProcessor<OptimizeMergeTreeMapGreedy>();
    registerProcessor<OrderingFromSpaceTimeContourTree>();
    registerProcessor<PersistenceCurvesForSequence>();
    registerProcessor<PersistenceDiagramsForSequence>();
    registerProcessor<SegmentationFromContourTree>();
    registerProcessor<SegmentationsFromContourTrees>();
    registerProcessor<TopologicalSimplificationForSequence>();
    registerProcessor<TrackingFromContourTreeLeaves>();
    registerProcessor<TrackingQualityEstimator>();
    registerProcessor<TreeGenerateFromTrackedContourTrees>();
    registerProcessor<TriangulationSequenceElementSelector>();
    registerProcessor<TriangulationToVolumeForSequence>();
    registerProcessor<VolumeToTriangulationForSequence>();

    // Properties
    registerProperty<SkewedNormalTrackProperty>();
    registerProperty<SkewedNormalProperty>();
    registerProperty<ComponentProperty>();

    // Readers and writes
	registerDataReader(std::make_unique<DecisionsReader>());
    // registerDataWriter(std::make_unique<MergeTreeMapsWriter>());

    // Data converters
    // registerRepresentationConverter(std::make_unique<MergeTreeMapsDisk2RAMConverter>());

    // Ports
    // using registerDefaults registers inport, outport and important here: Composite ports
    registerDefaultsForDataType<ContourTreeSequence>();
    registerDefaultsForDataType<TriangulationSequence>();
    registerDefaultsForDataType<PersistenceDiagramSequence>();
    registerDefaultsForDataType<DataFrameSequence>();
    registerDefaultsForDataType<LandscapeDecisions>();

    // PropertyWidgets
    // registerPropertyWidget<MergeTreeMapsPropertyWidget, MergeTreeMapsProperty>("Default");

    // Dialogs
    // registerDialog<MergeTreeMapsDialog>(MergeTreeMapsOutport);

    // Other things
    // registerCapabilities(std::make_unique<MergeTreeMapsCapabilities>());
    // registerSettings(std::make_unique<MergeTreeMapsSettings>());
    // registerMetaData(std::make_unique<MergeTreeMapsMetaData>());
    // registerPortInspector("MergeTreeMapsOutport", "path/workspace.inv");
    // registerProcessorWidget(std::string processorClassName, std::unique_ptr<ProcessorWidget>
    // processorWidget); registerDrawer(util::make_unique_ptr<MergeTreeMapsDrawer>());
}

}  // namespace inviwo
