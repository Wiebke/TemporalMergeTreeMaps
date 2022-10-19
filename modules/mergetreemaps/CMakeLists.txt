ivw_module(MergeTreeMaps)

set(HEADER_FILES
    include/inviwo/mergetreemaps/decisionsreader.h
    include/inviwo/mergetreemaps/mergetreemapsmodule.h
    include/inviwo/mergetreemaps/mergetreemapsmoduledefine.h
    include/inviwo/mergetreemaps/mergetreemaputils.h
    include/inviwo/mergetreemaps/processors/comparelandscapemergetreetooriginal.h
    include/inviwo/mergetreemaps/processors/contourtreeforspacetimetolandscape.h
    include/inviwo/mergetreemaps/processors/contourtreesequenceelementselector.h
    include/inviwo/mergetreemaps/processors/contourtreesequencetolandscape.h
    include/inviwo/mergetreemaps/processors/contourtreesforsequence.h
    include/inviwo/mergetreemaps/processors/contourtreetolandscape.h
    include/inviwo/mergetreemaps/processors/dataframesequenceelementselector.h
    include/inviwo/mergetreemaps/processors/dataframetoimage.h
    include/inviwo/mergetreemaps/processors/decisionssource.h
    include/inviwo/mergetreemaps/processors/decisionswriter.h
    include/inviwo/mergetreemaps/processors/evaluatemergetreemap.h
    include/inviwo/mergetreemaps/processors/generatedatatargetlevelset.h
    include/inviwo/mergetreemaps/processors/generatemovingskewednormals.h
    include/inviwo/mergetreemaps/processors/mergetreemap.h
    include/inviwo/mergetreemaps/processors/mergetreemapdifference.h
    include/inviwo/mergetreemaps/processors/mergetreemapindextospace.h
    include/inviwo/mergetreemaps/processors/mergetreemapsimplification1d.h
    include/inviwo/mergetreemaps/processors/mergetreemaptrackingoverlay.h
    include/inviwo/mergetreemaps/processors/optimizemergetreemap.h
    include/inviwo/mergetreemaps/processors/optimizemergetreemapgreedy.h
    include/inviwo/mergetreemaps/processors/orderingfromspacetimecontourtree.h
    include/inviwo/mergetreemaps/processors/persistencecurvesforsequence.h
    include/inviwo/mergetreemaps/processors/persistencediagramsforsequence.h
    include/inviwo/mergetreemaps/processors/segmentationfromcontourtree.h
    include/inviwo/mergetreemaps/processors/segmentationsfromcontourtrees.h
    include/inviwo/mergetreemaps/processors/topologicalsimplificationforsequence.h
    include/inviwo/mergetreemaps/processors/trackingfromcontourtreeleaves.h
    include/inviwo/mergetreemaps/processors/trackingqualityestimator.h
    include/inviwo/mergetreemaps/processors/treegeneratefromtrackedcontourtrees.h
    include/inviwo/mergetreemaps/processors/triangulationsequenceelementselector.h
    include/inviwo/mergetreemaps/processors/triangulationtovolumeforsequence.h
    include/inviwo/mergetreemaps/processors/volumetotriangulationforsequence.h
    include/inviwo/mergetreemaps/topologysequenceports.h
)
ivw_group("Header Files" ${HEADER_FILES})

set(SOURCE_FILES
    src/decisionsreader.cpp
    src/mergetreemapsmodule.cpp
    src/mergetreemaputils.cpp
    src/processors/comparelandscapemergetreetooriginal.cpp
    src/processors/contourtreeforspacetimetolandscape.cpp
    src/processors/contourtreesequenceelementselector.cpp
    src/processors/contourtreesequencetolandscape.cpp
    src/processors/contourtreesforsequence.cpp
    src/processors/contourtreetolandscape.cpp
    src/processors/dataframesequenceelementselector.cpp
    src/processors/dataframetoimage.cpp
    src/processors/decisionssource.cpp
    src/processors/decisionswriter.cpp
    src/processors/evaluatemergetreemap.cpp
    src/processors/generatedatatargetlevelset.cpp
    src/processors/generatemovingskewednormals.cpp
    src/processors/mergetreemap.cpp
    src/processors/mergetreemapdifference.cpp
    src/processors/mergetreemapindextospace.cpp
    src/processors/mergetreemapsimplification1d.cpp
    src/processors/mergetreemaptrackingoverlay.cpp
    src/processors/optimizemergetreemap.cpp
    src/processors/optimizemergetreemapgreedy.cpp
    src/processors/orderingfromspacetimecontourtree.cpp
    src/processors/persistencecurvesforsequence.cpp
    src/processors/persistencediagramsforsequence.cpp
    src/processors/segmentationfromcontourtree.cpp
    src/processors/segmentationsfromcontourtrees.cpp
    src/processors/topologicalsimplificationforsequence.cpp
    src/processors/trackingfromcontourtreeleaves.cpp
    src/processors/trackingqualityestimator.cpp
    src/processors/treegeneratefromtrackedcontourtrees.cpp
    src/processors/triangulationsequenceelementselector.cpp
    src/processors/triangulationtovolumeforsequence.cpp
    src/processors/volumetotriangulationforsequence.cpp
)
ivw_group("Source Files" ${SOURCE_FILES})

set(SHADER_FILES
	# Add shaders
)
ivw_group("Shader Files" ${SHADER_FILES})

set(TEST_FILES
    #tests/unittests/mergetreemaps-unittest-main.cpp
)
ivw_add_unittest(${TEST_FILES})

ivw_create_module(${SOURCE_FILES} ${HEADER_FILES} ${SHADER_FILES})

# Add shader directory to install package
#ivw_add_to_module_pack(${CMAKE_CURRENT_SOURCE_DIR}/glsl)
