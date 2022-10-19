#--------------------------------------------------------------------
# Inviwo KTH Tools Module
ivw_module(Tools)

#--------------------------------------------------------------------
# Add header files
set(HEADER_FILES
    include/modules/tools/amirameshvolumereader.h
    include/modules/tools/filelistproperty.h
    include/modules/tools/gaussiansmoothing.h
    include/modules/tools/iotexthelpers.h
    include/modules/tools/performancetimer.h
    include/modules/tools/setminmaxdatamap.h
    include/modules/tools/simplelrucache.h
    include/modules/tools/toolsmodule.h
    include/modules/tools/toolsmoduledefine.h
    include/modules/tools/volumefromslicesequence.h
    include/modules/tools/volumerepeatertosequence.h
    include/modules/tools/volumesequenceclampvaluerange.h
    include/modules/tools/volumesequencefromslices.h
    include/modules/tools/volumeseriesfromslices.h
    include/modules/tools/volumeshiftandreverse.h
    include/modules/tools/volumesourceseries.h
    include/modules/tools/volumesourceseriesdata.h
)
ivw_group("Header Files" ${HEADER_FILES})

#--------------------------------------------------------------------
# Add source files
set(SOURCE_FILES
    src/amirameshvolumereader.cpp
    src/filelistproperty.cpp
    src/gaussiansmoothing.cpp
    src/iotexthelpers.cpp
    src/performancetimer.cpp
    src/setminmaxdatamap.cpp
    src/toolsmodule.cpp
    src/volumefromslicesequence.cpp
    src/volumerepeatertosequence.cpp
    src/volumesequenceclampvaluerange.cpp
    src/volumesequencefromslices.cpp
    src/volumeseriesfromslices.cpp
    src/volumeshiftandreverse.cpp
    src/volumesourceseries.cpp
    src/volumesourceseriesdata.cpp
)
ivw_group("Source Files" ${SOURCE_FILES})

#--------------------------------------------------------------------
# Create module
ivw_create_module(${SOURCE_FILES} ${HEADER_FILES})
