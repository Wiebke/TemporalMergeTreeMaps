/*********************************************************************
 *  Author  : Wiebke Koepp
 *  Init    : Wednesday, October 03, 2018 - 10:36:11
 *
 *  Project : KTH Inviwo Modules
 *
 *  License : Follows the Inviwo BSD license model
 *********************************************************************
 */

#include <inviwo/core/util/filesystem.h>
#include <inviwo/core/util/colorconversion.h>
#include <modules/temporaltreemaps/processors/ntgrenderer.h>
#include <modules/temporaltreemaps/processors/treewriter.h>
#include <modules/temporaltreemaps/temporaltreemapsmodule.h>
#include <modules/webbrowser/interaction/cefinteractionhandler.h>
#include <modules/webbrowser/properties/propertycefsynchronizer.h>
#include <modules/webbrowser/webbrowserclient.h>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

namespace inviwo {
namespace kth {

// The Class Identifier has to be globally unique. Use a reverse DNS naming
// scheme
const ProcessorInfo NTGRenderer::processorInfo_{
    "org.inviwo.NTGRenderer",  // Class identifier
    "NTGRenderer",             // Display name
    "Temporal Tree",           // Category
    CodeState::Experimental,   // Code state
    Tags::None,                // Tags
};

const ProcessorInfo NTGRenderer::getProcessorInfo() const { return processorInfo_; }

NTGRenderer::NTGRenderer(InviwoApplication* app)
    : WebBrowserProcessor(app)
    , inTree("inTree")
    , propExportWeightScale("exportWScale", "Tree Export W Scale", 1.0f, 0.000001f, 1.0f, 0.1f)
    , propLoadFromFile("treeFromFile", "Load from File", false)
    , propTreeString("treeString", "Tree String", "")
    , propXScale("xScale", "X Scale", 1.0f, 0.0f, 5.0f, 0.01f)
    , propYScale("yScale", "Y Scale", 1.0f, 0.0f, 5.0f, 0.01f)
    , propWScale("wScale", "W Scale", 1.0f, 0.0f, 1.0f, 0.01f)
    , propForceClassic("forceClassic", "Classic NTG Layout", false)
    , propUseColorPreset("useColorPreset", "Use Color (Brewer) Preset", true)
    , propColorBrewerScheme("colorBrewerScheme", "Color Brewer Scheme")
    , propHierarchyLevelGroup("HierarchyLevelGroup", "Hierarchy")
    , propNumLevels("NumLevels", "Number of Levels", 0, 0, 10, 1)
    , propColormap("Colormap", "Colormap",
                   TransferFunction({{0.0f, vec4(0.2f, 0.2f, 0.2f, 1.0f)},
                                     {1.0f, vec4(1.0f, 1.0f, 1.0f, 1.0f)}}))
    , propSvgDimensions("svgDimensions", "Output Dimensions")
    , propSvgX("svgX", "Output X", 400, 100, 2000)
    , propSvgY("svgY", "Output Y", 800, 100, 2000)
    , propSvgString("svgString", "SVG String", "")
    , propSvgFilename("svgFileName", "File Name", "")
    , propSaveSvg("saveSvg", "Save SVG")
    , propOverwriteSvg("overWriteSvg", "Overwrite", false) {
    // Ports
    addPort(inTree);

    addProperty(propExportWeightScale);

    addProperty(propLoadFromFile);
    addProperty(propTreeString);
    propTreeString.setReadOnly(true);
    propTreeString.setSemantics(PropertySemantics::TextEditor);

    auto updateTree = [&]() {
        if (!inTree.hasData()) {
            return;
        }
        std::shared_ptr<const TemporalTree> InTree = inTree.getData();

        json jAll =
            TemporalTreeWriter::createJSON(InTree, true, true, true, propExportWeightScale.get());

        if (!propLoadFromFile.get()) {
            std::stringstream ss;
            ss << jAll;
            std::string treeJSON(ss.str());
            propTreeString.set(treeJSON);
        } else {
            // Get filename and open file
            const std::string& Filename =
                InviwoApplication::getPtr()->getModuleByType<TemporalTreeMapsModule>()->getPath(
                    ModulePath::Data) +
                "/webpage/graphs.js";
            std::ofstream outfile;
            outfile.exceptions(std::ofstream::failbit | std::ofstream::badbit);
            try {
                outfile.open(Filename);
            } catch (const std::ofstream::failure& e) {
                LogError("File could not be opened: " << Filename);
                LogError("  Error Code: " << e.code() << "    . " << e.what());
                return;
            }
            // Stream it out as ASCII or Binary
            try {
                outfile << "var graphs = {" << std::endl;
                outfile << "current: ";
                outfile << jAll << std::endl;
                outfile << "}";
            } catch (const std::ofstream::failure& e) {
                LogError("Error during save: " << Filename);
                LogError("  Error Code: " << e.code() << "    . " << e.what());
                return;
            }

            outfile.close();
        }
    };

    inTree.onChange(updateTree);
    propExportWeightScale.onChange(updateTree);
    propLoadFromFile.onChange(updateTree);

    addProperty(propForceClassic);
    addProperty(propXScale);
    addProperty(propYScale);
    addProperty(propWScale);
    addProperty(propUseColorPreset);
    addProperty(propColorBrewerScheme);
    propColorBrewerScheme.visibilityDependsOn(propUseColorPreset,
                                              [](const auto& p) { return p.get(); });
    propColorBrewerScheme.addOption("YlOrRd_6", "6-class YlOrRd", 0);
    propColorBrewerScheme.addOption("YlGnBu_6", "6-class YlGnBu", 1);
    propColorBrewerScheme.addOption("PuBuGn_6", "6-class PuBuGn", 2);
    propColorBrewerScheme.addOption("Blues_3", "3-class blue", 3);
    propColorBrewerScheme.addOption("Greens_3", "3-class green", 4);

    addProperties(propColormap);
    propColormap.visibilityDependsOn(propUseColorPreset, [](const auto& p) { return !p.get(); });
    addProperty(propHierarchyLevelGroup);
    propHierarchyLevelGroup.visibilityDependsOn(propUseColorPreset,
                                                [](const auto& p) { return !p.get(); });
    // propNumLevels.set(1, 0, 10, 1); //gives comilation error, since a >= operator is missing
    propHierarchyLevelGroup.addProperty(propNumLevels);
    size_t numDesiredLevels = 6;
    for (size_t i = 0; i < numDesiredLevels; i++) {
        StringProperty* ppropIsovalue = new StringProperty(
            "color" + std::to_string(i + 1), "Color for Lvl " + std::to_string(i + 1), "");
        propHierarchyLevelGroup.addProperty(ppropIsovalue, true);
        ppropIsovalue->setReadOnly(true);
    }
    propNumLevels.set(1);  // Add one slider

    auto updateColors = [this]() {
        const TransferFunction& Colormap = propColormap.get();
        const size_t nDesiredLevelsCurrent = propNumLevels.get();
        for (int i(0); i < nDesiredLevelsCurrent; i++) {
            vec4 fColor{1.0f};
            if (i < nDesiredLevelsCurrent) {
                const double NormalizedIsovalue =
                    double(i + 1) / double(nDesiredLevelsCurrent - 1 + 1);  // avoid zero

                fColor = Colormap.sample(NormalizedIsovalue);
            }
            auto p =
                propHierarchyLevelGroup.getPropertyByIdentifier("color" + std::to_string(i + 1));
            if (auto prop = dynamic_cast<StringProperty*>(p)) {
                prop->set(color::rgba2hex(fColor));
            }
        }
    };
    propNumLevels.onChange([this, updateColors]() {
        const size_t nDesiredLevels = propNumLevels.get();
        const size_t nCurrentLevels = propHierarchyLevelGroup.size() - 1;

        // Unhide new levels or create them
        for (size_t i(nCurrentLevels); i < nDesiredLevels; i++) {
            StringProperty* ppropIsovalue = new StringProperty(
                "color" + std::to_string(i + 1), "Color for Lvl " + std::to_string(i + 1), "");
            propHierarchyLevelGroup.addProperty(ppropIsovalue, true);
            ppropIsovalue->setReadOnly(true);
        }
        for (size_t i(0); i < nDesiredLevels; i++) {
            propHierarchyLevelGroup[i + 1]->setVisible(true);
        }
        // Hide old levels
        for (size_t i(nCurrentLevels); i > nDesiredLevels; i--) {
            propHierarchyLevelGroup[i]->setVisible(false);
        }
        updateColors();
    });
    propColormap.onChange([this, updateColors]() { updateColors(); });

    addProperty(propSvgDimensions);
    addProperty(propSvgX);
    addProperty(propSvgY);
    propSvgX.setReadOnly(true);
    propSvgY.setReadOnly(true);

    propSvgDimensions.onChange([&]() {
        auto dims = propSvgDimensions.get();
        propSvgX.set(dims.x);
        propSvgY.set(dims.y);
    });

    addProperty(propSvgString);
    propSvgString.setReadOnly(true);
    propSvgString.setSemantics(PropertySemantics::TextEditor);

    addProperty(propSvgFilename);
    addProperty(propSaveSvg);
    addProperty(propOverwriteSvg);
    propSaveSvg.onChange([&]() { saveSvg(); });

    auto path = InviwoApplication::getPtr()->getModuleByType<TemporalTreeMapsModule>()->getPath(
                    ModulePath::Data) +
                "/webpage/index_inviwo.html";
    if (!filesystem::fileExists(path)) {
        throw Exception("Could not find " + path);
    }

    fileName_.set(path);

    // Hide/Remove dynamic properties from the webbrowser we do not need
    removeProperty("addProperty");
    // util::hide(fileName_, sourceType_);
}

void NTGRenderer::saveSvg() {

    // Get filename and open file
    const std::string& Filename = propSvgFilename.get();

    if (filesystem::fileExists(Filename) && !propOverwriteSvg.get()) {
        LogWarn("File already exists: " << Filename);
        return;
    }

    std::ofstream outfile;
    outfile.exceptions(std::ofstream::failbit | std::ofstream::badbit);
    try {
        outfile.open(Filename);
    } catch (const std::ofstream::failure& e) {
        LogError("File could not be opened: " << Filename);
        LogError("  Error Code: " << e.code() << "    . " << e.what());
        return;
    }

    // Stream it out as ASCII or Binary
    try {
        std::string svgString = propSvgString.get();
        std::string openTag = "<svg ";
        auto pos = svgString.find(openTag);
        if (pos != std::string::npos) {
            svgString = svgString.replace(pos, openTag.length(),
                                          "<svg xmlns=\"http://www.w3.org/2000/svg\" ");
        }
        outfile << svgString << std::endl;

    } catch (const std::ofstream::failure& e) {
        LogError("Error during save: " << Filename);
        LogError("  Error Code: " << e.code() << "    . " << e.what());
        return;
    }

    outfile.close();
}

void NTGRenderer::process() { WebBrowserProcessor::process(); }

}  // namespace kth
}  // namespace inviwo
