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

#include <inviwo/mergetreemaps/processors/segmentationsfromcontourtrees.h>

namespace inviwo {

const std::string ComponentProperty::classIdentifier = "org.inviwo.ComponentProperty";
std::string ComponentProperty::getClassIdentifier() const { return classIdentifier; }

auto ComponentProperty::props() {
    return std::tie(syncWithVolSliceSample_, volSliceSample_, componentId_, timeStep_);
}

ComponentProperty::ComponentProperty(std::string identifier, std::string displayName,
                                     InvalidationLevel invalidationLevel,
                                     PropertySemantics semantics)
    : CompositeProperty(identifier, displayName, invalidationLevel, semantics)
    , syncWithVolSliceSample_("syncWithVolSlice", "Sync with Sample", false)
    , volSliceSample_("volumeSample", "Volume Slice Sample", vec4(0.0f),
                      vec4(std::numeric_limits<float>::lowest()),
                      vec4(std::numeric_limits<float>::max()), vec4(0.001f),
                      InvalidationLevel::Valid, PropertySemantics::Text)
    , componentId_("componentId", "Component Id", 0, 0, std::numeric_limits<size_t>::max(), 1,
                   InvalidationLevel::InvalidOutput, PropertySemantics::Text)
    , timeStep_("timeStep", "TimeStep (1-based!)", 1, 1, std::numeric_limits<size_t>::max(), 1,
                InvalidationLevel::InvalidOutput, PropertySemantics::Text) {

    util::for_each_in_tuple(
        [&](auto& e) {
            e.setSerializationMode(PropertySerializationMode::All);
            this->addProperty(e);
        },
        props());

    volSliceSample_.onChange([&]() {
        auto sample = volSliceSample_.get();
        if (syncWithVolSliceSample_.get()) componentId_.set(sample.x);
    });
}

ComponentProperty::ComponentProperty(const ComponentProperty& rhs)
    : CompositeProperty(rhs)
    , syncWithVolSliceSample_(rhs.syncWithVolSliceSample_)
    , volSliceSample_(rhs.volSliceSample_)
    , componentId_(rhs.componentId_)
    , timeStep_(rhs.timeStep_) {

    util::for_each_in_tuple([&](auto& e) { this->addProperty(e); }, props());

    volSliceSample_.onChange([&]() {
        auto sample = volSliceSample_.get();
        if (syncWithVolSliceSample_.get()) componentId_.set(sample.x);
    });
}

ComponentProperty* ComponentProperty::clone() const { return new ComponentProperty(*this); }

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo SegmentationsFromContourTrees::processorInfo_{
    "org.inviwo.SegmentationsFromContourTrees",  // Class identifier
    "Segmentations From Contour Trees",          // Display name
    "Topology",                                  // Category
    CodeState::Experimental,                     // Code state
    Tags::None,                                  // Tags
};
const ProcessorInfo SegmentationsFromContourTrees::getProcessorInfo() const {
    return processorInfo_;
}

SegmentationsFromContourTrees::SegmentationsFromContourTrees()
    : PoolProcessor()
    , treeType_("treeType", "Tree Type", "", InvalidationLevel::Valid)
    , leafOnly_("leafOnly", "Leaf Only", false)
    , trackingInsteadOfSUperArcs_("useTracking", "Tracking IDs", false)
    , selectedComponentsOnly_("selectedComponentsOnly", "Select IDs", false)
    , selectedComponentIds_("components", "Selected Components",
                            std::make_unique<ComponentProperty>("component", "Component"))
    , colorNonLeaf_("nonLeafColor", "Non-Leaf Color", vec4(1.0f), vec4(0.0f), vec4(1.0f),
                    vec4(0.1f), InvalidationLevel::InvalidOutput, PropertySemantics::Color)
    , colorNonTracked_("nonTrackedColor", "Non-Tracked Color", vec4(1.0f), vec4(0.0f), vec4(1.0f),
                       vec4(0.1f), InvalidationLevel::InvalidOutput, PropertySemantics::Color)
    , colorSeed_("colorSeed", "Random Seed", 0, 0, RAND_MAX + 1, 1)
    , segmentationsColorMap_("segmentationColors", "Segmentation Colors")
    , timer_("timer", "Eval Time (s)", 0.f, 0.f, std::numeric_limits<float>::max(), 0.001f,
             InvalidationLevel::Valid, PropertySemantics::Text) {
    addPort(inport_);
    addPort(treesInport_);
    addPort(trackingInport_);
    trackingInport_.setOptional(true);
    addPort(outport_);

    addProperty(treeType_);
    treeType_.setReadOnly(true);
    treesInport_.onChange([this]() {
        treeType_.set([this]() -> std::string {
            if (treesInport_.hasData() && treesInport_.getData()->size() >= 1) {
                return toString(treesInport_.getData()->at(0)->type);
            } else {
                return "";
            }
        }());
    });
    addProperty(leafOnly_);
    addProperty(trackingInsteadOfSUperArcs_);
    addProperties(selectedComponentsOnly_, selectedComponentIds_);
    selectedComponentIds_.PropertyOwnerObservable::addObserver(this);
    addProperties(colorNonLeaf_, colorNonTracked_, colorSeed_, segmentationsColorMap_);
    segmentationsColorMap_.setInvalidationLevel(InvalidationLevel::Valid);

    addProperty(timer_);
}

void SegmentationsFromContourTrees::generateColorMap(const int minComponentId,
                                                     const int maxComponentId) {
    auto& tf = segmentationsColorMap_.get();
    tf.clear();
    // component Ids between 0 and maxComponentId (where maxComponentId is actual components + 1)
    auto addPoint = [&](double pos, const vec4& color) {
        tf.add(std::clamp(pos, 0.0, 1.0), color);
    };
    auto addPointAlmost = [&](double pos, const vec4& color) {
        addPoint(pos - 100.0 * std::numeric_limits<double>::epsilon(), color);
    };
    auto getRandomColor = [&](int index) -> vec4 {
        randomGen.seed(static_cast<std::mt19937::result_type>(colorSeed_.get() + index));
        auto color = vec4(randomDis(randomGen), randomDis(randomGen), randomDis(randomGen), 1.0f);
        while (glm::distance(color, vec4(0.f, 0.f, 0.f, 1.f)) < 0.1f ||
               glm::distance(color, vec4(1, 1, 1, 1)) < 0.1f) {
            color = vec4(randomDis(randomGen), randomDis(randomGen), randomDis(randomGen), 1.0f);
        }
        return color;
    };

    // (range of values is from minComponentId to maxComponentId + 2, in between we have
    // (x - minComponentId)/(maxComponentId+2-minComponentsId)
    // Normalized ranged is 0.0 to 1.0
    const double dt = 1.0 / (maxComponentId + 2 - minComponentId);
    double start = -0.5 * dt;
    int i = 0;
    // number of colors between min and max is max-min + 1 (inclusive int interval)
    for (i; i < maxComponentId - minComponentId + 1; i++) {
        auto color = getRandomColor(i);
        addPoint(start + i * dt, color);
        addPointAlmost(start + (i + 1) * dt, color);
    }

    addPoint(start + i * dt, colorNonTracked_.get());
    addPointAlmost(start + (i + 1) * dt, colorNonTracked_.get());
    i++;
    addPoint(start + i * dt, colorNonLeaf_.get());
    addPointAlmost(start + (i + 1) * dt, colorNonLeaf_.get());
}

void SegmentationsFromContourTrees::onDidAddProperty(Property* property, size_t index) {
    selectedComponentIds_.setModified();
    if (selectedComponentsOnly_) this->invalidate(InvalidationLevel::InvalidOutput);
}

void SegmentationsFromContourTrees::onDidRemoveProperty(Property* property, size_t index) {
    selectedComponentIds_.setModified();
    if (selectedComponentsOnly_) this->invalidate(InvalidationLevel::InvalidOutput);
}

void SegmentationsFromContourTrees::process() {
    performanceTimer_.Reset();
    const auto scalarData = inport_.getData();
    const auto contourTrees = treesInport_.getData();

    if (scalarData->size() != contourTrees->size()) {
        LogProcessorError("Number of tringulations and trees does not match.");
        return;
    }

    using Result = std::shared_ptr<topology::TriangulationData>;

    const auto computeSegmentationJob = [](const topology::TriangulationData& triangulation,
                                           const topology::ContourTreeData& contourTree,
                                           const std::map<ttk::SimplexId, int>& componentMap,
                                           const int maxComponentId, const bool leafOnly,
                                           const bool useTrackings,
                                           const std::vector<int>& selectedComponents,
                                           const bool selectedComponentsOnly) {
        return [triangulation, contourTree, componentMap, maxComponentId, leafOnly, useTrackings,
                selectedComponents,
                selectedComponentsOnly](pool::Stop stop, pool::Progress progress) -> Result {
            // Lambda for diagram computation based on scalar value buffer
            auto computeSegmentation = [triangulation, contourTree, componentMap, maxComponentId,
                                        leafOnly, useTrackings, selectedComponents,
                                        selectedComponentsOnly, stop,
                                        progress](const auto buffer) -> Result {
                if (stop) return nullptr;
                using ValueType = util::PrecisionValueType<decltype(buffer)>;

                progress(0.1f);

                // create a copy of the data values, which we will use to fill the segmentation
                // auto segmentationValues = //buffer->getDataContainer();
                std::vector<float> segmentationValues(buffer->getDataContainer().size());
                std::vector<int> offsets(triangulation.getOffsets());

                // In the tree: vertexId = node->getVertexId();
                auto tree = contourTree.getTree();
                ttk::SimplexId vertexId;

                // Iterate over all normal nodes in the tree associated with super arcs
                size_t numSuperArcs = tree->getNumberOfSuperArcs();
                for (ttk::ftm::idSuperArc superArcId = 0; superArcId < numSuperArcs; superArcId++) {

                    auto superArc = tree->getSuperArc(superArcId);
                    auto downNodeId = superArc->getDownNodeId();
                    auto node = tree->getNode(downNodeId);
                    auto numDown = node->getNumberOfDownSuperArcs();
                    float currentId = static_cast<float>(superArcId);
                    // Mark superarcs that are not a leaf
                    if (leafOnly) {
                        if (numDown != 0) currentId = maxComponentId + 2;
                        // Mark leafs according to track or mark as untracked
                        else if (useTrackings) {
                            vertexId = node->getVertexId();
                            auto vertexComponentPair = componentMap.find(vertexId);
                            if (vertexComponentPair != componentMap.end())
                                currentId = vertexComponentPair->second;
                            else
                                currentId = maxComponentId + 1;
                        }
                    }
                    // Filter according to selected
                    if (selectedComponentsOnly && currentId <= maxComponentId) {
                        if (std::find(selectedComponents.begin(), selectedComponents.end(),
                                      currentId) == selectedComponents.end()) {
                            currentId = maxComponentId + 1;
                        }
                    }

                    size_t arcSize = superArc->regionSize();
                    for (size_t i = 0; i < arcSize; i++) {
                        vertexId = superArc->getRegularNodeId(i);
                        segmentationValues[vertexId] = currentId;
                    }
                }
                progress(0.8f);

                // Only critical points are missing now

                size_t numNodes = tree->getNumberOfNodes();
                ttk::ftm::idSuperArc superArcId;

                for (ttk::ftm::idNode nodeId = 0; nodeId < numNodes; ++nodeId) {
                    auto node = tree->getNode(nodeId);
                    auto numDown = node->getNumberOfDownSuperArcs();

                    if (node->getNumberOfUpSuperArcs() >= 1) {
                        // Assign leaf and saddles to their up superArcId (there will only be
                        // one)
                        superArcId = node->getUpSuperArcId(0);
                    } else {
                        // This is the case *only* for the root node, assign it to the first one
                        superArcId = node->getDownSuperArcId(0);
                    }
                    // Skip superarcs that are not a leaf
                    float currentId = static_cast<float>(superArcId);
                    if (leafOnly) {
                        if (numDown != 0)
                            currentId = maxComponentId + 2;
                        else if (useTrackings) {
                            vertexId = node->getVertexId();
                            auto vertexComponentPair = componentMap.find(vertexId);
                            if (vertexComponentPair != componentMap.end())
                                currentId = vertexComponentPair->second;
                            else
                                currentId = maxComponentId + 1;
                        }
                    }
                    // Filter according to selected
                    if (selectedComponentsOnly && currentId < maxComponentId) {
                        if (std::find(selectedComponents.begin(), selectedComponents.end(),
                                      currentId) == selectedComponents.end()) {
                            currentId = maxComponentId + 1;
                        }
                    }
                    vertexId = node->getVertexId();
                    segmentationValues[vertexId] = currentId;
                }

                // create a new triangulation based on the old one, but with segmentation as
                // scalar values
                auto result = std::make_shared<topology::TriangulationData>(triangulation);
                result->setScalarValues(util::makeBuffer(std::move(segmentationValues)));
                result->setOffsets(std::move(offsets));
                if (result->isUniformGrid()) {
                    auto datamapper = result->getDataMapper();
                    datamapper.dataRange = vec2(0, maxComponentId + 2);
                    datamapper.valueRange = vec2(0, maxComponentId + 2);
                    result->set(triangulation.getGridDimensions(), triangulation.getGridOrigin(),
                                triangulation.getGridExtent(), datamapper);
                }

                return result;
            };

            return triangulation.getScalarValues()
                ->getEditableRepresentation<BufferRAM>()
                ->dispatch<Result, dispatching::filter::Scalars>(computeSegmentation);
        };
    };

    size_t numTimeSteps = scalarData.get()->size();
    const auto type = contourTrees->at(0)->type;

    /* Extract selected Components from property*/
    std::vector<std::vector<int>> selectedComponentIdsFromProperty(numTimeSteps);
    std::vector<std::map<int, ttk::SimplexId>> componentToVertexMap(numTimeSteps);
    for (auto p : selectedComponentIds_) {
        if (auto feature = dynamic_cast<ComponentProperty*>(p)) {
            auto timestep =
                dynamic_cast<IntSizeTProperty*>(feature->getPropertyByIdentifier("timeStep"))
                    ->get();
            // Turn into 0-based index
            timestep -= 1;
            if (timestep >= numTimeSteps) continue;
            auto componentId =
                dynamic_cast<IntSizeTProperty*>(feature->getPropertyByIdentifier("componentId"))
                    ->get();
            selectedComponentIdsFromProperty[timestep].push_back(componentId);
        }
    }

    // If leaf tracking is connected and is enabled
    std::vector<std::map<ttk::SimplexId, int>> vertexToComponentMap(numTimeSteps);
    // Maps per timestep
    std::vector<std::map<ttk::SimplexId, std::vector<std::pair<ttk::SimplexId, int>>>> edgesForward(
        numTimeSteps);
    std::vector<std::map<ttk::SimplexId, std::vector<std::pair<ttk::SimplexId, int>>>>
        edgesBackward(numTimeSteps);
    int maxComponentId = 0;
    int minComponentId = std::numeric_limits<int>::max();
    int currentComponentId = 0;
    if (!trackingInport_.isConnected() && leafOnly_ && trackingInsteadOfSUperArcs_) {
        LogProcessorWarn("No input tracking given, despite tracking segmentation selected");
    }
    if (trackingInport_.isConnected() && leafOnly_ && trackingInsteadOfSUperArcs_) {
        auto tracking = trackingInport_.getData();
        size_t rows = tracking->getNumberOfRows();
        size_t columns = tracking->getNumberOfColumns();
        if (columns != 9) {
            return;
        }
        // Iterate through all pairs creating the edge structures, figuring out components Ids
        // for vertices, based on component Ids for tracks Columns 0 - Index 1 - Vertex Id Start
        std::shared_ptr<const Column> columnIdStart = tracking->getColumn(1);
        // 2 - Critical Type Start
        std::shared_ptr<const Column> columnCpTypeStart = tracking->getColumn(2);
        // 3 - Time Start
        std::shared_ptr<const Column> columnTimeStart = tracking->getColumn(3);
        // 4 - Vertex Id End
        std::shared_ptr<const Column> columnIdEnd = tracking->getColumn(4);
        // 5 - Critical Type End
        std::shared_ptr<const Column> columnCpTypeEnd = tracking->getColumn(5);
        // 6 - Time End
        std::shared_ptr<const Column> columnTimeEnd = tracking->getColumn(6);
        // 7 - Cost (To accumulate)
        std::shared_ptr<const Column> columnCost = tracking->getColumn(7);
        // 8 - Component Id (To count number of tracks)
        std::shared_ptr<const Column> columnComponentId = tracking->getColumn(8);

        for (size_t r(0); r < rows; r++) {

            ttk::CriticalType cpType =
                ttk::CriticalType(std::stoi(columnCpTypeStart->getAsString(r)));

            // Join tree -> all leaves are minima
            // Split tree -> all leaves are maxima
            // Disregard the respective other
            if ((type == topology::TreeType::Join && cpType != ttk::CriticalType::Local_minimum) ||
                (type == topology::TreeType::Split && cpType != ttk::CriticalType::Local_maximum)) {
                continue;
            }

            auto tStart = std::stoi(columnTimeStart->getAsString(r));
            if (tStart < 0 || tStart >= numTimeSteps) {
                continue;
            }
            auto vertexIdStart = std::stoi(columnIdStart->getAsString(r));
            auto tEnd = std::stoi(columnTimeEnd->getAsString(r));
            if (tEnd < 0 || tEnd >= numTimeSteps) {
                continue;
            }
            auto vertexIdEnd = std::stoi(columnIdEnd->getAsString(r));
            auto componentId = std::stoi(columnComponentId->getAsString(r));
            edgesForward[tStart][vertexIdStart].push_back(std::make_pair(vertexIdEnd, componentId));
            edgesBackward[tEnd][vertexIdEnd].push_back(std::make_pair(vertexIdStart, componentId));
            if (maxComponentId < componentId) maxComponentId = componentId;
            if (minComponentId > componentId) minComponentId = componentId;
        }
        currentComponentId = maxComponentId + 1;
        // Iterate through all pairs again
        // If forwards = backwards = 1 (or one of them 0) -> take component that exist
        // If backwards > 1, forwards = 1 (merge) -> forwards component
        // If backwards = 1, forwards > 1 (split) -> backwards components
        // If both backwards > 1 or one >1 and the other 0 -> new componentId
        for (size_t r(0); r < rows; r++) {

            ttk::CriticalType cpType =
                ttk::CriticalType(std::stoi(columnCpTypeStart->getAsString(r)));

            // Join tree -> all leaves are minima
            // Split tree -> all leaves are maxima
            // Disregard the respective other
            if ((type == topology::TreeType::Join && cpType != ttk::CriticalType::Local_minimum) ||
                (type == topology::TreeType::Split && cpType != ttk::CriticalType::Local_maximum)) {
                continue;
            }

            auto tStart = std::stoi(columnTimeStart->getAsString(r));
            auto vertexIdStart = std::stoi(columnIdStart->getAsString(r));
            auto tEnd = std::stoi(columnTimeEnd->getAsString(r));
            auto vertexIdEnd = std::stoi(columnIdEnd->getAsString(r));
            // Component no longer needed here, was put into edgesForward/edgesBackward structure in
            // previous loop

            auto& startingComponent = vertexToComponentMap[tStart][vertexIdStart];
            auto& edgesForwardCurrent = edgesForward[tStart][vertexIdStart];
            size_t numForward = edgesForwardCurrent.size();
            auto& edgesBackwardCurrent = edgesBackward[tStart][vertexIdStart];
            size_t numBackward = edgesBackwardCurrent.size();
            if (numForward == 1)
                vertexToComponentMap[tStart][vertexIdStart] = edgesForwardCurrent[0].second;
            else if (numForward > 1 && numBackward == 1)
                vertexToComponentMap[tStart][vertexIdStart] = edgesBackwardCurrent[0].second;
            // numForward < 1 (ends here) or numBackward < 1 (starts here, but is followed by
            // split),
            else {
                // Could already be assigned if component ends here but is result of merge
                auto alreadyAssignedId = vertexToComponentMap[tEnd].find(vertexIdEnd);
                if (alreadyAssignedId == vertexToComponentMap[tEnd].end()) {
                    vertexToComponentMap[tStart][vertexIdStart] = currentComponentId;
                    currentComponentId++;
                }
            }
            // Create reverse map if we need it later
            if (selectedComponentsOnly_.get()) {
                componentToVertexMap[tStart][startingComponent] = vertexIdStart;
            }

            // If there are no forward edges present, also label the end point
            auto& edgesForwardAtEnd = edgesForward[tEnd][vertexIdEnd];
            size_t numForwardAtEnd = edgesForwardAtEnd.size();
            auto& endingComponent = vertexToComponentMap[tEnd][vertexIdEnd];
            if (numForwardAtEnd == 0) {
                auto& edgesBackwardAtEnd = edgesBackward[tEnd][vertexIdEnd];
                size_t numBackwardAtEnd = edgesBackwardAtEnd.size();
                if (numBackwardAtEnd == 1)
                    vertexToComponentMap[tEnd][vertexIdEnd] = edgesBackwardAtEnd[0].second;
                // numBackward > 1 (must be, since it is within a pair)
                else {
                    // Unless already assigned (there's at least two pointing to the end edge
                    auto alreadyAssignedId = vertexToComponentMap[tEnd].find(vertexIdEnd);
                    if (alreadyAssignedId == vertexToComponentMap[tEnd].end()) {
                        vertexToComponentMap[tEnd][vertexIdEnd] = currentComponentId;
                        currentComponentId++;
                    }
                }

                // Also create reverse map if we need it later
                if (selectedComponentsOnly_.get()) {
                    componentToVertexMap[tEnd][endingComponent] = vertexIdEnd;
                }
            }
        }
        maxComponentId = currentComponentId;

        if (selectedComponentsOnly_.get()) {
            std::vector<std::vector<int>> selectedComponentIdsExtended(numTimeSteps);
            // For the selected components, traverse forward and backward edges until there the end
            // is reached
            std::function<void(size_t, ttk::SimplexId,
                               std::map<size_t, std::vector<ttk::SimplexId>>&)>
                traverseEdgesForwards =
                    [&edgesForward, &traverseEdgesForwards](
                        const size_t timestep, const ttk::SimplexId vertexId,
                        std::map<size_t, std::vector<ttk::SimplexId>>& verticesPerTimeStep) {
                        if (timestep >= edgesForward.size()) return;
                        auto& verticesThisTimeStep = verticesPerTimeStep[timestep];
                        verticesThisTimeStep.push_back(vertexId);
                        const auto& forwardEdges = edgesForward[timestep][vertexId];
                        for (auto& edge : forwardEdges) {
                            traverseEdgesForwards(timestep + 1, edge.first, verticesPerTimeStep);
                        }
                    };

            std::function<void(size_t, ttk::SimplexId,
                               std::map<size_t, std::vector<ttk::SimplexId>>&)>
                traverseEdgesBackward =
                    [&edgesBackward, &traverseEdgesBackward](
                        const size_t timestep, const ttk::SimplexId vertexId,
                        std::map<size_t, std::vector<ttk::SimplexId>>& verticesPerTimeStep) {
                        auto& verticesThisTimeStep = verticesPerTimeStep[timestep];
                        verticesThisTimeStep.push_back(vertexId);
                        const auto& backwardEdges = edgesBackward[timestep][vertexId];
                        for (auto& edge : backwardEdges) {
                            if (timestep > 0)
                                traverseEdgesBackward(timestep - 1, edge.first,
                                                      verticesPerTimeStep);
                        }
                    };

            for (size_t i = 0; i < numTimeSteps; i++) {
                const auto& selectedThisTimeStep = selectedComponentIdsFromProperty[i];
                for (auto& componentId : selectedThisTimeStep) {
                    // Add component to extended list
                    selectedComponentIdsExtended[i].push_back(componentId);
                    // Find vertex that corresponds to this component
                    const auto& componentToVertexMapThisTimeStep = componentToVertexMap[i];
                    auto vertexIdIt = componentToVertexMapThisTimeStep.find(componentId);
                    if (vertexIdIt != componentToVertexMapThisTimeStep.end()) {
                        auto vertexId = vertexIdIt->second;
                        // Possible improvement: Make this a set, would remove duplicates
                        std::map<size_t, std::vector<ttk::SimplexId>> verticesForComponent;
                        traverseEdgesForwards(i, vertexId, verticesForComponent);
                        traverseEdgesBackward(i, vertexId, verticesForComponent);
                        // verticesForComponent contains all vertices that this component turns into
                        // over time, we now need to find the corresponding components
                        for (auto& timestepVerticesPair : verticesForComponent) {
                            size_t timestep = timestepVerticesPair.first;
                            // Skip current timestep (property selected component twice)
                            if (timestep == i) continue;
                            auto& verticesForTimeStep = timestepVerticesPair.second;
                            auto& vertexToComponentMapTimeStep = vertexToComponentMap[timestep];
                            for (ttk::SimplexId vertexInTimeStep : verticesForTimeStep) {
                                int correspondingComponent =
                                    vertexToComponentMapTimeStep[vertexInTimeStep];
                                selectedComponentIdsExtended[timestep].push_back(
                                    correspondingComponent);
                            }
                        }
                    } else {
                        LogProcessorWarn("Selected component " << componentId << "at timestep " << i
                                                               << " is not a tracked leaf.")
                    }
                }
            }

            size_t overallSelected = 0;
            // Overwrite selected property
            for (size_t i = 0; i < numTimeSteps; i++) {
                selectedComponentIdsFromProperty[i] = selectedComponentIdsExtended[i];
                overallSelected += selectedComponentIdsExtended[i].size();
            }
            LogProcessorInfo("Selected a total of " << overallSelected << " components");
        }
    }

    std::vector<std::function<Result(pool::Stop, pool::Progress progress)>> jobs;
    if (!trackingInsteadOfSUperArcs_) {
        for (size_t i = 0; i < numTimeSteps; i++) {
            const auto numSuperArcs = contourTrees.get()->at(i)->getTree()->getNumberOfSuperArcs();
            if (static_cast<int>(numSuperArcs) > maxComponentId) maxComponentId = numSuperArcs;
        }
    }
    // At the end of this maxComponentId is one larger than the actual last assigned components Id
    // (either from tracked components of from numSuperArcs
    maxComponentId -= 1;

    for (size_t i = 0; i < numTimeSteps; i++) {
        jobs.push_back(computeSegmentationJob(
            *scalarData.get()->at(i), *contourTrees.get()->at(i), vertexToComponentMap[i],
            maxComponentId, leafOnly_.get(), trackingInsteadOfSUperArcs_.get(),
            selectedComponentIdsFromProperty[i], selectedComponentsOnly_.get()));
    }
    generateColorMap(0, maxComponentId);
    outport_.setData(nullptr);
    dispatchMany(jobs, [this](std::vector<Result> result) {
        segmentations_ = result;
        outport_.setData(std::make_shared<TriangulationSequence>(segmentations_));
        newResults();
        timer_.set(performanceTimer_.ElapsedTimeAndReset());
    });
}

}  // namespace inviwo
