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

#include <modules/temporaltreemaps/processors/treeconverter.h>

namespace inviwo {
namespace kth {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo TreeConverter::processorInfo_{
    "org.inviwo.TreeConverter",  // Class identifier
    "Tree Converter",            // Display name
    "Temporal Tree",             // Category
    CodeState::Experimental,     // Code state
    Tags::None,                  // Tags
};
const ProcessorInfo TreeConverter::getProcessorInfo() const { return processorInfo_; }

TreeConverter::TreeConverter()
    : Processor()
    , portInTree("inTree")
    , portOutTree("outTree")
    , propOutputType("outputType", "Output Aggregation") {

    addPort(portInTree);
    addPort(portOutTree);

    addProperty(propOutputType);
    propOutputType.addOption("aggregated", "Fully Aggregated",
                             TemporalTree::TTreeAggregation::FullyAggregated);
    propOutputType.addOption("deaggregated", "Fully Deaggregated",
                             TemporalTree::TTreeAggregation::FullyDeaggregated);
    propOutputType.addOption("aggreagtedLeaves", "Aggregated Leaves",
                             TemporalTree::TTreeAggregation::LeafOnlyAggregation);
    propOutputType.setSelectedIndex(0);
    propOutputType.setCurrentStateAsDefault();
}

void TreeConverter::process() {
    // Get tree
    std::shared_ptr<const TemporalTree> pInTree = portInTree.getData();
    if (!pInTree) return;

    std::shared_ptr<TemporalTree> pOutTree;

    if (propOutputType.get() == TemporalTree::TTreeAggregation::FullyAggregated) {
        // Aggregate
        pOutTree = std::make_shared<TemporalTree>(pInTree->aggregate(false));
    } else if (propOutputType.get() == TemporalTree::TTreeAggregation::FullyDeaggregated) {
        // Deaggregate
        pOutTree = std::make_shared<TemporalTree>();
        pInTree->deaggregate(*pOutTree);
    } else if (propOutputType.get() == TemporalTree::TTreeAggregation::LeafOnlyAggregation) {
        // Tree could be aggregated fully, thus deaggregate first, then aggregate leaves
        auto pDeaggreagted = std::make_shared<TemporalTree>();
        pInTree->deaggregate(*pDeaggreagted);
        pOutTree = std::make_shared<TemporalTree>(pDeaggreagted->aggregate(true));
    } else {
        // Copy input
        pOutTree = std::make_shared<TemporalTree>(TemporalTree(*pInTree));
    }

    portOutTree.setData(pOutTree);
}

}  // namespace kth
}  // namespace inviwo
