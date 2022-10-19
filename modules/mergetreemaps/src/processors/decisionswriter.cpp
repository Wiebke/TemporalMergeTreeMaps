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

#include <inviwo/mergetreemaps/processors/decisionswriter.h>
#include <inviwo/core/util/filesystem.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo DecisionsWriter::processorInfo_{
    "org.inviwo.DecisionsWriter",  // Class identifier
    "Decisions Writer",            // Display name
    "Merge Tree Maps",             // Category
    CodeState::Experimental,       // Code state
    Tags::None,                    // Tags
};
const ProcessorInfo DecisionsWriter::getProcessorInfo() const { return processorInfo_; }

DecisionsWriter::DecisionsWriter()
    : Processor()
    , inport_("inport")
    , treesInport_("treesInport")
    , propFilename("filename", "Filename")
    , propMarkLeaves("markLeaves", "Mark Leaves")
    , propOverwrite("overwrite", "Overwrite", false) {

    addPort(inport_);
    addPort(treesInport_);
    treesInport_.setOptional(true);
    propFilename.addNameFilter(FileExtension("mzn", "MiniZinc Decision Variables"));
    propFilename.setAcceptMode(AcceptMode::Save);
    addProperty(propFilename);
    addProperty(propMarkLeaves);
    addProperty(propOverwrite);
}

void DecisionsWriter::process() {
    // Get filename and open file
    const std::string& Filename = propFilename.get();

    if (filesystem::fileExists(Filename) && !propOverwrite.get()) {
        LogWarn("File already exists: " << Filename);
        return;
    }

    // Get the decisions
    auto decisions = inport_.getData();

    bool markLeaves = false;
    std::shared_ptr<const ContourTreeSequence> trees;
    if (treesInport_.isConnected() && treesInport_.getData()) {
        trees = treesInport_.getData();
        markLeaves = propMarkLeaves.get() && decisions->size() == trees->size();
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

    const auto isLeaf = [trees](size_t index, size_t superArcIndex) -> bool {
        if (index >= trees->size()) return false;
        const auto currentTree = trees->at(index)->getTree();
        if (superArcIndex < currentTree->getNumberOfSuperArcs()) {
            const auto& superArc = currentTree->getSuperArc(superArcIndex);
            const auto& downNode = currentTree->getNode(superArc->getDownNodeId());
            return downNode->getNumberOfDownSuperArcs() == 0;
        }
        return false;
    };

    // Stream it out as ASCII
    try {
        for (size_t i = 0; i < decisions->size(); i++) {
            outfile << "decisions_" << i << " = [";
            const auto currentDecisions = decisions->at(i);
            const auto currentSize = currentDecisions.size();
            for (size_t s = 0; s < currentSize; s++) {
                if (!markLeaves || !isLeaf(i, s)) {
                    outfile << ((currentDecisions[s] == 0) ? "false" : "true");
                } else {
                    outfile << "-";
                }
                if (s != currentSize - 1) outfile << ", ";
            }
            // Closing brace for decision
            outfile << "];\n";
        }

    } catch (const std::ofstream::failure& e) {
        LogError("Error during save: " << Filename);
        LogError("  Error Code: " << e.code() << "    . " << e.what());
        return;
    }

    LogProcessorInfo("Wrote decisions to " << Filename);
    outfile.close();
}

}  // namespace inviwo
