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

#include <inviwo/mergetreemaps/processors/orderingfromspacetimecontourtree.h>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo OrderingFromSpaceTimeContourTree::processorInfo_{
    "org.inviwo.OrderingFromSpaceTimeContourTree",  // Class identifier
    "Ordering From Space Time Contour Tree",        // Display name
    "Merge Tree Maps",                              // Category
    CodeState::Experimental,                        // Code state
    Tags::None,                                     // Tags
};
const ProcessorInfo OrderingFromSpaceTimeContourTree::getProcessorInfo() const {
    return processorInfo_;
}

OrderingFromSpaceTimeContourTree::OrderingFromSpaceTimeContourTree()
    : PoolProcessor()
    , treeType_("treeType", "Tree Type", "", InvalidationLevel::Valid)
    , timer_("timer", "Eval Time (s)", 0.f, 0.f, std::numeric_limits<float>::max(), 0.001f,
             InvalidationLevel::Valid, PropertySemantics::Text) {

    addPort(inport_);
    addPort(treesInport_);
    addPort(decisionsOutport_);
    addProperties(treeType_, timer_);
    timer_.setReadOnly(true);

    treeType_.setReadOnly(true);
    inport_.onChange([this]() {
        treeType_.set([this]() -> std::string {
            if (inport_.hasData()) {
                return toString(inport_.getData()->type);
            } else {
                return "";
            }
        }());
    });
}

void OrderingFromSpaceTimeContourTree::process() {
    performanceTimer_.Reset();
    // outport_.setData(myImage);
    timer_.set(performanceTimer_.ElapsedTimeAndReset());
}

}  // namespace inviwo
