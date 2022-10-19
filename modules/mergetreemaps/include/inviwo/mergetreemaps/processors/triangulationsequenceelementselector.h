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
#include <modules/base/processors/vectorelementselectorprocessor.h>
#include <inviwo/topologytoolkit/ports/triangulationdataport.h>
#include <inviwo/topologytoolkit/datastructures/triangulationdata.h>

namespace inviwo {

/** \docpage{org.inviwo.Triangulation Sequence Element Selector, Triangulation Sequence Element
 * Selector}
 * ![](org.inviwo.Triangulation Sequence Element
 * Selector.png?classIdentifier=org.inviwo.Triangulation Sequence Element Selector) Explanation of
 * how to use the processor.
 *
 * Select a specific triangulation out of a sequence.
 *
 * ### Inport
 *   * __inport__ Sequence of triangulation for TTK
 * ### Outport
 *   * __outport__ Selected triangulation
 *
 * ### Properties
 *   * __Step__ The triangulation sequence index to extract
 */
class IVW_MODULE_MERGETREEMAPS_API TriangulationSequenceElementSelector
    : public VectorElementSelectorProcessor<topology::TriangulationData,
                                            topology::TriangulationOutport> {
public:
    TriangulationSequenceElementSelector();
    virtual ~TriangulationSequenceElementSelector() = default;

    virtual const ProcessorInfo getProcessorInfo() const override;
    static const ProcessorInfo processorInfo_;

private:
};

}  // namespace inviwo
