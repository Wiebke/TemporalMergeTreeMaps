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
#include <inviwo/mergetreemaps/topologysequenceports.h>

#include <inviwo/core/io/datareader.h>
#include <inviwo/core/io/datareaderexception.h>

namespace inviwo {

/**
 * \class DecisionsReaderException
 *
 * \brief This exception is thrown by the DecisionsReader in case the input is malformed.
 * This includes empty sources, unmatched quotes, missing headers.
 * \see DecisionsReader
 */
class IVW_MODULE_MERGETREEMAPS_API DecisionsReaderException : public DataReaderException {
public:
    DecisionsReaderException(const std::string& message = "",
                             ExceptionContext context = ExceptionContext());
};

/**
 * \class DecisionsReader
 * \ingroup dataio
 *
 * \brief A reader for a merge tree map optimization decisions
 */
class IVW_MODULE_MERGETREEMAPS_API DecisionsReader : public DataReaderType<LandscapeDecisions> {
public:
    DecisionsReader();
    DecisionsReader(const DecisionsReader&) = default;
    DecisionsReader(DecisionsReader&&) noexcept = default;
    DecisionsReader& operator=(const DecisionsReader&) = default;
    DecisionsReader& operator=(DecisionsReader&&) noexcept = default;
    virtual DecisionsReader* clone() const override;
    virtual ~DecisionsReader() = default;

    using DataReaderType<LandscapeDecisions>::readData;

    /**
     * read a decisions file from a file
     *
     * @param fileName name of the input decisions file
     * @return a LandscapeDecisions (vector of vector of ints)
     * @throws FileException if the file cannot be accessed
     * @throws DecisionsReaderException if the file contains no data or other problems occur while
     * reading
     */
    virtual std::shared_ptr<LandscapeDecisions> readData(const std::string& fileName) override;

    /**
     * read a CSV file from a input stream, e.g. a std::ifstream. In case
     * file streams are used, the file must have be opened prior calling this function.
     *
     * @param stream    input stream with the CSV data
     * @return a DataFrame containing the CSV data
     * @throws CSVDataReaderException if the given stream is in a bad state,
     * the stream contains no data, other problems occur while reading
     */
    std::shared_ptr<LandscapeDecisions> readData(std::istream& stream) const;

private:
};

}  // namespace inviwo
