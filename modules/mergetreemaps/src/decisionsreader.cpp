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

#include <inviwo/mergetreemaps/decisionsreader.h>

#include <inviwo/core/util/filesystem.h>
#include <inviwo/core/util/stringconversion.h>

#include <fstream>

namespace inviwo {

DecisionsReaderException::DecisionsReaderException(const std::string& message,
                                                   ExceptionContext context)
    : DataReaderException("DecisionsReader: " + message, context) {}

DecisionsReader::DecisionsReader() : DataReaderType<LandscapeDecisions>() {
    addExtension(FileExtension("mzn", "MiniZinc Variables"));
}

DecisionsReader* DecisionsReader::clone() const { return new DecisionsReader(*this); }

std::shared_ptr<LandscapeDecisions> DecisionsReader::readData(const std::string& fileName) {
    auto file = filesystem::ifstream(fileName);

    if (!file.is_open()) {
        throw FileException(
            std::string("DecisionsReader: Could not open file \"" + fileName + "\"."), IVW_CONTEXT);
    }
    file.seekg(0, std::ios::end);
    std::streampos len = file.tellg();
    file.seekg(0, std::ios::beg);

    if (len == std::streampos(0)) {
        throw DecisionsReaderException("Empty file, no data", IVW_CONTEXT);
    }

    return readData(file);
}

std::shared_ptr<LandscapeDecisions> DecisionsReader::readData(std::istream& stream) const {

    auto decisions = std::make_shared<LandscapeDecisions>();

    // Every line should look like this:
    // decisions_0 = [ -, -, false, -, -, false, -, -, false, false, false ];
    std::string line;

    while (std::getline(stream, line)) {
        decisions->push_back(std::vector<int>());
        auto& currentDecisions = decisions->back();
        std::stringstream sLine;
        sLine << line;
        std::string nextToken;
        while (sLine >> nextToken) {
            size_t pos = -1;
            if (((pos = nextToken.find('[')) != std::string::npos)) nextToken = nextToken.substr(1);
            while ((pos = nextToken.rfind(',')) != std::string::npos) {
                nextToken.erase(pos, 1);
            }
            while ((pos = nextToken.rfind(']')) != std::string::npos) {
                nextToken.erase(pos, 1);
            }
            while ((pos = nextToken.rfind(';')) != std::string::npos) {
                nextToken.erase(pos, 1);
            }

            if (nextToken.compare("true") == 0) {
                currentDecisions.push_back(1);
            } else if (nextToken.compare("false") == 0 || nextToken.compare("-") == 0) {
                currentDecisions.push_back(0);
            }
        }
    }

    return decisions;
}

}  // namespace inviwo
