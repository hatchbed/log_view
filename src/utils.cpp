/**
 * Copyright 2020 Hatchbed L.L.C.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <log_view/utils.h>

#include <cstdlib>
#include <sstream>

namespace log_view {

std::string toString(double val, int precision) {
  std::ostringstream ss;
  ss.precision(precision);
  ss << std::fixed << val;
  return ss.str();
}

std::vector<std::string> split(const std::string &text, char sep) {
  if (text.empty()) {
    return {};
  }

  std::vector<std::string> tokens;
  size_t start = 0;
  size_t end = 0;
  while ((end = text.find(sep, start)) != std::string::npos) {
    if (end != start) {
      tokens.push_back(text.substr(start, end - start));
    }
    start = end + 1;
  }
  if (end != start) {
    auto token = text.substr(start);
    if (!token.empty()) {
      tokens.push_back(text.substr(start));
    }
  }
  return tokens;
}

bool contains(const std::string& text, const std::string& substr, bool case_insensitive) {
  if (substr.empty()) {
    return true;
  }

  if (case_insensitive) {
    auto it = std::search(
      text.begin(), text.end(),
      substr.begin(), substr.end(),
      [](char ch1, char ch2) { return std::toupper(ch1) == std::toupper(ch2); }
    );
    return it != text.end();
  }
  else {
    return text.find(substr) != std::string::npos;
  }
}

std::vector<size_t> find(const std::string& text, const std::string& substr, bool case_insensitive) {
  if (substr.empty()) {
    return {};
  }

  std::vector<size_t> indices;

  if (case_insensitive) {
    auto it = std::search(
      text.begin(), text.end(),
      substr.begin(), substr.end(),
      [](char ch1, char ch2) { return std::toupper(ch1) == std::toupper(ch2); }
    );

    while (it != text.end()) {
      size_t index = std::distance(text.begin(), it);
      indices.push_back(index);
      it = std::search(
        text.begin() + index + 1, text.end(),
        substr.begin(), substr.end(),
        [](char ch1, char ch2) { return std::toupper(ch1) == std::toupper(ch2); }
      );
    }
  }
  else {
    size_t loc = text.find(substr, 0);
    while (loc != std::string::npos) {
      indices.push_back(loc);
      loc = text.find(substr, loc + 1);
    }
  }

  return indices;
}


int ctrl(char key) {
  return key & 0x1f;
}

void toClipboard(const std::string& text) {
  FILE* pipe = popen("xclip -sel clip", "w");
  if (!pipe) {
    return;
  }
  fwrite(text.data(), sizeof(char), text.size(), pipe);
  pclose(pipe);
}

}  // namespace log_view
