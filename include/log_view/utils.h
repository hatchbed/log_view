// Copyright 2020 Hatchbed L.L.C.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <algorithm>
#include <cctype>
#include <string>
#include <vector>

#include <curses.h>

namespace log_view {

enum Color {
  CP_DEFAULT, CP_RED, CP_YELLOW, CP_GREY, CP_DEFAULT_CYAN, CP_DEFAULT_GREY,
  CP_ANSI_BLACK, CP_ANSI_RED, CP_ANSI_GREEN, CP_ANSI_YELLOW,
  CP_ANSI_BLUE, CP_ANSI_MAGENTA, CP_ANSI_CYAN, CP_ANSI_WHITE
};

struct AnsiSegment {
  std::string text;
  int ansi_fg = -1;  // -1 = default, 0-7 = ANSI standard color index
  bool bold = false;
  bool dim  = false;
};

std::string toString(double val, int precision);

int ctrl(char key);

std::vector<std::string> split(const std::string &text, char sep);

bool contains(const std::string& text, const std::string& substr, bool case_insensitive);

std::vector<size_t> find(
  const std::string& text, const std::string& substr, bool case_insensitive);

void toClipboard(const std::string& text);

std::string stripAnsi(const std::string& raw);
std::vector<AnsiSegment> parseAnsiSegments(const std::string& raw);

size_t utf8DisplayWidth(const std::string& s);
std::string utf8EraseDisplayCols(const std::string& s, size_t cols);
std::string utf8TruncateDisplayCols(const std::string& s, size_t cols);

extern attr_t kAttrGrey;    // replaces COLOR_PAIR(CP_GREY)   — dim on 8-color terminals
extern attr_t kAttrGreyBg;  // replaces COLOR_PAIR(CP_DEFAULT_GREY) — reverse on 8-color terminals

}  // namespace log_view
