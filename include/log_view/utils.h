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
#include <regex>
#include <string>
#include <vector>

#include <curses.h>

namespace log_view {

enum Color {
  CP_DEFAULT, CP_RED, CP_YELLOW, CP_GREY, CP_DEFAULT_CYAN, CP_DEFAULT_GREY,
  CP_ANSI_BLACK, CP_ANSI_RED, CP_ANSI_GREEN, CP_ANSI_YELLOW,
  CP_ANSI_BLUE, CP_ANSI_MAGENTA, CP_ANSI_CYAN, CP_ANSI_WHITE,
  CP_BRIGHT_BLUE,           // color 12 on 16+ color terminals; falls back to COLOR_BLUE
  CP_BRIGHT_MAGENTA,        // color 13 on 16+ color terminals; falls back to COLOR_MAGENTA
  CP_CYAN_GREY,             // cyan fg on grey bg (16+ color); falls back to CP_ANSI_CYAN
  CP_BRIGHT_MAGENTA_GREY,   // bright magenta fg on grey bg (16+); falls back to CP_BRIGHT_MAGENTA
  CP_WHITE_CYAN             // white foreground on cyan background
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

std::string levelName(uint8_t level);

void printStyledAt(WINDOW* win, int y, int x, attr_t attr, const char* fmt, ...);

// Render a pattern input string at (row, col), coloring {placeholder} segments cyan and
// /regex/ segments bright-magenta. If base_attr is non-zero it is re-applied via wattron
// after each colored segment so the caller's background attribute is preserved.
// Returns the column after the last character printed.
int renderPatternInput(WINDOW* win, int row, int col, const std::string& text,
                       attr_t base_attr = 0);

std::string stripAnsi(const std::string& raw);
std::vector<AnsiSegment> parseAnsiSegments(const std::string& raw);

size_t utf8DisplayWidth(const std::string& s);
std::string utf8EraseDisplayCols(const std::string& s, size_t cols);
std::string utf8TruncateDisplayCols(const std::string& s, size_t cols);

extern attr_t kAttrGrey;      // replaces COLOR_PAIR(CP_GREY)         - dim on 8-color terminals
extern attr_t kAttrGreyBg;    // replaces COLOR_PAIR(CP_DEFAULT_GREY) - reverse on 8-color terminals
extern attr_t kAttrBoldBlue;  // A_BOLD | COLOR_PAIR(CP_ANSI_BLUE)    - section headers

// Maps ANSI color index (0-7) to the corresponding CP_ANSI_* color pair.
inline constexpr int kAnsiPairs[] = {
  CP_ANSI_BLACK, CP_ANSI_RED,     CP_ANSI_GREEN,   CP_ANSI_YELLOW,
  CP_ANSI_BLUE,  CP_ANSI_MAGENTA, CP_ANSI_CYAN,    CP_ANSI_WHITE
};

// Compiled pattern supporting three matching modes:
//   Literal (default): case-insensitive substring match (zero overhead over plain contains())
//   Regex: /pattern/ or /pattern/i  -- std::regex, compiled once on Pattern::compile()
//   Template: fixed text with {placeholder} gaps for numeric comparisons and alternations
//
// Placeholder syntax:
//   {>N} {<N} {>=N} {<=N} {=N}  -- numeric comparison
//   {N..M}                       -- numeric range [N, M]
//   {a|b|c}                      -- case-insensitive token alternation
//   {*} or {}                    -- wildcard (any non-whitespace token)
struct Pattern {
  enum class Type { Literal, Regex, Template };

  static Pattern compile(const std::string& raw);

  bool matches(const std::string& text) const;

  // Returns {start_byte, length} for every match within text.
  std::vector<std::pair<size_t, size_t>> findAll(const std::string& text) const;

  bool valid() const { return valid_; }
  bool empty() const { return raw.empty(); }

  std::string raw;
  Type type = Type::Literal;

private:
  struct Placeholder {
    enum class Kind { Wildcard, Numeric, Alternation };
    enum class NumOp { GT, LT, GEQ, LEQ, EQ, RANGE };
    Kind   kind    = Kind::Wildcard;
    NumOp  num_op  = NumOp::EQ;
    double num_val  = 0.0;
    double num_val2 = 0.0;
    std::vector<std::string> choices;
  };

  bool valid_ = true;
  std::regex compiled_regex_;
  std::vector<std::string> fixed_parts_;
  std::vector<Placeholder> placeholders_;

  static Placeholder parsePlaceholder(const std::string& content);
  // Returns end position (exclusive) of a template match starting at pos, or npos on failure.
  size_t matchTemplateEnd(const std::string& text, size_t pos) const;
  bool matchTemplate(const std::string& text, size_t pos) const;
};

}  // namespace log_view
