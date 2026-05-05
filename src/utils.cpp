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

#include <log_view/utils.h>

#include <cstdlib>
#include <cwchar>

#include <sstream>

namespace log_view {

attr_t kAttrGrey   = 0;
attr_t kAttrGreyBg = 0;

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
  } else {
    return text.find(substr) != std::string::npos;
  }
}

std::vector<size_t> find(
  const std::string& text, const std::string& substr, bool case_insensitive) {
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
  } else {
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

// Parse semicolon-separated SGR params and update the AnsiSegment state.
static void applySgr(const std::string& params, int& ansi_fg, bool& bold, bool& dim) {
  // Split on ';', treating empty string as a single "0" param.
  std::string buf = params.empty() ? "0" : params;
  size_t start = 0;
  while (true) {
    size_t end = buf.find(';', start);
    std::string token = buf.substr(start, end == std::string::npos ? end : end - start);
    int p = token.empty() ? 0 : std::stoi(token);

    if (p == 0) {
      ansi_fg = -1; bold = false; dim = false;
    } else if (p == 1) {
      bold = true;
    } else if (p == 2) {
      dim = true;
    } else if (p == 22) {
      bold = false; dim = false;
    } else if (p >= 30 && p <= 37) {
      ansi_fg = p - 30;
    } else if (p == 39) {
      ansi_fg = -1;
    } else if (p >= 90 && p <= 97) {
      ansi_fg = p - 90; bold = true;
    }
    // All other params (bg colors, 256-color, true-color) are silently ignored.

    if (end == std::string::npos) break;
    start = end + 1;
  }
}

std::string stripAnsi(const std::string& raw) {
  std::string out;
  out.reserve(raw.size());
  enum { NORMAL, ESC, CSI } state = NORMAL;
  for (char c : raw) {
    if (state == NORMAL) {
      if (c == '\033') {
        state = ESC;
      } else {
        out += c;
      }
    } else if (state == ESC) {
      state = (c == '[') ? CSI : NORMAL;
    } else {  // CSI
      if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z')) { state = NORMAL; }
    }
  }
  return out;
}

std::vector<AnsiSegment> parseAnsiSegments(const std::string& raw) {
  std::vector<AnsiSegment> segments;
  int  cur_fg   = -1;
  bool cur_bold = false;
  bool cur_dim  = false;

  AnsiSegment seg;
  seg.ansi_fg = cur_fg; seg.bold = cur_bold; seg.dim = cur_dim;

  enum { NORMAL, ESC, CSI } state = NORMAL;
  std::string param_buf;

  for (char c : raw) {
    if (state == NORMAL) {
      if (c == '\033') {
        state = ESC;
      } else {
        seg.text += c;
      }
    } else if (state == ESC) {
      if (c == '[') {
        state = CSI;
        param_buf.clear();
      } else {
        state = NORMAL;  // non-CSI escape: discard
      }
    } else {  // CSI
      if (c == 'm') {
        // SGR sequence: flush current segment, start new one with updated attrs
        if (!seg.text.empty()) {
          segments.push_back(std::move(seg));
          seg = AnsiSegment{};
        }
        try { applySgr(param_buf, cur_fg, cur_bold, cur_dim); } catch (...) {}
        seg.ansi_fg = cur_fg; seg.bold = cur_bold; seg.dim = cur_dim;
        state = NORMAL;
      } else if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z')) {
        state = NORMAL;  // non-m terminator: discard sequence
      } else {
        param_buf += c;  // digit or ';'
      }
    }
  }

  if (!seg.text.empty()) {
    segments.push_back(std::move(seg));
  }
  return segments;
}

size_t utf8DisplayWidth(const std::string& s) {
  size_t width = 0;
  const char* p = s.c_str();
  const char* end = p + s.size();
  mbstate_t state{};
  while (p < end) {
    wchar_t wc;
    size_t n = mbrtowc(&wc, p, static_cast<size_t>(end - p), &state);
    if (n == 0 || n == static_cast<size_t>(-1) || n == static_cast<size_t>(-2)) {
      ++p;
      ++width;
    } else {
      int w = wcwidth(wc);
      if (w > 0) width += static_cast<size_t>(w);
      p += n;
    }
  }
  return width;
}

std::string utf8EraseDisplayCols(const std::string& s, size_t cols) {
  size_t consumed = 0;
  const char* p = s.c_str();
  const char* end = p + s.size();
  mbstate_t state{};
  while (p < end && consumed < cols) {
    wchar_t wc;
    size_t n = mbrtowc(&wc, p, static_cast<size_t>(end - p), &state);
    if (n == 0 || n == static_cast<size_t>(-1) || n == static_cast<size_t>(-2)) {
      ++consumed;
      ++p;
    } else {
      int w = wcwidth(wc);
      if (w > 0) consumed += static_cast<size_t>(w);
      p += n;
    }
  }
  return s.substr(static_cast<size_t>(p - s.c_str()));
}

std::string utf8TruncateDisplayCols(const std::string& s, size_t cols) {
  size_t used = 0;
  const char* p = s.c_str();
  const char* end = p + s.size();
  mbstate_t state{};
  while (p < end) {
    wchar_t wc;
    size_t n = mbrtowc(&wc, p, static_cast<size_t>(end - p), &state);
    size_t col_w;
    if (n == 0 || n == static_cast<size_t>(-1) || n == static_cast<size_t>(-2)) {
      col_w = 1;
      n = 1;
    } else {
      int w = wcwidth(wc);
      col_w = (w > 0) ? static_cast<size_t>(w) : 0;
    }
    if (used + col_w > cols) break;
    used += col_w;
    p += n;
  }
  return s.substr(0, static_cast<size_t>(p - s.c_str()));
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
