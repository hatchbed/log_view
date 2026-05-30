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

#include <cstdarg>
#include <cstdlib>
#include <cwchar>

#include <rcl_interfaces/msg/log.hpp>

#include <sstream>

namespace log_view {

attr_t kAttrGrey     = 0;
attr_t kAttrGreyBg   = 0;
attr_t kAttrBoldBlue = 0;

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
      tokens.push_back(token);
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

std::string levelName(uint8_t level) {
  if (level == rcl_interfaces::msg::Log::DEBUG) { return "DEBUG"; }
  if (level == rcl_interfaces::msg::Log::INFO)  { return "INFO"; }
  if (level == rcl_interfaces::msg::Log::WARN)  { return "WARN"; }
  if (level == rcl_interfaces::msg::Log::ERROR) { return "ERROR"; }
  if (level == rcl_interfaces::msg::Log::FATAL) { return "FATAL"; }
  return std::to_string(level);
}

void printStyledAt(WINDOW* win, int y, int x, attr_t attr, const char* fmt, ...) {
  if (attr) wattron(win, attr);
  wmove(win, y, x);
  va_list args;
  va_start(args, fmt);
  vw_printw(win, fmt, args);
  va_end(args);
  if (attr) wattroff(win, attr);
}

int renderPatternInput(WINDOW* win, int row, int col, const std::string& text, attr_t base_attr) {
  attr_t tmpl_attr  = base_attr ? COLOR_PAIR(CP_CYAN_GREY)           : COLOR_PAIR(CP_ANSI_CYAN);
  attr_t regex_attr = base_attr ? COLOR_PAIR(CP_BRIGHT_MAGENTA_GREY) : COLOR_PAIR(CP_BRIGHT_MAGENTA);

  size_t pos = 0;
  while (pos <= text.size()) {
    size_t seg_end = text.find(';', pos);
    if (seg_end == std::string::npos) seg_end = text.size();

    const char* seg_ptr = text.c_str() + pos;
    int seg_len = static_cast<int>(seg_end - pos);

    bool is_regex = seg_len > 0 && seg_ptr[0] == '/' &&
                    text.rfind('/', seg_end - 1) > pos;
    bool has_template = !is_regex &&
                        text.find('{', pos) < seg_end;

    if (is_regex) {
      printStyledAt(win, row, col, regex_attr, "%.*s", seg_len, seg_ptr);
      col += seg_len;
      if (base_attr) wattron(win, base_attr);
    } else if (has_template) {
      size_t tpos = pos;
      while (tpos < seg_end) {
        size_t brace = text.find('{', tpos);
        if (brace == std::string::npos || brace >= seg_end) {
          int len = static_cast<int>(seg_end - tpos);
          if (len > 0) { mvwprintw(win, row, col, "%.*s", len, text.c_str() + tpos); col += len; }
          break;
        }
        if (brace > tpos) {
          int len = static_cast<int>(brace - tpos);
          mvwprintw(win, row, col, "%.*s", len, text.c_str() + tpos);
          col += len;
        }
        size_t close = text.find('}', brace + 1);
        if (close == std::string::npos || close >= seg_end) {
          int len = static_cast<int>(seg_end - brace);
          mvwprintw(win, row, col, "%.*s", len, text.c_str() + brace);
          col += len;
          break;
        }
        int len = static_cast<int>(close - brace + 1);
        printStyledAt(win, row, col, tmpl_attr, "%.*s", len, text.c_str() + brace);
        col += len;
        if (base_attr) wattron(win, base_attr);
        tpos = close + 1;
      }
    } else {
      if (seg_len > 0) {
        mvwprintw(win, row, col, "%.*s", seg_len, seg_ptr);
        col += seg_len;
      }
    }

    pos = seg_end;
    if (pos < text.size()) {
      mvwprintw(win, row, col, ";");
      col++;
      pos++;
    } else {
      break;
    }
  }
  return col;
}

void toClipboard(const std::string& text) {
  FILE* pipe = popen("xclip -sel clip", "w");
  if (!pipe) {
    return;
  }
  fwrite(text.data(), sizeof(char), text.size(), pipe);
  pclose(pipe);
}

// ------ Pattern ------

Pattern Pattern::compile(const std::string& raw) {
  Pattern p;
  p.raw = raw;
  if (raw.empty()) {
    return p;
  }

  if (raw[0] == '/') {
    size_t close = raw.rfind('/');
    if (close == 0) {
      return p;  // no closing slash — treat as literal
    }
    p.type = Type::Regex;
    std::string pat = raw.substr(1, close - 1);
    std::string flag_str = raw.substr(close + 1);
    auto flags = std::regex_constants::ECMAScript;
    if (flag_str.find('i') != std::string::npos) {
      flags |= std::regex_constants::icase;
    }
    try {
      p.compiled_regex_ = std::regex(pat, flags);
    } catch (const std::regex_error&) {
      p.valid_ = false;
    }
    return p;
  }

  if (raw.find('{') != std::string::npos) {
    p.type = Type::Template;
    size_t pos = 0;
    while (pos <= raw.size()) {
      size_t brace = raw.find('{', pos);
      if (brace == std::string::npos) {
        p.fixed_parts_.push_back(raw.substr(pos));
        break;
      }
      p.fixed_parts_.push_back(raw.substr(pos, brace - pos));
      size_t end_brace = raw.find('}', brace + 1);
      if (end_brace == std::string::npos) {
        p.fixed_parts_.back() += raw.substr(brace);
        break;
      }
      std::string content = raw.substr(brace + 1, end_brace - brace - 1);
      p.placeholders_.push_back(parsePlaceholder(content));
      pos = end_brace + 1;
    }
    if (p.placeholders_.empty()) {
      p.type = Type::Literal;
      p.fixed_parts_.clear();
    }
    return p;
  }

  return p;
}

Pattern::Placeholder Pattern::parsePlaceholder(const std::string& content) {
  Placeholder ph;
  if (content.empty() || content == "*") {
    ph.kind = Placeholder::Kind::Wildcard;
    return ph;
  }
  if (content.find('|') != std::string::npos) {
    ph.kind = Placeholder::Kind::Alternation;
    ph.choices = split(content, '|');
    return ph;
  }
  ph.kind = Placeholder::Kind::Numeric;
  try {
    if (content.size() >= 2 && content[0] == '>' && content[1] == '=') {
      ph.num_op  = Placeholder::NumOp::GEQ;
      ph.num_val = std::stod(content.substr(2));
    } else if (content.size() >= 2 && content[0] == '<' && content[1] == '=') {
      ph.num_op  = Placeholder::NumOp::LEQ;
      ph.num_val = std::stod(content.substr(2));
    } else if (content[0] == '>') {
      ph.num_op  = Placeholder::NumOp::GT;
      ph.num_val = std::stod(content.substr(1));
    } else if (content[0] == '<') {
      ph.num_op  = Placeholder::NumOp::LT;
      ph.num_val = std::stod(content.substr(1));
    } else if (content[0] == '=') {
      ph.num_op  = Placeholder::NumOp::EQ;
      ph.num_val = std::stod(content.substr(1));
    } else {
      size_t range = content.find("..");
      if (range != std::string::npos) {
        ph.num_op   = Placeholder::NumOp::RANGE;
        ph.num_val  = std::stod(content.substr(0, range));
        ph.num_val2 = std::stod(content.substr(range + 2));
      } else {
        ph.num_op  = Placeholder::NumOp::EQ;
        ph.num_val = std::stod(content);
      }
    }
  } catch (...) {
    ph.kind    = Placeholder::Kind::Alternation;
    ph.choices = {content};
  }
  return ph;
}

bool Pattern::matches(const std::string& text) const {
  if (raw.empty()) return true;
  switch (type) {
    case Type::Literal:
      return contains(text, raw, true);
    case Type::Regex:
      return valid_ && std::regex_search(text, compiled_regex_);
    case Type::Template: {
      if (placeholders_.empty()) {
        return fixed_parts_.empty() || contains(text, fixed_parts_[0], true);
      }
      const std::string& anchor = fixed_parts_[0];
      if (!anchor.empty()) {
        size_t search_pos = 0;
        while (search_pos < text.size()) {
          auto it = std::search(
            text.begin() + static_cast<ptrdiff_t>(search_pos), text.end(),
            anchor.begin(), anchor.end(),
            [](char a, char b) { return std::toupper(a) == std::toupper(b); });
          if (it == text.end()) return false;
          size_t occ = static_cast<size_t>(it - text.begin());
          if (matchTemplate(text, occ)) return true;
          search_pos = occ + 1;
        }
        return false;
      }
      for (size_t i = 0; i < text.size(); i++) {
        if (matchTemplate(text, i)) return true;
      }
      return false;
    }
  }
  return false;
}

size_t Pattern::matchTemplateEnd(const std::string& text, size_t pos) const {
  static constexpr size_t kNone = std::string::npos;
  for (size_t i = 0; i < placeholders_.size(); i++) {
    const std::string& fixed = fixed_parts_[i];
    if (!fixed.empty()) {
      if (pos + fixed.size() > text.size()) return kNone;
      auto it = std::search(
        text.begin() + static_cast<ptrdiff_t>(pos),
        text.begin() + static_cast<ptrdiff_t>(pos + fixed.size()),
        fixed.begin(), fixed.end(),
        [](char a, char b) { return std::toupper(a) == std::toupper(b); });
      if (it != text.begin() + static_cast<ptrdiff_t>(pos)) return kNone;
      pos += fixed.size();
    }
    if (pos > text.size()) return kNone;

    const Placeholder& ph = placeholders_[i];
    if (ph.kind == Placeholder::Kind::Wildcard) {
      while (pos < text.size() && !std::isspace(static_cast<unsigned char>(text[pos]))) {
        pos++;
      }
    } else if (ph.kind == Placeholder::Kind::Numeric) {
      while (pos < text.size() && std::isspace(static_cast<unsigned char>(text[pos]))) {
        pos++;
      }
      const char* start_ptr = text.c_str() + pos;
      char* end_ptr = nullptr;
      double val = std::strtod(start_ptr, &end_ptr);
      if (!end_ptr || end_ptr == start_ptr) return kNone;
      pos = static_cast<size_t>(end_ptr - text.c_str());
      bool ok = false;
      switch (ph.num_op) {
        case Placeholder::NumOp::GT:    ok = val > ph.num_val; break;
        case Placeholder::NumOp::LT:    ok = val < ph.num_val; break;
        case Placeholder::NumOp::GEQ:   ok = val >= ph.num_val; break;
        case Placeholder::NumOp::LEQ:   ok = val <= ph.num_val; break;
        case Placeholder::NumOp::EQ:    ok = (val == ph.num_val); break;
        case Placeholder::NumOp::RANGE: ok = (val >= ph.num_val && val <= ph.num_val2); break;
      }
      if (!ok) return kNone;
    } else {
      bool found = false;
      for (const auto& choice : ph.choices) {
        if (pos + choice.size() > text.size()) continue;
        auto it = std::search(
          text.begin() + static_cast<ptrdiff_t>(pos),
          text.begin() + static_cast<ptrdiff_t>(pos + choice.size()),
          choice.begin(), choice.end(),
          [](char a, char b) { return std::toupper(a) == std::toupper(b); });
        if (it != text.begin() + static_cast<ptrdiff_t>(pos)) continue;
        pos += choice.size();
        found = true;
        break;
      }
      if (!found) return kNone;
    }
  }

  const std::string& trailing = fixed_parts_.back();
  if (!trailing.empty()) {
    if (pos + trailing.size() > text.size()) return kNone;
    auto it = std::search(
      text.begin() + static_cast<ptrdiff_t>(pos),
      text.begin() + static_cast<ptrdiff_t>(pos + trailing.size()),
      trailing.begin(), trailing.end(),
      [](char a, char b) { return std::toupper(a) == std::toupper(b); });
    if (it != text.begin() + static_cast<ptrdiff_t>(pos)) return kNone;
    pos += trailing.size();
  }
  return pos;
}

bool Pattern::matchTemplate(const std::string& text, size_t pos) const {
  return matchTemplateEnd(text, pos) != std::string::npos;
}

std::vector<std::pair<size_t, size_t>> Pattern::findAll(const std::string& text) const {
  std::vector<std::pair<size_t, size_t>> results;
  if (raw.empty()) return results;

  switch (type) {
    case Type::Literal: {
      for (size_t pos : find(text, raw, true)) {
        results.push_back({pos, raw.size()});
      }
      break;
    }
    case Type::Regex: {
      if (!valid_) break;
      try {
        std::sregex_iterator it(text.begin(), text.end(), compiled_regex_);
        std::sregex_iterator end_it;
        for (; it != end_it; ++it) {
          size_t len = static_cast<size_t>(it->length());
          if (len == 0) continue;
          results.push_back({static_cast<size_t>(it->position()), len});
        }
      } catch (...) {}
      break;
    }
    case Type::Template: {
      if (placeholders_.empty()) {
        if (!fixed_parts_.empty() && !fixed_parts_[0].empty()) {
          for (size_t pos : find(text, fixed_parts_[0], true)) {
            results.push_back({pos, fixed_parts_[0].size()});
          }
        }
        break;
      }
      const std::string& anchor = fixed_parts_[0];
      if (!anchor.empty()) {
        size_t sp = 0;
        while (sp < text.size()) {
          auto it = std::search(
            text.begin() + static_cast<ptrdiff_t>(sp), text.end(),
            anchor.begin(), anchor.end(),
            [](char a, char b) { return std::toupper(a) == std::toupper(b); });
          if (it == text.end()) break;
          size_t occ = static_cast<size_t>(it - text.begin());
          size_t end = matchTemplateEnd(text, occ);
          if (end != std::string::npos) {
            results.push_back({occ, end - occ});
            sp = (end > occ) ? end : occ + 1;
          } else {
            sp = occ + 1;
          }
        }
      } else {
        for (size_t i = 0; i < text.size(); ) {
          size_t end = matchTemplateEnd(text, i);
          if (end != std::string::npos && end > i) {
            results.push_back({i, end - i});
            i = end;
          } else {
            i++;
          }
        }
      }
      break;
    }
  }
  return results;
}

}  // namespace log_view
