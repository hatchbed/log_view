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

#include <log_view/preferences.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace log_view {

static const char* kFmtSeconds   = "seconds";
static const char* kFmtElapsed   = "elapsed";
static const char* kFmtTimeOfDay = "time_of_day";

std::string Preferences::workspaceDataDir(std::string* error) {
  const char* colcon = std::getenv("COLCON_PREFIX_PATH");
  if (!colcon || colcon[0] == '\0') {
    if (error) *error = "COLCON_PREFIX_PATH not set";
    return "";
  }
  std::string first_entry;
  std::istringstream ss(colcon);
  std::getline(ss, first_entry, ':');
  std::filesystem::path install_dir(first_entry);
  if (!std::filesystem::is_directory(install_dir)) {
    if (error) *error = "workspace dir not found";
    return "";
  }
  return (install_dir.parent_path() / ".log_view").string();
}

bool Preferences::load() {
  workspace_dir = workspaceDataDir(&workspace_error);
  if (workspace_dir.empty()) {
    persist_logs = false;
    persist_filters = false;
    return false;
  }

  std::string path = workspace_dir + "/preferences.yaml";

  try {
    YAML::Node cfg = YAML::LoadFile(path);

    if (cfg["timestamp_format"]) {
      std::string fmt = cfg["timestamp_format"].as<std::string>();
      if (fmt == kFmtElapsed) {
        timestamp_format = TimestampFormat::ELAPSED;
      } else if (fmt == kFmtTimeOfDay) {
        timestamp_format = TimestampFormat::TIME_OF_DAY;
      } else {
        timestamp_format = TimestampFormat::SECONDS;
      }
    }

    if (cfg["persist_filters"]) {
      persist_filters = cfg["persist_filters"].as<bool>();
    }

    if (cfg["persist_logs"]) {
      persist_logs = cfg["persist_logs"].as<bool>();
    }
    if (cfg["log_rotate_size"]) {
      log_rotate_size = cfg["log_rotate_size"].as<size_t>();
    }
    if (cfg["log_max_size"]) {
      log_max_size = cfg["log_max_size"].as<size_t>();
    }

    if (persist_filters && cfg["filters"]) {
      auto f = cfg["filters"];

      if (f["log_levels"]) {
        auto lvl = f["log_levels"];
        if (lvl["debug"]) filters.debug = lvl["debug"].as<bool>();
        if (lvl["info"])  filters.info  = lvl["info"].as<bool>();
        if (lvl["warn"])  filters.warn  = lvl["warn"].as<bool>();
        if (lvl["error"]) filters.error = lvl["error"].as<bool>();
        if (lvl["fatal"]) filters.fatal = lvl["fatal"].as<bool>();
      }

      if (f["node_filter_enabled"]) {
        filters.node_filter_enabled = f["node_filter_enabled"].as<bool>();
      }
      if (f["filter_pattern"]) {
        filters.filter_pattern = f["filter_pattern"].as<std::string>();
      }
      if (f["exclude_pattern"]) {
        filters.exclude_pattern = f["exclude_pattern"].as<std::string>();
      }
      if (f["node_whitelist"]) {
        for (const auto& node : f["node_whitelist"]) {
          filters.node_whitelist.insert(node.as<std::string>());
        }
      }
    }

    return true;
  } catch (const std::exception&) {
    return false;
  }
}

void Preferences::save() const {
  if (workspace_dir.empty()) return;
  std::string path = workspace_dir + "/preferences.yaml";
  try {
    std::filesystem::create_directories(workspace_dir);
  } catch (...) {
    return;
  }

  YAML::Emitter out;
  out << YAML::BeginMap;

  switch (timestamp_format) {
    case TimestampFormat::ELAPSED:
      out << YAML::Key << "timestamp_format" << YAML::Value << kFmtElapsed;
      break;
    case TimestampFormat::TIME_OF_DAY:
      out << YAML::Key << "timestamp_format" << YAML::Value << kFmtTimeOfDay;
      break;
    default:
      out << YAML::Key << "timestamp_format" << YAML::Value << kFmtSeconds;
      break;
  }

  out << YAML::Key << "persist_filters" << YAML::Value << persist_filters;
  out << YAML::Key << "persist_logs"    << YAML::Value << persist_logs;
  out << YAML::Key << "log_rotate_size" << YAML::Value << log_rotate_size;
  out << YAML::Key << "log_max_size"    << YAML::Value << log_max_size;

  if (persist_filters) {
    out << YAML::Key << "filters" << YAML::Value << YAML::BeginMap;

    out << YAML::Key << "log_levels" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "debug" << YAML::Value << filters.debug;
    out << YAML::Key << "info"  << YAML::Value << filters.info;
    out << YAML::Key << "warn"  << YAML::Value << filters.warn;
    out << YAML::Key << "error" << YAML::Value << filters.error;
    out << YAML::Key << "fatal" << YAML::Value << filters.fatal;
    out << YAML::EndMap;

    out << YAML::Key << "node_filter_enabled" << YAML::Value << filters.node_filter_enabled;
    out << YAML::Key << "filter_pattern"      << YAML::Value << filters.filter_pattern;
    out << YAML::Key << "exclude_pattern"     << YAML::Value << filters.exclude_pattern;

    out << YAML::Key << "node_whitelist" << YAML::Value << YAML::BeginSeq;
    for (const auto& name : filters.node_whitelist) {
      out << name;
    }
    out << YAML::EndSeq;

    out << YAML::EndMap;
  }

  out << YAML::EndMap;

  std::ofstream file(path);
  if (file.is_open()) {
    file << out.c_str() << "\n";
  }
}

}  // namespace log_view
