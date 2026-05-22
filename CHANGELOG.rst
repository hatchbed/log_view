^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package log_view
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.0 (2026-05-22)
------------------
* Added subscription to /clock to display sim time on status bar.
* Updated stamp format selection to preview the stamp format change.
* Improved formatting of nodes, preferences, and details panels.
* Improved search navigation.
* Fixed exit to only require a single ctrl-c press.
* Fixed message counts for nodes when loading messages from previous sessions.

0.3.3 (2026-05-20)
------------------
* Added support for UTF8 glyphs in log messages.
* Added support for ANSI color codes in log messages.
* Fixed display for terminals limited to 8 colors.
* Fixed session boundaries to not be treated as normal log entries.
* Fixed details panel to scroll vertically when insufficient space is available.
* Fixed help panel to scroll vertically when insufficient space is available.
* Fixed preference panel to scroll vertically when insufficient space is available.
* Updated node panel scrolling to make it consistent with other panels.
* Improved selection controls.

0.3.2 (2026-05-04)
------------------
* Scope log and preference storage to the active ROS 2 workspace via ``COLCON_PREFIX_PATH`` (stored under ``<workspace>/.log_view/``).
* Show file paths in preferences panel; display a warning when persistence is unavailable.
* Contributors: Marc Alban

0.3.1 (2026-05-04)
------------------
* Persist logs to disk in ``~/.local/share/log_view/`` with configurable rotation and max size.
* Load persisted logs on startup.
* Add preferences panel (CTRL-k) for persistence and filter settings.
* Show filtered log count in status bar (``logs: X of Y`` when a filter is active).
* Contributors: Marc Alban

0.3.0 (2026-03-29)
------------------
* Add details panel with per-message metadata display.
* Enable static analysis and linting tests.
* Contributors: Marc Alban

0.2.7 (2026-02-18)
------------------
* Replace ament_target_dependencies with target_link_libraries.
* Contributors: Marc Alban

0.2.5 (2024-11-25)
------------------
* Fix mvwprintw format-security error
* Contributors: Marc Alban

0.2.4 (2024-07-24)
------------------
* Fix build error caused by mvwprintw. (`#19 <https://github.com/hatchbed/log_view/issues/19>`_)
* Contributors: Marc Alban

0.2.3 (2024-07-08)
------------------
* Use default C++ version (`#16 <https://github.com/hatchbed/log_view/issues/16>`_)
* Disable mouse move events on exit. (`#12 <https://github.com/hatchbed/log_view/issues/12>`_)
* Contributors: Marc Alban

0.2.2 (2022-07-30)
------------------
* Improve message handling so that log messages are not dropped. (`#8 <https://github.com/hatchbed/log_view/issues/8>`_)
* Contributors: Marc Alban

0.2.1 (2022-07-07)
------------------
* Remove unused boost include.
* Contributors: Marc Alban

0.2.0 (2022-07-04)
------------------
* Port to ROS2
* Contributors: Marc Alban

0.1.3 (2021-03-01)
------------------
* Prevent help screen text from wrapping.
* Make help screen modal.
* Add keybindings for selecting all nodes and inverting the node selection.
* Update README.
* Fix compiler warnings.
* Contributors: Marc Alban

0.1.2 (2020-11-26)
------------------
* Install binary log_viewer to package destination instead of global destination.
* Contributors: Marc Alban

0.1.1 (2020-11-24)
------------------
* Fixes for build farm.
* Contributors: Marc Alban

0.1.0 (2020-11-22)
------------------
* Initial working version.
* Initial code.
* Contributors: Marc Alban
