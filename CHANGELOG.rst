^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package log_view
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Added subscription to /clock to display sim time on status bar.
* Updated stamp format selection to preview the stamp format change.
* Improved formatting of nodes, preferences, and details panels.
* Improved search navigation.
* Fixed exit to only require a single ctrl-c press.
* Fixed message counts for nodes when loading messages from previous sessions.
* Fixed details panel to scroll vertically when insufficient space is available.
* Fixed help panel to scroll vertically when insufficient space is available.
* Fixed preference panel to scroll vertically when insufficient space is available.
* Updated node panel scrolling to make it consistent with other panels.
* Improved selection controls.

0.1.4 (2026-05-05)
------------------
* Scope log and preference storage to the active catkin workspace via ``CMAKE_PREFIX_PATH`` (stored under ``<workspace>/.log_view/``).
* Show file paths in preferences panel; display a warning when persistence is unavailable.
* Persist logs to disk with configurable rotation and max size.
* Load persisted logs on startup.
* Add preferences panel (CTRL-k) for persistence and timestamp format settings.
* Add message details panel (CTRL-d) with per-message metadata display.
* Add shortcut to clear message history (CTRL-r).
* Show filtered log count in status bar (``logs: X of Y`` when a filter is active).
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
