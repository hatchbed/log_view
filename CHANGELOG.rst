^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package log_view
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
