# log_view

log_viewer is an ncurses text UI for viewing rosout logs.

![](https://raw.githubusercontent.com/wiki/hatchbed/log_view/log_viewer.gif)

### Motivation
Receive, filter, and navigate through all published rosout log messages from within a terminal to support debugging a
robot live.

### Problem
Many rosout log messages don't get printed to the terminal in the normal course when there are lots of nodes/messages.
There is also not a convienient way to filter and navigate through them in real time.

[swri_console](https://github.com/swri-robotics/swri_console) provides most of the desired functionality very well, but
is Qt based and requires a graphical interface.

### Solution
log_viewer is very similar in concept and design to swri_console, but uses ncurses instead of Qt for the interface.

Log messages can be filtered based on:
 - log level
 - node
 - whitelist text filter
 - exclude text filter

log_viewer also supports text searches and jumping from match to match.

Log messages are copied to the clipboard by selecting them.

Like swri_console, log_viewer doesn't need roscore to start and will automatically connect/reconnect with roscore when it
becomes available.

##### Keybindings
```
CTRL-c       Exit log viewer
CTRL-h       Show/hide help screen

Up Arrow     Scroll up
Down Arrow   Scroll down
Left Arrow   Scroll left
Right Arrow  Scroll right
Page Up      Scroll up a screen
Page Down    Scroll down a screen
Home         Scroll to first message
End          Scroll to last message and follow

Tab          Change focus to next input

F1           Show/hide debug level
F2           Show/hide info level
F3           Show/hide warning level
F4           Show/hide error level
F5           Show hide fatal level
F7           Enable/disable node filter
CTRL-a       Select all log lines and copy to clipboard
CTRL-n       Show/hide node selection
CTRL-s       Search for matching string
CTRL-x       Clear search
Backspace    Prev match
Enter        Next match
CTRL-e       Enable/disable text exclude filter
CTRL-f       Enable/disable text include filter
```

##### Mouse Support:

There is limited mouse support for selecting log messages and enabling/disabling the log level and node filters.  Due to
a bug in the currently distributed version of ncurses, mousewheel scrolling only works in the up direction.

![](https://raw.githubusercontent.com/wiki/hatchbed/log_view/log_viewer2.gif)

### Possible Improvements:
 - Regular expression support
 - ROS2 support
 - Support loading logs directly from bag files
 - Support loading logs from ROS log directory
 - Refine UI design and keybindings to be more consistent/intuitive

