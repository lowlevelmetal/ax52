#!/bin/sh
# Graphical password prompt for `sudo -A` (used because Claude Code's ! prefix has no TTY).
exec zenity --password --title="sudo: ax52 project"
