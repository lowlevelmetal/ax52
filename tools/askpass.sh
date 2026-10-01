#!/bin/sh
# SUDO_ASKPASS helper: graphical password prompt for `sudo -A`, for running the
# root-only tools from environments without a terminal (IDEs, automation).
exec zenity --password --title="sudo: ax52"
