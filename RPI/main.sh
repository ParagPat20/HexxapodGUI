#!/bin/bash
tmux kill-session
tmux new-session -d -s main "/usr/bin/python3 /home/rollo/HexxapodGUI/RPI/Main.py"
echo "Python script is running in tmux session: main"
