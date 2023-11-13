#!/bin/sh

python3 -m venv .venv 2> /dev/null
. .venv/bin/activate
python3 -m pip install -r requirements.txt > /dev/null
python3 main.py --args-from socket
