@echo off

python -m venv .venv 2> NUL
call .venv/Scripts/activate
python -m pip install -r requirements.txt > NUL
python main.py --args-from socket
