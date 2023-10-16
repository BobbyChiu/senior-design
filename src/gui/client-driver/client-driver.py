import socket
from enum import Enum


HOST = 'localhost'
PORT = 12369


def blocking_driver() -> None:
    addr = (HOST, PORT)
    print(f'Trying to connect to {addr}.')
    with socket.create_connection(address=addr) as s:
        print('Connected.')

        s.sendall(b'calibrate')
        print('Calibrating.')

        s.sendall(b'scan')
        print('Scanning.')

        s.sendall(b'process')
        print('Processing.')

        s.sendall(b'generate')
        print('Generating.')

        s.sendall(b'done')
        print('Done.')


if __name__ == '__main__':
    blocking_driver()
