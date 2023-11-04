import socket
import time
from enum import Enum


HOST = 'localhost'
PORT = 12369


def blocking_driver() -> None:
    addr = (HOST, PORT)
    print(f'Trying to connect to {addr}.')
    with socket.create_connection(address=addr) as s:
        print('Connected.')

        s.sendall(b'calibrateXXXXXXX')
        print('Calibrating.')

        s.sendall(b'scanXXXXXXXXXXXX')
        print('Scanning.')

        s.sendall(b'processXXXXXXXXX')
        print('Processing.')

        s.sendall(b'generateXXXXXXXX')
        print('Generating.')

        s.sendall(b'doneXXXXXXXXXXXX')
        print('Done.')


if __name__ == '__main__':
    #time.sleep(3)
    blocking_driver()
