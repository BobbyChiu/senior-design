import socket
from enum import Enum


HOST = 'localhost'
PORT = 12369

class MessageEnum(Enum):
    CALIBRATE = b'calibrate'
    SCAN = b'scan'
    PROCESS = b'process'
    GENERATE = b'generate'
    DONE = b'done'


def blocking_driver() -> None:
    addr = (HOST, PORT)
    print(f'Trying to connect to {addr}.')
    with socket.create_connection(address=addr) as s:
        print('Connected.')

        s.sendall(MessageEnum.CALIBRATE)
        print('Calibrating.')

        s.sendall(MessageEnum.SCAN)
        print('Scanning.')

        s.sendall(MessageEnum.PROCESS)
        print('Processing.')

        s.sendall(MessageEnum.GENERATE)
        print('Generating.')

        s.sendall(MessageEnum.DONE)
        print('Done.')


if __name__ == '__main__':
    blocking_driver()
