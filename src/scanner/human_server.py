"""
human_server.py

Act as a server program for the scanner (client) program, main.py.
You provide command arguments via stdin.

Start this server program in one terminal and execute the following in another terminal:
`python main.py --args-from socket`
"""

import socket
from NetstringClient import NetstringClient


if __name__ == '__main__':
    host = 'localhost'
    port = 12369

    addr = (host, port)

    print(f'Starting server at {addr}')
    with socket.create_server(addr) as server:
        s, _ = server.accept()
        print(f'Received client')
        with s:
            while True:
                args = input('////////> ')
                s.sendall(NetstringClient._data_to_netstring(bytes(args, encoding='ascii')))

                response = NetstringClient._netstring_to_data(s.recv(4096)).decode('ascii')
                print(response)
                if response == '!quit':
                    break