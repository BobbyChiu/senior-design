import socket


class NetstringClient:
    """
    TCP client class for communicating to the server with netstring.

    You must use the context manager keyword `with`.
    """

    def __init__(self, host: str='localhost', port: int=12369):
        """
        Initialize with an address host and port.
        """

        self.addr = (host, port)

    def __enter__(self):
        self.socket = socket.create_connection(address=self.addr)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.socket.close()

    def send_data(self, data: bytes) -> None:
        """
        Send data as a netstring.
        Blocking.
        """

        netstring = NetstringClient._data_to_netstring(data)

        self.socket.sendall(netstring)

    def receive_data(self) -> bytes:
        """
        Receive data from netstring.
        Blocking.
        """

        # Get length
        length_ba = bytearray()
        while True:
            byte = self.socket.recv(1)
            if byte == b':':
                break
            else:
                length_ba += byte
        length = int(length_ba)

        # Get data
        data = self.socket.recv(length)

        # Discard comma
        self.socket.recv(1)

        return data

    @staticmethod
    def _data_to_netstring(data: bytes) -> bytes:
        length = bytes(str(len(data)), encoding='ascii')
        netstring = length + b':' + bytes(data) + b','
        return netstring

    @staticmethod
    def _netstring_to_data(netstring: bytes) -> bytes:
        i = 0

        # Get length
        length_ba = bytearray()
        while True:
            byte = bytes(chr(netstring[i]), encoding='ascii')
            i += 1
            if byte == b':':
                break
            else:
                length_ba += byte
        length = int(length_ba)

        # Get data
        data = netstring[i:i + length]
        return data


if __name__ == '__main__':
    # print(NetstringClient._netstring_to_data(b'49:Here are instructions for performing this action.,'))
    # print('Performing action in client.')
    # print(NetstringClient._data_to_netstring(b'Client is done with performing the action.'))

    host = 'localhost'
    port = 12369

    print(f'Trying to connect to {(host, port)}.')

    with NetstringClient(host, port) as nsc:
        print('Connected.')
        while True:
            data = nsc.receive_data()
            print(data)
            nsc.send_data(b'done')

    print('Closed connection.')
