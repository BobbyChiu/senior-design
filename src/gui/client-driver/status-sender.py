import socket


class StatusSender:
    """
    TCP client class for messaging the current status to the server.

    You must use the context manager keyword `with`.
    """

    class StatusMessage(bytes):
        """
        Derived class of bytes to limit what messages can be sent.
        """
        pass

    msg_calibrate = StatusMessage(b'calibrateXXXXXXX')
    msg_scan = StatusMessage(b'scanXXXXXXXXXXXX')
    msg_process = StatusMessage(b'processXXXXXXXXX')
    msg_generate = StatusMessage(b'generateXXXXXXXX')
    msg_done = StatusMessage(b'doneXXXXXXXXXXXX')


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

    def send_message(self, msg: StatusMessage) -> None:
        """
        Send a supported StatusMessage.
        Blocking.
        """

        self.socket.sendall(msg)


if __name__ == '__main__':
    host = 'localhost'
    port = 12369

    print(f'Trying to connect to {(host, port)}.')

    with StatusSender(host, port) as ns:
        print('Connected.')

        ns.send_message(StatusSender.msg_calibrate)
        print('Calibrating.')

        ns.send_message(StatusSender.msg_scan)
        print('Scanning.')

        ns.send_message(StatusSender.msg_process)
        print('Processing.')

        ns.send_message(StatusSender.msg_generate)
        print('Generating.')

        ns.send_message(StatusSender.msg_done)
        print('Done.')

    print('Closed connection.')
