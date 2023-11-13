from NetstringClient import NetstringClient
import shlex


class ArgSource:
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        pass

    def command_generator(self) -> list[str]:
        if False:
            yield

    def output_string(self, string: str) -> None:
        pass


class ScriptSource(ArgSource):
    def __init__(self, input_streamlike: list[list[str]]):
        self._input_streamlike = input_streamlike

    def command_generator(self) -> list[str]:
        for command in self._input_streamlike:
            yield command

    def output_string(self, string: str) -> None:
        print(string)


class SocketSource(ArgSource):
    def __init__(self, host: str, port: int):
        self._host = host
        self._port = port

    def __enter__(self):
        self._client = NetstringClient(self._host, self._port)
        self._client.__enter__()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self._client.__exit__(exc_type, exc_val, exc_tb)

    def command_generator(self) -> list[str]:
        while True:
            data = self._client.receive_data()
            string = data.decode(encoding='ascii')
            args = shlex.split(string)
            yield args

    def output_string(self, string: str) -> None:
        self._client.send_data(bytes(string, encoding='ascii'))
