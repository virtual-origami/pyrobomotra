import asyncio, socket


class HealthServer:
    def __init__(self, config, event_loop):
        self.host = config["host"]
        self.port = int(config["port"])
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.bind((self.host, self.port))
        self.server.listen(8)
        self.server.setblocking(False)
        self.event_loop = event_loop

    @staticmethod
    async def handle_client(client):
        loop = asyncio.get_event_loop()
        response = 'OK\n'
        await loop.sock_sendall(client, response.encode('utf8'))
        client.close()

    async def server_loop(self):
        client, _ = await self.event_loop.sock_accept(self.server)
        self.event_loop.create_task(self.handle_client(client))

