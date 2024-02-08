from app.tss import get_from_tss, tss_keys
import asyncio


class Datastore:
    def __init__(self):
        self.new_data_queue = asyncio.Queue(maxsize=100)
        self.cached_deque = {}

    async def append(self, key, value):
        if key not in self.cached_deque:
            self.cached_deque[key] = asyncio.Queue(maxsize=100)
        await self.cached_deque[key].put(value)

    async def get(self, key):
        return list(self.new_data_deques[key])

    async def start_polling_key(self, key):
        while True:
            response = await get_from_tss(key)
            if response.status_code == 200:
                await self.new_data_queue.put((key, response.json()))
            await asyncio.sleep(1)

    async def start_polling(self):
        for key in tss_keys:
            asyncio.create_task(self.start_polling_key(key))

    def make_async_gen(self):
        async def gen():
            while True:
                key, update_data = await self.new_data_queue.get()
                await self.append(key, update_data)
                yield update_data

        return gen()

    def get_all(self):
        to_return = []
        for key, value in self.cached_deque.items():
            to_return += list(value)
        return to_return

