import asyncio
from collections import deque

from app.tss import get_from_tss, tss_keys
from app.event import Event


class Datastore:
    def __init__(self):
        self.new_data_queue = asyncio.Queue(maxsize=500)
        self.cache = {}
        self.outputcache = {}

    def print_size(self):
        print(f"cache keys {self.cache.keys()}")
        print(f"new data queue size {self.new_data_queue.qsize()}")

    async def add_event(self, key, new_event):
        await self.new_data_queue.put((key, new_event))

    async def append(self, key, value):
        if key not in self.cache:
            self.cache[key] = deque(maxlen=100)
        self.cache[key].append(value)

    async def start_polling_key(self, key):
        while True:
            response = await get_from_tss(key)
            if response.status_code == 200:
                new_event = Event.create_event(key, response.json()[key])
                if key not in self.outputcache.keys():
                    self.outputcache[key] = response.json()[key]
                    await self.add_event(key, new_event)
                else:
                    if self.outputcache[key] != response.json()[key]:
                        self.outputcache[key] = response.json()[key]
                        await self.add_event(key, new_event)

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

    async def get_all(self):
        to_return = []
        for key, value in self.cache.items():
            to_return += list(value)
        return to_return


ds = Datastore()
