from collections import deque
from app.tss import get_from_tss, tss_keys
import asyncio
import json


class Datastore:
    def __init__(self):
        self.new_data_deques = {}
        self.cached_deque = {}

    async def append(self, dqs, key, value):
        if key not in dqs:
            dqs[key] = deque(maxlen=100)
        dqs[key].append(value)

    async def get(self, key):
        return list(self.new_data_deques[key])

    async def start_polling(self):
        while True:
            for key in tss_keys:
                response = await get_from_tss(key)
                await self.append(self.new_data_deques, key, response.json())
            await asyncio.sleep(1)
            print([len(self.new_data_deques[k]) for k in self.new_data_deques.keys()])

    async def get_updates(self):
        to_send = []
        for key in self.new_data_deques.keys():
            dq = self.new_data_deques[key]
            while dq:
                update_data = dq.popleft()
                await self.append(self.cached_deque, key, update_data)
                to_send.append(json.dumps(update_data))
        return to_send
