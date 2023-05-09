import json
from fastapi import APIRouter
from pydantic import BaseModel

router = APIRouter(prefix="/chat", tags=["procedures"])

# {"user": [ {from1: {type: "text", content: "message1"}}, {from2: {type: "options", content: ["opt1", "opt2", ... ]}}, ... ], ... }
messages = {}

@property
def connected_users():
    return list(messages.keys())

@router.get("/")
async def procedures():
    return connected_users


@router.get("/{name}")
def get_messages(name: str):
    res = messages.get(name, None)
    if res is not None:
        return {
            "name": name,
            "messages": res,
        }
    return 404

class Message(BaseModel):
    content: str

@router.post("/{to}/{sender}")
def post_message(to: str, sender: str, message: Message, type: str = "text"):
    """Sends a message to a user and returns 200 if successful.
    
    **NOTE:** This is not secure, it does not verify that the sender is actually the sender. This is just a proof of concept.
    """
    message = {sender: {"type": type, "content": message.content}}
    res = messages.get(to, None)
    if res is not None:
        res.append(message)
    else:
        messages[to] = [message]
    return 200