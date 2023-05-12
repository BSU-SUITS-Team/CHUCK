import base64
import requests

class Message:
    def __init__(self, message, sender):
        self.message = message
        self.sender = sender
        self.image_encoded = None
        self.choices = None
    def add_image(self, image):
        # image is the filename of the image to send
        self.image_encoded = base64.b64encode(open(image, "rb").read())
    def add_choice(self, choices: list):
        # choices is a list of the options to send (str)s
        self.choices = choices

class Chat:
    """
    This is how messages are sent:
   class Message(BaseModel):
        content: str

    @router.post("/{to}/{sender}")
    def post_message(to: str, sender: str, message: Message, type: str = "text"):
        message = {sender: {"type": type, "content": message.content}}
        res = messages.get(to, None)
        if res is not None:
            res.append(message)
        else:
            messages[to] = [message]
        return 200
    """
    def __init__(self, host: str, port: int = 8000, username: str = "Anonymous"):
        self.host = host
        self.port = port
        self.messages = {}
        self.connected_users = []
        self.username = username

    def send(self, message: Message, to: str):
        # sends text than images than choices
        print(f"posting {message.message} to {to}")
        requests.post(f"http://{self.host}:{self.port}/chat/{to}/{self.username}?type=text", json={"content": message.message})
        if message.image_encoded is not None:
            requests.post(f"http://{self.host}:{self.port}/chat/{to}/{self.username}?type=image", json={"content": message.image_encoded})
        if message.choices is not None:
            requests.post(f"http://{self.host}:{self.port}/chat/{to}/{self.username}?type=choice", json={"content": str(message.choices)})

    def get_messages(self, name: str):
        res = requests.get(f"http://{self.host}:{self.port}/chat/{name}")
        if res.status_code == 200:
            return res.json()
        return None
    
    def get_users(self):
        res = requests.get(f"http://{self.host}:{self.port}/chat/")
        if res.status_code == 200:
            return res.json()
        return None
    
if __name__ == "__main__":
    chat = Chat("localhost", port=8181)
    print(chat.get_users())
    print(chat.get_messages("test"))
    message = Message("Hello!", "test")
    # message.add_image("image.png")
    message.add_choice(["option1", "option2"])
    chat.send(message, "test")
    print(chat.get_messages("test"))
