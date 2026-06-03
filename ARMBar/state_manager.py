class StateManager:
    def __init__(self):
        self.current_page = 0
        self.max_pages = 5

    def scroll_up(self):
        self.current_page = (self.current_page + 1) % self.max_pages

    def scroll_down(self):
        self.current_page = (self.current_page - 1) % self.max_pages