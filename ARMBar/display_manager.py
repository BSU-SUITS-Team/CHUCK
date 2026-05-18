import tkinter as tk

class DisplayManager:
    def __init__(self):
        self.root = tk.Tk()
        self.root.attributes("-fullscreen", True)
        self.root.configure(bg="black")
        self.root.title("Armbar Display")
        self.root.geometry("400x300")
        self.root.bind("<Escape>", lambda e: self.root.destroy())
        # This code will help the display of the Arm Bar

        # Title
        self.title_label = tk.Label(self.root, text="Armbar System", font=("Arial", 16, "bold"))
        self.title_label.pack(pady=5)

        # Page title
        self.page_label = tk.Label(self.root, text="Page 0", font=("Arial", 14))
        self.page_label.pack(pady=5)

        # Content area
        self.content = tk.Label(self.root, text="", font=("Arial", 12), justify="left")
        self.content.pack(pady=10)

        # Status bar
        self.status = tk.Label(self.root, text="Status: OK", fg="green")
        self.status.pack(side="bottom", fill="x")

        # Example pages
        self.pages = [
            "Numeric Data:\nTemp: 72\nHR: 80",
            "System Data:\nCPU: 40%\nRAM: 60%",
            "Warnings:\nNone",
            "Errors:\nNone"
        ]

    def update(self, page_index):
        self.page_label.config(text=f"Page {page_index}")
        self.content.config(text=self.pages[page_index])

    def show_warning(self, msg):
        self.status.config(text=f"WARNING: {msg}", fg="orange")

    def show_error(self, msg):
        self.status.config(text=f"ERROR: {msg}", fg="red")

    def show_ok(self):
        self.status.config(text="Status: OK", fg="green")

    def run(self):
        self.root.mainloop()
class DisplayManager:
    def update(self, page):
        print(f"Displaying page {page}")

    def show_warning(self, msg):
        print(f"WARNING: {msg}")

        # sudo nano /etc/systemd/system/chuck.service
        #sudo systemctl daemon-reload
        #sudo systemctl enable chuck.service
        #sudo systemctl start chuck.service
        #sudo reboot
        #sudo chown -R pi /home/pi/CHUCK-main
        #python3 /home/pi/CHUCK-main/ARMBar/main.py