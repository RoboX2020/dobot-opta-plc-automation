import customtkinter as ctk
import threading
import sys
import time

# Attempt to import dobot3 logic (you may need to tweak dobot3.py to be fully non-blocking)
try:
    import dobot3
except ImportError:
    dobot3 = None

# Set UI Theme
ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

class DobotGUI(ctk.CTk):
    def __init__(self):
        super().__init__()

        self.title("Dobot Opta PLC - Control Dashboard")
        self.geometry("900x600")
        
        # Grid layout
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)

        # ================= Sidebar =================
        self.sidebar_frame = ctk.CTkFrame(self, width=200, corner_radius=0)
        self.sidebar_frame.grid(row=0, column=0, sticky="nsew")
        self.sidebar_frame.grid_rowconfigure(6, weight=1)

        self.logo_label = ctk.CTkLabel(self.sidebar_frame, text="🤖 Dobot Control", font=ctk.CTkFont(size=20, weight="bold"))
        self.logo_label.grid(row=0, column=0, padx=20, pady=(20, 10))

        self.connect_btn = ctk.CTkButton(self.sidebar_frame, text="Connect Dobot", command=self.connect_dobot)
        self.connect_btn.grid(row=1, column=0, padx=20, pady=10)

        self.home_btn = ctk.CTkButton(self.sidebar_frame, text="Home Arm", command=self.home_arm)
        self.home_btn.grid(row=2, column=0, padx=20, pady=10)

        self.play_btn = ctk.CTkButton(self.sidebar_frame, text="▶ Play Mode", fg_color="green", hover_color="darkgreen", command=self.play_mode)
        self.play_btn.grid(row=3, column=0, padx=20, pady=10)
        
        self.stop_btn = ctk.CTkButton(self.sidebar_frame, text="🛑 Stop / Disconnect", fg_color="red", hover_color="darkred", command=self.disconnect_dobot)
        self.stop_btn.grid(row=4, column=0, padx=20, pady=10)

        # ================= Main View =================
        self.main_frame = ctk.CTkFrame(self)
        self.main_frame.grid(row=0, column=1, padx=20, pady=20, sticky="nsew")
        self.main_frame.grid_columnconfigure(0, weight=1)
        self.main_frame.grid_rowconfigure(1, weight=1)

        self.status_label = ctk.CTkLabel(self.main_frame, text="System Status: OFFLINE", font=ctk.CTkFont(size=18, weight="bold"), text_color="red")
        self.status_label.grid(row=0, column=0, padx=20, pady=20, sticky="w")

        # Tabs for different controls
        self.tabview = ctk.CTkTabview(self.main_frame)
        self.tabview.grid(row=1, column=0, padx=20, pady=10, sticky="nsew")
        self.tabview.add("Console Logs")
        self.tabview.add("Teach Menu")
        self.tabview.add("PLC Settings")

        # Console Tab
        self.console_textbox = ctk.CTkTextbox(self.tabview.tab("Console Logs"), width=600, height=350, font=("Consolas", 12))
        self.console_textbox.pack(fill="both", expand=True, padx=10, pady=10)
        self.log("GUI Initialized. Waiting for connection...")

        # Teach Tab
        self.teach_frame = ctk.CTkFrame(self.tabview.tab("Teach Menu"))
        self.teach_frame.pack(fill="both", expand=True, padx=10, pady=10)
        
        self.pos_name_entry = ctk.CTkEntry(self.teach_frame, placeholder_text="Position Name (e.g. pick_a)")
        self.pos_name_entry.grid(row=0, column=0, padx=10, pady=10)
        
        self.teach_btn = ctk.CTkButton(self.teach_frame, text="Teach Position", command=self.teach_position)
        self.teach_btn.grid(row=0, column=1, padx=10, pady=10)

        self.list_pos_btn = ctk.CTkButton(self.teach_frame, text="List Positions", command=self.list_positions)
        self.list_pos_btn.grid(row=1, column=0, padx=10, pady=10)

        # Redirect stdout to the console
        sys.stdout = self

    def write(self, text):
        self.log(text.strip())

    def flush(self):
        pass

    def log(self, message):
        if message:
            self.console_textbox.insert("end", message + "\n")
            self.console_textbox.see("end")

    def connect_dobot(self):
        self.status_label.configure(text="System Status: CONNECTING...", text_color="yellow")
        self.update()
        if dobot3:
            try:
                dobot3.load_data()
                dobot3.connect()
                self.status_label.configure(text="System Status: ONLINE", text_color="green")
                self.log("Dobot connected successfully.")
            except Exception as e:
                self.log(f"Connection failed: {e}")
                self.status_label.configure(text="System Status: ERROR", text_color="red")
        else:
            self.log("[SIMULATION] Dobot Connected.")
            self.status_label.configure(text="System Status: ONLINE (SIM)", text_color="green")

    def home_arm(self):
        self.log("Homing arm...")
        if dobot3:
            threading.Thread(target=dobot3.home_arm, daemon=True).start()
        else:
            self.log("[SIMULATION] Homing complete.")

    def play_mode(self):
        self.log("Starting Play Mode... Waiting for PLC triggers.")
        if dobot3:
            threading.Thread(target=dobot3.play, daemon=True).start()
        else:
            self.log("[SIMULATION] Play Mode running...")

    def disconnect_dobot(self):
        self.log("Disconnecting and stopping conveyors...")
        if dobot3:
            dobot3.disconnect()
        self.status_label.configure(text="System Status: OFFLINE", text_color="red")

    def teach_position(self):
        name = self.pos_name_entry.get().strip()
        if not name:
            self.log("Please enter a position name.")
            return
        self.log(f"Teaching position: '{name}'. Please move the arm and check console.")
        # Note: dobot3.teach_position uses input(), so in a real GUI, you'd replace it with a button press event
        if dobot3:
            self.log("WARNING: teach_position requires console input in current dobot3.py architecture.")
        else:
            self.log(f"[SIMULATION] Saved position {name} at current coordinates.")

    def list_positions(self):
        if dobot3:
            dobot3.list_positions()
        else:
            self.log("[SIMULATION] Positions: start, pick_a, drop_b")

if __name__ == "__main__":
    app = DobotGUI()
    app.mainloop()
