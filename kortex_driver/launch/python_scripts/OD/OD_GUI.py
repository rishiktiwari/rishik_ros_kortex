"""
Created on: 14th Aug 2024
Created by: Devin
Revised by: Rishik
"""

from os import path as os_path
from time import sleep

# from tkinter.ttk import *
import tkinter as tk
from PIL import Image, ImageTk

from OD_main import ODMain



class OD_GUI:
    def __init__(self):
        self.ASSETS_DIR = os_path.dirname(os_path.abspath(__file__)) + '/assets/'
        self.STATUS_LABELS = {
            'default': 'Please select KitKat or Biscuits (max 2)',
            'busy': 'Processing request...',
            'err': 'Request failed :(',
            'done': 'Please collect your order, thanks :)'
        }
        self.counter_k = 0
        self.counter_b = 0
        self.is_busy = False
        
        self.window = tk.Tk()
        self.window.title("Kinova Dual Arm Demo 2024 - VU, Melbourne")
        self.window.geometry("1280x720")
        self.window.configure(bg="#FFFFFF")
        
        self.statusLabel = tk.Label(self.window, text=self.STATUS_LABELS['default'], font=("Arial", 28, "bold"), bg="#FFFFFF")
        self.statusLabel.grid(row=0, column=1, columnspan=2, pady=60, padx=60)

        #Create labels 
        self.label_choice_kk = tk.Label(self.window, text=f"{self.counter_k}", font=("Arial", 48), bg="#FFFFFF")
        self.label_choice_kk.grid(row=1, column=1, columnspan=2, pady=10)
        self.label_choice_biscuit = tk.Label(self.window, text=f"{self.counter_b}", font=("Arial", 48), bg="#FFFFFF")
        self.label_choice_biscuit.grid(row=1, column=3, columnspan=2, pady=10)
        
        # Load images into file
        self.image_kk = ImageTk.PhotoImage(Image.open(self.ASSETS_DIR + "KitKat.jpeg").resize((300,200)))
        label_kk = tk.Label(self.window, image = self.image_kk, compound="center")
        label_kk.grid(row=2,column=1,columnspan=2)
        self.image_biscuit = ImageTk.PhotoImage(Image.open(self.ASSETS_DIR + "Biscuit.png").resize((200,200)))
        label_biscuit = tk.Label(self.window, image=self.image_biscuit, compound="center")
        label_biscuit.grid(row=2,column=3,columnspan=2)

        # Key binding and events
        self.window.bind("<Key>", self.update_label)

        # if to check if its k or b and ignore everything else
        self.window.bind("<Return>", self.submit_request)
        self.window.bind("<BackSpace>", self.reset)

        # launch robotic arm controllers
        self.armControllers = ODMain()



    def update_label(self, event):
        if (not self.is_busy and self.counter_k + self.counter_b <= 1):
            if event.char == 'k':
                self.counter_k += 1
                self.label_choice_kk.config(text=str(self.counter_k))
            elif event.char == 'b':
                self.counter_b += 1
                self.label_choice_biscuit.config(text=str(self.counter_b))



    def submit_request(self, _) -> None:
        print("---\nRequested:\n\t%d x KitKat\n\t%d x Biscuit" % (self.counter_k, self.counter_b))
        if(self.counter_k + self.counter_b == 0):
            print('request ignored')
            return None
        
        self.is_busy = True
        self.statusLabel.config(text=self.STATUS_LABELS['busy'], foreground="#FFBF00")
        self.statusLabel.update_idletasks()
        # sleep(3)

        if self.armControllers.executeRequest(self.counter_k, self.counter_b):
            self.statusLabel.config(text=self.STATUS_LABELS['done'], foreground="#00CC00")
            self.statusLabel.update_idletasks()
        else:
            self.statusLabel.config(text=self.STATUS_LABELS['err'], foreground="#FF0000")
            self.statusLabel.update_idletasks()

        self.statusLabel.config(text=self.STATUS_LABELS['done'], foreground="#00CC00")
        self.statusLabel.update_idletasks()
        sleep(5)
        
        self.is_busy = False
        self.reset()
        self.statusLabel.config(text=self.STATUS_LABELS['default'], foreground="#000000")
        return None



    def reset(self, _ = None):
        if self.is_busy: # ignore input if already executing action
            return None
        self.counter_k =0
        self.counter_b = 0
        self.label_choice_kk.config(text=str(self.counter_k))
        self.label_choice_biscuit.config(text=str(self.counter_b))
    


if __name__ == "__main__":
    guiObj = OD_GUI()
    guiObj.window.mainloop()
