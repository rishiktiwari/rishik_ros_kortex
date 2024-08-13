# -*- coding: utf-8 -*-
"""
Created on Fri Aug  9 15:31:41 2024
@author: Devin
"""

import tkinter as tk
from PIL import Image, ImageTk

def update_label(event):
    global counter_k, counter_b
    if (counter_k + counter_b <= 1):
        if event.char == 'k':
            counter_k += 1
            label_choice_kk.config(text=f"{counter_k}")
        elif event.char == 'b':
            counter_b += 1
            label_choice_eatwell.config(text=f" {counter_b}")

def enter(event):
    print("They selected: ",counter_k,"KitKat and ",counter_b," Biscuits")
    return None



def do_backspace(event):
    global counter_k, counter_b
    counter_k =0
    counter_b = 0
    label_choice_kk.config(text=f"{counter_k}")
    label_choice_eatwell.config(text=f" {counter_b}")
    


window = tk.Tk()
window.title("KitKat or Biscuit")
counter_k = 0
counter_b = 0
label1 = tk.Label(window, text="Please Hit 'K' for KitKats and 'B' for Biscuits", font=("Arial", 24))
label1.grid(row=1,column=0)
window.geometry("1500x1000")


# Load images into file
image_kk = ImageTk.PhotoImage(Image.open('assets/KitKat.jpeg'))
label_choice_kk = tk.Label(window, text=f"{counter_k}", font=("Arial", 24))
label_choice_kk.grid(row=3,column=0)
image_eatwell = ImageTk.PhotoImage(Image.open("assets/Biscuit.png"))
label_choice_eatwell = tk.Label(window, text=f"{counter_b}", font=("Arial", 24))
label_choice_eatwell.grid(row=3,column=4)
#Create labels 
label_kk = tk.Label(window, image = image_kk)
label_kk.grid(row=4,column=0)
label_eatwell = tk.Label(window, image=image_eatwell)
label_eatwell.grid(row=4,column=4)

# Key binding and events
window.bind("<Key>", update_label)
# xxx =  window.bind("<Key>",choice(event,kk_counter))
# print(xxx)
# window.bind("<b>",choice)

# if to check if its k or b and ignore everything else
window.bind("<Return>",enter)
window.bind("<BackSpace>",do_backspace)

window.mainloop()
