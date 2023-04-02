"""Project user interface."""

import tkinter as tk

def main():
    #  Window configuration
    root = tk.Tk()
    root.geometry("800x500") # Set window dimenstions
    root.title("Self Drive")

    label = tk.Label(root, text="Hello World!", font=("Arial", 18)) # Create text
    label.pack(padx=20, pady=20)  # Set label padding

    textbox = tk.Text(root, height=3, font=("arial", 16)) # Create text entry box
    textbox.pack(padx=10, pady=10) #  Set textbox padding

    button = tk.Button(root, text="Click Me!", font=("Arial", 18)) # Add a button
    button.pack(padx=10, pady=10) # Set button padding

    buttonframe = tk.Frame(root)
    buttonframe.columnconfigure(0, weight=1)
    buttonframe.columnconfigure(1, weight=1)
    buttonframe.columnconfigure(2, weight=1)

    btn1 = tk.Button(buttonframe, text="1", font=("Arial",18))
    btn1.grid(row=0, column=0, sticky=tk.W+tk.E)

    btn2 = tk.Button(buttonframe, text="2", font=("Arial",18))
    btn2.grid(row=0, column=1, sticky=tk.W+tk.E)
    
    btn3 = tk.Button(buttonframe, text="3", font=("Arial",18))
    btn3.grid(row=0, column=2, sticky=tk.W+tk.E)

    btn4 = tk.Button(buttonframe, text="4", font=("Arial",18))
    btn4.grid(row=1, column=0, sticky=tk.W+tk.E)

    btn5 = tk.Button(buttonframe, text="5", font=("Arial",18))
    btn5.grid(row=1, column=1, sticky=tk.W+tk.E)

    btn6 = tk.Button(buttonframe, text="6", font=("Arial",18))
    btn6.grid(row=1, column=2, sticky=tk.W+tk.E)

    buttonframe.pack(fill="x")

    anotherbtn = tk.Button(root, text="button 2")
    anotherbtn.place(x=200, y=200, height=100, width=100)

    
    # Run Window
    root.mainloop()

if __name__ == "__main__":
    main()
