"""Implementation of MainGUI Class for Emobided AI Platform"""

import tkinter as tk
from tkinter import messagebox


class MainGUI:
    def __init__(self) -> None:
        # Configure Root
        self.root = tk.Tk()
        self.configure_root()

        # Configure widgets
        self.build_label1()
        self.build_textbox1()
        self.build_check1()
        self.build_button1()
        self.build_menu1()
        self.build_clear_button1()
        self.root.mainloop()

    def configure_root(self):
        self.root.geometry("500x500")
        self.root.title("Self Drive")

    def build_label1(self):
        self.label1 = tk.Label(
            self.root, text="Embodied AI Platform Control", font=("Arial", 18)
        )
        self.label1.pack(padx=20, pady=20)

    def build_textbox1(self):
        self.textbox1 = tk.Text(self.root, height=3, font=("arial", 16))
        self.textbox1.bind(
            "<KeyPress>", self.shortcut
        )  # Add "ctrl + enter" shortcut to trigger message
        self.textbox1.pack(padx=10, pady=10)

    def build_check1(self):
        self.check1_state = tk.IntVar()
        self.check1 = tk.Checkbutton(
            self.root,
            text="Show Messagebox",
            font=("Arial", 18),
            variable=self.check1_state,
        )
        self.check1.pack(padx=10, pady=10)

    def build_button1(self):
        self.button1 = tk.Button(
            self.root,
            text="Show Message",
            font=("Arial", 18),
            command=self.show_message,
        )
        self.button1.pack(padx=10, pady=10)

    def show_message(self):
        if self.check1_state.get() == 0:
            print(self.textbox1.get("1.0", tk.END))
        else:
            messagebox.showinfo(
                title="Message", message=self.textbox1.get("1.0", tk.END)
            )

    def shortcut(self, event):
        if event.state == 4 and event.keysym == "Return":
            self.show_message()

    def build_menu1(self):
        self.menubar1 = tk.Menu(self.root)
        self.filemenu1 = tk.Menu(self.menubar1, tearoff=0)
        self.filemenu1.add_command(label="Close", command=exit)

        self.menubar1.add_cascade(menu=self.filemenu1, label="File")
        self.root.config(menu=self.menubar1)

    def build_clear_button1(self):
        self.clear_button1 = tk.Button(
            self.root, text="Clear", font=("Arial", 18), command=self.clear
        )
        self.clear_button1.pack(padx=10, pady=10)

    def clear(self):
        self.textbox1.delete("1.0", tk.END)


def main():
    """Driver program to test MainGUI class."""
    MainGUI()


if __name__ == "__main__":
    main()
