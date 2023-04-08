"""Implementation of MainGUI Class for Emobided AI Platform"""

from tkinter import Tk, Button, Canvas, NW, Label
from cv2 import (
    VideoCapture,
    imwrite,
    cvtColor,
    COLOR_BGR2RGB,
    CAP_PROP_FRAME_WIDTH,
    CAP_PROP_FRAME_HEIGHT,
    resize,
)
from PIL import (
    Image,
    ImageTk,
)
from os import (
    path,
    getcwd,
)


class MainGUI:
    def __init__(self, video_source=0) -> None:

        # Configure video source
        self.video_source = video_source
        self.vid = VideoCapture(self.video_source)

        # Configure Root
        self.root = Tk()
        self.configure_root()

        # Configure location and name of saved snaps
        self.save_file_name = "snapshot.jpg"
        self.directory = getcwd() + "/UserInterface/Snapshots"

        # Configure widgets
        self.build_video_stream()  # Builds the video stream widget
        self.build_snapshot_button()  # Builds a button that takes a snapshot of the current video steam of the
        self.build_map()  # Builds the map
        self.build_control_interface()  # Builds the control interface

        # Launch the GUI
        self.root.mainloop()

    def configure_root(self):
        self.root.geometry("500x500")
        self.root.title("Self Drive")

        # Configure grid layout
        self.root.columnconfigure(0, weight=1)
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=1)

    def snap(self):
        # Get a frame from the video source
        ret, frame = self.vid.read()

        if ret:
            # Save the frame to disk
            imwrite(path.join(self.directory, self.save_file_name), frame)
            print("Snap Taken")

    def build_snapshot_button(self):
        self.snap_button = Button(
            self.root,
            text="Take Snap",
            font=("Arial", 18),
            command=self.snap,
        )
        self.snap_button.grid(row=1, column=0, padx=10, pady=10, sticky="s")

    def update(self):
        # Get a frame from the video source
        ret, frame = self.vid.read()

        if ret:
            # Convert the frame from BGR to RGB
            frame = cvtColor(frame, COLOR_BGR2RGB)

            # Resize the frame to match the size of the canvas
            canvas_width = self.canvas.winfo_width()
            canvas_height = self.canvas.winfo_height()
            frame = resize(frame, (canvas_width, canvas_height))

            # Display the frame on the Tkinter canvas
            self.photo = ImageTk.PhotoImage(
                master=self.canvas, image=Image.fromarray(frame)
            )
            self.canvas.create_image(0, 0, image=self.photo, anchor=NW)

        # Schedule the next update
        self.root.after(15, self.update)

    def build_video_stream(self):
        # create a canvas object that can display the video stream
        self.canvas = Canvas(
            self.root,
            width=500,
            height=500,
            # width=self.vid.get(CAP_PROP_FRAME_WIDTH),
            # height=self.vid.get(CAP_PROP_FRAME_HEIGHT),
        )
        self.canvas.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")
        self.update()

    def build_map(self):
        self.map = ImageTk.PhotoImage(Image.open("UserInterface/map_placeholder.png"))
        self.map_label = Label(self.root, image=self.map)
        self.map_label.grid(row=0, column=1)

    def build_control_interface(self):
        self.controls = ImageTk.PhotoImage(Image.open("UserInterface/controller.jpeg"))
        self.controls_label = Label(self.root, image=self.controls)
        self.controls_label.grid(row=1, column=1, padx=10, pady=10, sticky="nsew")


def main():
    """Driver program to test MainGUI class."""
    MainGUI()


if __name__ == "__main__":
    main()
