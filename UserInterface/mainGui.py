"""Implementation of MainGUI Class for Emobided AI Platform"""

from tkinter import (
    Tk,
    Button,
    Canvas,
    NW,
)
from cv2 import (
    VideoCapture,
    imwrite,
    cvtColor,
    COLOR_BGR2RGB,
    CAP_PROP_FRAME_WIDTH,
    CAP_PROP_FRAME_HEIGHT,
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

        # Launch the GUI
        self.root.mainloop()

    def configure_root(self):
        self.root.geometry("500x500")
        self.root.title("Self Drive")

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
        self.snap_button.pack(padx=10, pady=10)

    def update(self):
        # Get a frame from the video source
        ret, frame = self.vid.read()

        if ret:
            # Convert the frame from BGR to RGB
            frame = cvtColor(frame, COLOR_BGR2RGB)

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
            width=self.vid.get(CAP_PROP_FRAME_WIDTH),
            height=self.vid.get(CAP_PROP_FRAME_HEIGHT),
        )
        self.canvas.pack()
        self.update()


def main():
    """Driver program to test MainGUI class."""
    MainGUI()


if __name__ == "__main__":
    main()
