"""Implementation of MainGUI Class for Emobided AI Platform"""

from tkinter import (
    Tk,
    Button,
    Canvas,
    NW,
    Label,
    Text,
)
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

        # Configure controls key binding
        self.root.bind("<KeyPress>", self.on_key_event)
        self.root.bind("<KeyRelease>", self.on_key_event)

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
        self.map = ImageTk.PhotoImage(
            Image.open("UserInterface/map_placeholder.png").resize((200, 200))
        )
        self.map_label = Label(
            self.root,
            text="Map Placeholer",
            fg="black",
            compound="top",
            image=self.map,
            padx=10,
            pady=10,
        )
        self.map_label.grid(row=0, column=1)

    def switch_image(self, cell, new_image):
        self.control_canvas.itemconfigure(cell, image=new_image)

    def on_key_event(self, event):
        key = event.char  # Stores the character pressed
        key_press_to_image = {  # Maps key presses to  pressed images
            "w": self.arrow_key_images_pressed[0],
            "a": self.arrow_key_images_pressed[3],
            "s": self.arrow_key_images_pressed[2],
            "d": self.arrow_key_images_pressed[1],
        }

        key_release_to_image = {  # Maps pressed images to their unpressed images
            "w": self.arrow_key_images_unpressed[0],
            "a": self.arrow_key_images_unpressed[3],
            "s": self.arrow_key_images_unpressed[2],
            "d": self.arrow_key_images_unpressed[1],
        }

        if key in key_press_to_image:  # Relevant key detected
            new_image = key_press_to_image[key]
            if event.type == "2":
                if key == "w":
                    self.switch_image(self.image1, new_image)
                elif key == "a":
                    self.switch_image(self.image2, new_image)
                elif key == "s":
                    self.switch_image(self.image3, new_image)
                elif key == "d":
                    self.switch_image(self.image4, new_image)
                print(f"You pressed {event.char}")
            elif event.type == "3":  # Key release
                # original_image = cell_to_image[event.widget.find_withtag(CURRENT)[0]]
                old_image = key_release_to_image[key]
                if key == "w":
                    self.switch_image(self.image1, old_image)
                elif key == "a":
                    self.switch_image(self.image2, old_image)
                elif key == "s":
                    self.switch_image(self.image3, old_image)
                elif key == "d":
                    self.switch_image(self.image4, old_image)
                print(f"You released {event.char}")

    def build_control_interface(self):
        # Create control canvas
        self.control_canvas = Canvas(
            self.root,
            width=300,
            height=200,
        )
        self.control_canvas.grid(row=1, column=1, padx=10, pady=10, sticky="nsew")

        # Configure arrow key images and add to control canvas
        self.arrow_key_images_unpressed = [  # Unpressed images
            ImageTk.PhotoImage(
                Image.open("UserInterface/arrowKeyImages/up_unpressed.jpeg").resize(
                    (50, 50)
                )
            ),
            ImageTk.PhotoImage(
                Image.open("UserInterface/arrowKeyImages/right_unpressed.jpeg").resize(
                    (50, 50)
                )
            ),
            ImageTk.PhotoImage(
                Image.open("UserInterface/arrowKeyImages/down_unpressed.jpeg").resize(
                    (50, 50)
                )
            ),
            ImageTk.PhotoImage(
                Image.open("UserInterface/arrowKeyImages/left_unpressed.jpeg").resize(
                    (50, 50)
                )
            ),
        ]

        self.arrow_key_images_pressed = [  # Pressed images
            ImageTk.PhotoImage(
                Image.open("UserInterface/arrowKeyImages/up_pressed.png").resize(
                    (50, 50)
                )
            ),
            ImageTk.PhotoImage(
                Image.open("UserInterface/arrowKeyImages/right_pressed.png").resize(
                    (50, 50)
                )
            ),
            ImageTk.PhotoImage(
                Image.open("UserInterface/arrowKeyImages/down_pressed.png").resize(
                    (50, 50)
                )
            ),
            ImageTk.PhotoImage(
                Image.open("UserInterface/arrowKeyImages/left_pressed.jpeg").resize(
                    (50, 50)
                )
            ),
        ]
        self.image1 = self.control_canvas.create_image(
            100, 0, image=self.arrow_key_images_unpressed[0], anchor=NW
        )  # Up key
        self.image2 = self.control_canvas.create_image(
            0, 100, image=self.arrow_key_images_unpressed[3], anchor=NW
        )  # Left key
        self.image3 = self.control_canvas.create_image(
            100, 100, image=self.arrow_key_images_unpressed[2], anchor=NW
        )  # Down key
        self.image4 = self.control_canvas.create_image(
            200, 100, image=self.arrow_key_images_unpressed[1], anchor=NW
        )  # Right Key

        # Placeholder image representing controls
        # self.controls = ImageTk.PhotoImage(Image.open("UserInterface/controller.jpeg"))
        # self.controls_label = Label(self.root, image=self.controls)
        # self.controls_label.grid(row=1, column=1, padx=10, pady=10, sticky="nsew")


def main():
    """Driver program to test MainGUI class."""
    MainGUI()


if __name__ == "__main__":
    main()
