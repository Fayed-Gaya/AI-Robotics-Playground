import tkinter as tk
from PIL import Image, ImageTk

class MapWidget(tk.Canvas):
    def __init__(self, master, width, height, rows=10, cols=20, **kwargs):
        super().__init__(master, width=width, height=height, **kwargs)
        self.rows = rows
        self.cols = cols
        self.width = width
        self.height = height
        self.squares = []
        self.images = [[None for _ in range(self.cols)] for _ in range(self.rows)]
        
        # Create grid of squares
        for row in range(self.rows):
            row_squares = []
            for col in range(self.cols):
                x1 = col * (self.width / self.cols)
                y1 = row * (self.height / self.rows)
                x2 = x1 + (self.width / self.cols)
                y2 = y1 + (self.height / self.rows)
                square = self.create_rectangle(x1, y1, x2, y2, fill="white", outline="black")
                row_squares.append(square)
            self.squares.append(row_squares)

    def add_image(self, row, col, filename):
        # Load image and resize to fit cell size
        image = Image.open(filename)
        image = image.resize((int(self.width / self.cols), int(self.height / self.rows)))
        
        # Add image to canvas and save reference
        image_tk = ImageTk.PhotoImage(image)
        self.create_image(col * (self.width / self.cols), row * (self.height / self.rows),
                          image=image_tk, anchor="nw")
        self.images[row][col] = image_tk
        
    def snap(self, row, col, filename):
        # Save current frame to file
        # Here, you can add the code to capture the current frame from your video stream
        # and save it to the specified file
        
        # Add image to map
        self.add_image(row, col, filename)
        # Add link to image in corresponding square
        self.itemconfigure(self.squares[row][col], cursor="hand2")
        self.tag_bind(self.squares[row][col], "<Button-1>", lambda event, filename=filename:
                      self.master.open_image(filename))

class App:
    def __init__(self, master):
        self.master = master
        self.map_widget = MapWidget(master, width=600, height=300)
        self.map_widget.pack()
        self.button = tk.Button(master, text="Snap", command=self.snap)
        self.button.pack()

    def snap(self):
        # Here, you can call your video stream to capture the current frame and save it to a file
        # For demonstration purposes, we'll just create a blank image
        filename = "snap.png"
        Image.new("RGB", (640, 480), (255, 255, 255)).save(filename)
        
        # Get cell coordinates from user input
        row = int(input("Enter row: "))
        col = int(input("Enter column: "))
        
        # Add image to map and link to corresponding square
        self.map_widget.snap(row, col, filename)

    def open_image(self, filename):
        # Here, you can add the code to open the specified image file in a new window
        # For demonstration purposes, we'll just print the filename to the console
        print("Opening image:", filename)

root = tk.Tk()

