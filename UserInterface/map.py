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

        self.draw_grid()

    # Create grid of squares
    def draw_grid(self):
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
    
    # Add image to cell
    def add_image(self, row, col, filename):
        # Load image and resize to fit cell size
        image = Image.open(filename)
        image = image.resize((int(self.width / self.cols), int(self.height / self.rows)))

        # Add image to map and save reference
        image_tk = ImageTk.PhotoImage(image)
        self.create_image(col * (self.width / self.cols), row * (self.height / self.rows), image=image_tk, anchor=tk.NW)
        self.images[row][col] = image_tk
    
    def snap(self, row, col, filename):
        # Add image to map
        self.add_image(row, col, filename)

        # Add link to image in corresponding square
        self.itemconfigure(self.squares[row][col])
        self.config(cursor="hand2")
        self.tag_bind(self.squares[row][col], "<Button-1>", lambda event, filename=filename:self.master.open_image(filename))

def main():
    root = tk.Tk()
    map = MapWidget(root)
    map.pack()
    root.mainloop()

if __name__ == "__main__":
    main()
