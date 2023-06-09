"""Launch program for main user interface for embodied AI platform."""

from mainGui import MainGUI
from ImageSubscriber import ImageSubscriber
import rospy

def main():
    """Driver function."""
    # Launch graphical user interface
    MainGUI()


if __name__ == "__main__":
    main()