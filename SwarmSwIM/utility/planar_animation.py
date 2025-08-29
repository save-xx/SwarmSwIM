import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore
import numpy as np
import sys
import imageio


class Simulator2D:
    def __init__(
            self,
            tick_function: callable, 
            state, # temp
            fps: float = 60, 
            properties: dict | None = None
            ) -> None:
        """

        """
        self.myfunc = tick_function
        self.state = state
        self.fps = fps
        self.dt = 1.0 / fps

        # Default properties
        defaults = {
            "title": "2D Simulator",
            "record": False,
            "outfile": "simulation.mp4",
            "bg_color": "k",  # black background
            "bg_shapes": []  # list of shapes to draw
        }
        # Load proprieties
        self.props = {**defaults, **(properties or {})}

        # Recording buffer
        self.frames = []

        # Qt app
        self.app = QtWidgets.QApplication(sys.argv)

        # Window + plot
        self.win = pg.GraphicsLayoutWidget(show=True, title=self.props["title"])
        self.plot = self.win.addPlot()
        self.plot.setAspectLocked(True)
        self.plot.showGrid(x=True, y=True)
        self.win.setBackground(self.props["bg_color"])

        # Background image
        if "bg_image" in self.props and self.props["bg_image"]:
            path = self.props["bg_image"]["path"]
            scale = self.props["bg_image"].get("scale", 1.0)
            img = imageio.v2.imread(path)  
            # Ensure RGB (drop alpha if present)
            if img.shape[-1] == 4:
                img = img[..., :3]          
            # Fix orientation:
            # - Transpose to swap axes (correct 90° rotation)
            # - Flip vertically to match Cartesian Y-up
            img = np.fliplr(np.transpose(img, (1, 0, 2)))
            img_item = pg.ImageItem(img)
            w, h = img.shape[:2]
            img_item.setScale(scale)
            img_item.setPos(-w * scale / 2, -h * scale / 2)
            img_item.setZValue(-10)  # put behind everything
            self.plot.addItem(img_item)

        # Example: polygon
        poly = self.state.get("poly", np.array([[0, 0]]))
        self.item = pg.PlotDataItem(
            poly[:, 0], poly[:, 1], pen="c", symbol=None, connect="all"
        )
        self.plot.addItem(self.item)

        # Timer
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(int(1000 / fps))

    def update(self):
        """Advance simulation and update graphics."""
        self.state = self.myfunc(self.dt, self.state)
        if "poly" in self.state:
            poly = self.state["poly"]
            self.item.setData(poly[:, 0], poly[:, 1])

        if self.props["record"]:
            qpix = self.win.grab()
            qimg = qpix.toImage()
            ptr = qimg.bits()
            ptr.setsize(qimg.byteCount())
            arr = np.array(ptr, dtype=np.uint8).reshape(qimg.height(), qimg.width(), 4)
            self.frames.append(arr[..., :3])  # drop alpha

    def run(self):
        """Run simulation."""
        exit_code = self.app.exec()
        if self.props["record"] and self.frames:
            print(f"Saving video to {self.outfile} ...")
            imageio.mimsave(self.outfile, self.frames, fps=self.fps)
        sys.exit(exit_code)


### TEST


def myfunc(dt, state):
    poly = state["poly"]
    vel = state["vel"]

    # Move polygon
    poly[:, 0] += vel[0] * dt
    poly[:, 1] += vel[1] * dt

    # Bounce
    if np.any(poly[:, 0] < -5) or np.any(poly[:, 0] > 5):
        vel[0] *= -1
    if np.any(poly[:, 1] < -5) or np.any(poly[:, 1] > 5):
        vel[1] *= -1

    state["poly"] = poly
    state["vel"] = vel
    return state


if __name__ == "__main__":
    poly = np.array([[0, 0], [1, 0], [0.5, 1]], dtype=float)
    vel = np.array([1.0, 0.5])
    state = {"poly": poly, "vel": vel}

    properties = {
        "bg_image": {
            "path": "drift_analisys3.png",
            "scale": 0.01   # scaling factor in data coords
        }
}
    sim = Simulator2D(myfunc, state, fps=30, properties=properties)
    sim.run()