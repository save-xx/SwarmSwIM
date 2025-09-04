import pyqtgraph as pg
from PyQt5.QtWidgets import QGraphicsItem
from pyqtgraph.Qt import QtWidgets, QtCore, QtGui
import numpy as np
import sys
import os
import time
import imageio
import logging


LOG_THROTTLE = 2.0
logger = logging.getLogger(__name__)


# ------------------------------------------
# Initialize sequential high contrast colors
# ------------------------------------------
def color_cycle(hues=10, step=3):
    i = 0
    while True:
        yield pg.intColor(i % hues, hues=hues)
        i += step
colors = color_cycle(hues=30)

# --------------
# Animator class
# --------------
class Visualizer2D:
    def __init__(
            self,
            simulation,
            tick_function: callable, 
            properties: dict | None = None
            ) -> None:
        """

        """
        self._log_throttler = time.perf_counter()

        self.tick_function = tick_function
        self.fps = 1 / simulation.Dt
        self.sim = simulation


        self._last_update = None
        # Default properties
        defaults = {
            "title": "2D Simulator",
            "record": False,
            "window_size": (800, 608),
            "outfile": "simulation.mp4",
            "bg_color": "k",  # black background
            "bg_shapes": []  # list of shapes to draw
        }
        # Load proprieties
        self.props = {**defaults, **(properties or {})}
        self.props.setdefault('grid', True)
        self.props.setdefault('color_by_type', False)

        # Recording buffer
        self.frames = []

        # Qt app
        self.app = QtWidgets.QApplication(sys.argv)

        # Window + plot
        self.win = pg.GraphicsLayoutWidget(show=True, title=self.props["title"])

        self.plot = self.win.addPlot()
        self.plot.setAspectLocked(True)

        # image size 
        w, h = self.props["window_size"]
        self.win.resize(w, h)

        # lock window size if recording
        if self.props["record"]:
            self.win.setFixedSize(w, h)

        if self.props['grid']:
            self.plot.showGrid(x=True, y=True)
        self.win.setBackground(self.props["bg_color"])

        # Legend
        self.legend = self.plot.addLegend(offset=(30, 30))

        # Background image
        if "bg_image" in self.props and self.props["bg_image"]:
            path = self.props["bg_image"]["path"]
            scale = self.props["bg_image"].get("scale", 1.0)
            img = imageio.v2.imread(path)
            # grayscale → convert to RGB  
            if img.ndim == 2:
                img = np.stack([img]*3, axis=-1)
            # drop alpha
            elif img.shape[-1] == 4:
                img = img[..., :3]      
            # Fix orientation:
            # - Transpose to swap axes (90° rotation)
            img = np.fliplr(np.transpose(img, (1, 0, 2)))
            img_item = pg.ImageItem(img)
            w, h = img.shape[:2]
            img_item.setScale(scale)
            img_item.setPos(-w * scale / 2, -h * scale / 2)
            img_item.setZValue(-10)  # put behind everything
            self.plot.addItem(img_item)

        # add shapes
        if self.props['bg_shapes']:
            for shape in self.props["bg_shapes"]:
                if isinstance(shape, QGraphicsItem):
                    shape.setZValue(-1)
                    self.plot.addItem(shape)
                else:
                    logger.warning(f"Can't add shape {shape} \nIt is not an Instance of QGraphicsItem")

        # Initiate all agents
        for _, agent in self.sim.agents.items():
            self._initiate_agent(agent)

        # connect resize event only if not locked
        if not self.props["record"]:
            # print new sizes
            self.win.resizeEvent = self._on_resize

        # Timer
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(int(1000 / self.fps ))


    def _on_resize(self, event):
        """Wrap resize function to print new sizes"""
        size = event.size()
        w, h = size.width(), size.height()
        logger.info(f"Window resized to {w}x{h}px")
        # call the normal Qt resize handler
        super(type(self.win), self.win).resizeEvent(event)


    def type_color(self, agent):
        """Return color based on agent type"""
        # initialize dict if does not exist
        if not hasattr(self, "color_map"):
            self.color_map = {}
        agent_type = os.path.splitext(os.path.basename(agent.agent_type))[0]
        # add new color
        new = False
        if not agent_type in self.color_map:
            new_color = next(colors)
            self.color_map[agent_type] = new_color
            new = True
        #return color based on associated agent_type
        return new, agent_type, self.color_map[agent_type]


    def _initiate_agent(self, agent):
        # generate agent color
        if self.props['color_by_type']:
            new, name, color = self.type_color(agent)
        else:
            color = next(colors)
            name = agent.name
        # initialize triangle
        triangle_item = QtWidgets.QGraphicsPolygonItem()
        triangle_item.setBrush(pg.mkBrush(color))
        triangle_item.setPen(pg.mkPen(None))
        triangle_item.setZValue(9) 
        # initialize plot
        curve = self.plot.plot(pen=pg.mkPen(color, width=2))
        curve.setZValue(8)
        # Add to legend
        if self.props['color_by_type'] and not new:
            pass # avoid duplicates
        else:
            self.legend.addItem(curve, name)
            logger.debug(f"Added legend entry for {name}")
        # add to plot:
        self.plot.addItem(triangle_item)
        # create storage:
        setattr(agent, "animation2Dsettings", {'triangle': triangle_item, 'curve': curve})


    def update_agent(self, agent):
        """Update visual representation of the agent"""
        self._update_triangle(agent.animation2Dsettings['triangle'], agent)
        self._update_curve(agent.animation2Dsettings['curve'], agent)


    def _update_triangle(self, item, agent):
        """Update a QGraphicsPolygonItem to an oriented isosceles triangle.d"""
        cy, cx = agent.pos[0], agent.pos[1]
        h, w = agent.dimentions[0], agent.dimentions[1]
        psi = -np.deg2rad(agent.psi)
        # Local coordinates (centered at origin)
        points = np.array([[0,  h/2], [-w/2, -h/2], [ w/2, -h/2]])
        # Rotation matrix
        R = np.array([
            [np.cos(psi), -np.sin(psi)],
            [np.sin(psi),  np.cos(psi)]
        ])
        # Rotate + translate
        pts_rot = (R @ points.T).T + np.array([cx, cy])
        # Convert to QPolygonF
        qpoints = [QtCore.QPointF(x, y) for x, y in pts_rot]
        poly = QtGui.QPolygonF(qpoints)
        # Update polygon
        item.setPolygon(poly)


    def _update_curve(self, curve, agent):
        """Update plot of each agent of np.array([x,y,z])"""
        xy = np.array(agent.memory)[:, :2]  # shape (N,2)
        curve.setData(x=xy[:,1], y=xy[:,0])


    def check_real_time(self):
        """Logs discrepancies of the animation compared to real time."""
        now = time.perf_counter()
        if self._last_update is not None:
            elapsed = now - self._last_update
            if elapsed > 1.5 * self.sim.Dt and self._log_throttler + LOG_THROTTLE < now:  # allow a small tolerance
                self._log_throttler = now
                logger.warning(
                    f"Animation is lagging! Frame took {elapsed*1000:.1f} ms "
                    f"(target {self.sim.Dt*1000:.1f} ms)"
                )
        self._last_update = now


    @staticmethod
    def qimage_to_rgb_array(qimg):
        qimg = qimg.convertToFormat(QtGui.QImage.Format.Format_RGBA8888)
        width = qimg.width()
        height = qimg.height()
        ptr = qimg.bits()
        ptr.setsize(qimg.sizeInBytes())   # use sizeInBytes() instead of byteCount()
        arr = np.array(ptr, dtype=np.uint8).reshape((height, width, 4))
        # discard alpha, convert from RGBA to RGB
        arr = arr[..., :3]
        return arr

    def update(self):
        """Advance simulation and update graphics."""
        self.check_real_time()
        # advance simulation and all associated user functions:
        self.tick_function()
        # update each agent visual representation
        for _, agent in self.sim.agents.items():
            self.update_agent(agent)
        # if set record to True, save image
        if self.props["record"]:
            qpix = self.win.grab()
            qimg = qpix.toImage()
            arr = self.qimage_to_rgb_array(qimg)
            self.frames.append(arr)


    def run(self):
        """Run simulation."""
        exit_code = self.app.exec()
        if self.props["record"] and self.frames:
            self.outfile = self.props["outfile"]
            logger.info(f"Saving video to {self.outfile} ...")
            imageio.mimsave(self.outfile, self.frames, fps=self.fps)
        sys.exit(exit_code)


### TEST

if __name__ == "__main__":
    from SwarmSwIM.sim_class import Simulator
    S = Simulator(0.033)
    
    S.enable_memory()
    S.agents['A01'].cmd_forces = 0.5
    S.agents['A02'].cmd_heading = -90

    def my_func():
        _ = S.tick()
        time.sleep(0.03)
        if S.step_count == 100:
            S.agents['A01'].cmd_heading = 90

    properties = {
        "bg_image": {
            # "path": "images.jpg",
            "path": "sample.png",
            "scale": 0.01,   # scaling factor in data coords
        },
        "bg_color": 'w',
        "grid": True,
        "color_by_type": False,
        "record": True,
    }

    sim = Visualizer2D(S, my_func, properties=properties)
    sim.run()