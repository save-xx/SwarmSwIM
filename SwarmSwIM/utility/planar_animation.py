import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore, QtGui
import numpy as np
import sys


def color_cycle(hues=10, step=3):
    i = 0
    while True:
        yield pg.intColor(i % hues, hues=hues)
        i += step


colors = color_cycle(hues=30)


class Visualizer2D:
    """
    2D visualizer for SwarmSwIM using the planar NED convention.

    Plot convention
    ---------------
    - horizontal axis: x = North
    - vertical axis:   y = East

    Agent convention
    ----------------
    From agent_class.py:
    - position is stored as pos = [x, y, z]
    - heading psi is in degrees, NED convention
      * 0 deg   -> North (+x)
      * 90 deg  -> East  (+y)

    Therefore, in the plot:
    - psi=0 must point to the right
    - psi=90 must point upward
    """

    def __init__(self, simulation, tick_function, properties=None, mac=None):
        self.sim = simulation
        self.tick_function = tick_function
        self.mac = mac

        self.props = properties or {}
        self.fps = self.props.get("render_fps", 30)
        self.scale = self.props.get("scale", 1.0)

        self.edges = []
        self.last_delivered = {}

        self.app = QtWidgets.QApplication(sys.argv)
        self.win = pg.GraphicsLayoutWidget(show=True)
        self.plot = self.win.addPlot()
        self.plot.setAspectLocked(True)

        if self.props.get("grid", True):
            self.plot.showGrid(x=True, y=True)

        self.plot.setLabel("bottom", "North / x [m]")
        self.plot.setLabel("left", "East / y [m]")

        for _, agent in self.sim.agents.items():
            self._init_agent(agent)

        self.physics_timer = QtCore.QTimer()
        self.physics_timer.timeout.connect(self.tick_function)
        self.physics_timer.start(int(self.sim.Dt * 1000))

        self.render_timer = QtCore.QTimer()
        self.render_timer.timeout.connect(self.update_visuals)
        self.render_timer.start(int(1000 / self.fps))

    def _init_agent(self, agent):
        color = next(colors)
        color = "b"   # uncomment if you want all agents blue
        triangle = QtWidgets.QGraphicsPolygonItem()
        triangle.setBrush(pg.mkBrush(color))
        triangle.setPen(pg.mkPen(None))
        triangle.setZValue(10)

        curve = self.plot.plot(
            pen=pg.mkPen(color, width=max(1, int(2 * self.scale)))
        )
        curve.setZValue(5)

        self.plot.addItem(triangle)

        agent.anim = {
            "triangle": triangle,
            "curve": curve,
            "base_color": color,
        }

    def update_visuals(self):
        active_agent = None
        if self.mac and getattr(self.mac, "agents_order", None):
            t_frame = self.sim.time % self.mac.frame_duration
            slot_idx = int(t_frame // self.mac.nav_duration)
            if 0 <= slot_idx < len(self.mac.agents_order):
                active_agent = self.mac.agents_order[slot_idx]

        for name, agent in self.sim.agents.items():
            self._update_triangle(agent, is_active=(name == active_agent))
            self._update_curve(agent)

        self._update_edges()

    def update_visuals_TDMA(self):
        active_agent = None
        if self.mac and getattr(self.mac, "agents_order", None):
            t_frame = self.sim.time % self.mac.frame_duration
            slot_idx = int(t_frame // self.mac.slot_duration)
            if 0 <= slot_idx < len(self.mac.agents_order):
                active_agent = self.mac.agents_order[slot_idx]

        for name, agent in self.sim.agents.items():
            self._update_triangle(agent, is_active=(name == active_agent))
            self._update_curve(agent)

        self._update_edges()

    def _update_triangle(self, agent, is_active=False):
        item = agent.anim["triangle"]

        # Standard plotting:
        # x-axis <- North component = pos[0]
        # y-axis <- East  component = pos[1]
        cx, cy = agent.pos[0], agent.pos[1]

        dims = getattr(agent, "dimentions", [10.0, 8.5])
        h = dims[0] * self.scale
        w = dims[1] * self.scale

        # Base triangle defined pointing along +x (North, psi=0)
        pts = np.array([
            [ h / 2.0,  0.0],
            [-h / 2.0, -w / 2.0],
            [-h / 2.0,  w / 2.0],
        ])

        # NED heading:
        # psi=0   -> +x
        # psi=90  -> +y
        # Standard CCW rotation in the plot frame
        psi = np.deg2rad(agent.psi)

        R = np.array([
            [np.cos(psi), -np.sin(psi)],
            [np.sin(psi),  np.cos(psi)],
        ])

        pts = (R @ pts.T).T + np.array([cx, cy])

        poly = QtGui.QPolygonF([QtCore.QPointF(x, y) for x, y in pts])
        item.setPolygon(poly)

        if is_active:
            item.setBrush(pg.mkBrush("r"))
        else:
            item.setBrush(pg.mkBrush(agent.anim["base_color"]))

    def _update_curve(self, agent):
        curve = agent.anim["curve"]
        mem = np.asarray(agent.memory)

        if len(mem) > 1:
            # Standard plotting:
            # x-axis <- mem[:, 0]
            # y-axis <- mem[:, 1]
            curve.setData(x=mem[:, 0], y=mem[:, 1])

    def _update_edges(self):
        max_age = 0.1
        kept = []

        for line, t0 in self.edges:
            if self.sim.time - t0 < max_age:
                kept.append((line, t0))
            else:
                self.plot.removeItem(line)

        self.edges = kept

        delivered = self.last_delivered or {}
        for receiver, msg in delivered.items():
            if msg is None or not msg.intact:
                continue

            s = self.sim.agents[msg.sender].pos
            r = self.sim.agents[receiver].pos

            line = self.plot.plot(
                [s[0], r[0]],
                [s[1], r[1]],
                pen=pg.mkPen((0, 255, 0, 150), width=max(1, int(2 * self.scale))),
            )
            line.setZValue(7)
            self.edges.append((line, self.sim.time))

    def run(self):
        sys.exit(self.app.exec())