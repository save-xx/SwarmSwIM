import signal
import sys
import logging
from itertools import chain
import sqlite3
import pandas as pd
from datetime import datetime


logger = logging.getLogger(__name__)


def save_bag(simulation, name = "swsw"):
    """Activate the data saving"""
    simulation.save_plugin = DataBagger(simulation, name)


class DataBagger:
    def __init__(self, simulation, name):
        self.sim = simulation

        # name
        now = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        filename = f"{name}_{now}.db"
        logger.info(f"Recording simulation data in {filename}")
        # create pd structure and append to file
        self.conn = sqlite3.connect(filename)

        # Install Ctrl+C handler
        signal.signal(signal.SIGINT, self._handle_sigint)

    def __call__(self):
        """Collect and save all valuable inormation at this timestep"""
        # Simulation data information
        idx = {"index": self.sim.step_count}
        time = {"time": self.sim.time}

        # --- Batch all agents for this timestep ---
        agent_rows = {}  # table name → list of dicts
        for name, agent in self.sim.agents.items():
            pos = {"x": agent.pos[0], "y": agent.pos[1], "z": agent.pos[2]}
            psi = {"psi": agent.psi}
            internal_clock = {"internal_clock": agent.internal_clock}
            measured_depth = {"measured_depth": agent.measured_depth}
            measured_heading = {"measured_heading": agent.measured_heading}
            measured_pos = {"measured_x": agent.measured_pos[0], "measured_y": agent.measured_pos[1]}

            control = {
                'depth_control': agent.depth_control,
                'heading_control': agent.heading_control,
                'planar_control': agent.planar_control
            }

            cmds = {
                "cmd_depth": agent.cmd_depth,
                "cmd_heave": agent.cmd_heave,
                "cmd_heading": agent.cmd_heading,
                "cmd_yawrate": agent.cmd_yawrate,
                "cmd_x": agent.cmd_planar[0],
                "cmd_y": agent.cmd_planar[1],
                "cmd_local_vx": agent.cmd_local_vel[0],
                "cmd_local_vy": agent.cmd_local_vel[1],
                "cmd_Fx": agent.cmd_forces[0],
                "cmd_Fy": agent.cmd_forces[1]
            }

            row = {**idx, **time, **pos, **psi, **internal_clock,
                **measured_depth, **measured_heading, **measured_pos,
                **control, **cmds}

            agent_rows.setdefault(name, []).append(row)

        # Write all agent tables in one shot per table
        for name, rows in agent_rows.items():
            df_agents = pd.DataFrame(rows)
            df_agents.to_sql(name, self.conn, if_exists="append", index=False)
        
        # Collect all plugin rows per table
        plugin_rows = {}  # key -> list of dicts

        for key, plugin in chain(self.sim.plugins_calls_prestep.items(),
                                self.sim.plugins_calls_poststep.items()):
            if hasattr(plugin, "_bag") and callable(getattr(plugin, "_bag")):
                rows = plugin._bag()  # returns list of dicts
                if not rows:  # skip empty
                    continue
                plugin_rows.setdefault(key, []).extend(rows)  # append rows

        # Write all plugin tables in one shot per table
        for key, rows in plugin_rows.items():
            df = pd.DataFrame(rows)
            df.to_sql(key, self.conn, if_exists="append", index=False)


    # handle closures
    def _handle_sigint(self, sig, frame):
        logger.warning("Ctrl+C detected: closing database before exit")
        self.close()
        sys.exit(0)

    def close(self):
        if self.conn:
            self.conn.commit()
            self.conn.close()
            self.conn = None
            logger.info("Database connection closed")