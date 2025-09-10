import signal
import sys
import os
import logging
import atexit
from itertools import chain
import sqlite3
import pandas as pd
from datetime import datetime


logger = logging.getLogger(__name__)


def save_bag(simulation, name = "swsw"):
    """
    Activate automatic simulation data recording (bagging) to a SQLite database.

    This function sets up a DataBagger plugin that collects the state of all agents,
    sensor readings, and plugin outputs at every simulation timestep. Data is written 
    to a local SQLite database file, which can optionally be converted to Excel (.xlsx) 
    for easier analysis.

    The bagging plugin will automatically:

    1. Record agent states:
       - Positions (x, y, z)
       - Orientation (psi)
       - Measured depth, heading, and position
       - Control modes (depth, heading, planar)
       - Commanded actions (e.g., cmd_depth, cmd_planar, cmd_forces)
    2. Record plugin outputs if the plugin implements a `_bag()` method that returns
       a list of dictionaries.
    3. Save all data in separate tables per agent and plugin.
    4. Handle clean shutdown on Ctrl+C, ensuring the database is closed properly.
    5. Optionally convert the SQLite database to Excel for analysis using `sqlite_to_excel()`.

    Parameters
    ----------
    simulation : object
        The simulation instance containing agents, plugins, and the timestep logic.
    name : str, optional
        Base filename for the saved database (default is "swsw"). The final database
        will include a timestamp in the filename.

    Notes
    -----
    - The plugin attaches to the simulation as `simulation.save_plugin`.
        It will be executed automatically once atteched.
    - Recording starts immediately after activation.
    - The recorded database can be accessed directly with SQLite tools, or converted
      to Excel for further analysis.
    - Each timestep appends data; no overwriting occurs unless the filename already exists.
    """
    simulation.save_plugin = DataBagger(simulation, name)


class DataBagger:
    def __init__(self, simulation, name):
        self.sim = simulation

        # name
        now = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.filename = f"{name}_{now}"
        logger.info(f"Recording simulation data in {self.filename}.db")
        # create pd structure and append to file
        self.conn = sqlite3.connect(self.filename + ".db")

        # Install Ctrl+C handler
        signal.signal(signal.SIGINT, self._handle_sigint)
        # execute closure at normal exit
        atexit.register(self.close)  

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

            row = {**idx, **time, **pos, **psi,
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
        # close database
        if self.conn:
            self.conn.commit()
            self.conn.close()
            self.conn = None
            logger.info("Database connection closed")
        # try conversion in excel
        try:
            sqlite_to_excel(self.filename)
        except Exception as e:
            logger.warning(f"Could not conver bag to xlsx: {e}")

# ===================================
# Additional utility, convert to csv
# ===================================
def sqlite_to_excel(name):
    """
    Convert a SQLite database containing simulation data to an Excel (.xlsx) file.

    This utility reads all tables from a SQLite database (created by `DataBagger`) 
    and writes each table into a separate sheet in an Excel workbook. Column headers 
    and data types are preserved as much as possible.

    Parameters
    ----------
    name : str
        Base filename of the SQLite database (without extension). The function expects
        a file named `{name}.db` and will create `{name}.xlsx` in the same directory.

    Notes
    -----
    - If the database file does not exist, a warning is logged and no Excel file is created.
    - Each table in the SQLite database becomes a separate sheet in the Excel workbook.
    - This function uses `pandas` and `openpyxl` for reading/writing data.
    - Existing Excel files with the same name will be overwritten without prompt.
    """
    sqlite_file = name + ".db"
    excel_file = name + ".xlsx"
    
    if not os.path.exists(sqlite_file):
        logger.warning(f"⚠️ {sqlite_file} not found. No Excel file created.")
        return    
    
    # Connect to SQLite
    conn = sqlite3.connect(sqlite_file)

    # Create Excel writer
    with pd.ExcelWriter(excel_file, engine="openpyxl") as writer:
        # Get all table names in SQLite
        tables = pd.read_sql("SELECT name FROM sqlite_master WHERE type='table';", conn)
        
        for table in tables["name"]:
            # Read each table into a DataFrame
            df = pd.read_sql_query(f"SELECT * FROM {table}", conn)
            # Write to Excel sheet with table name
            df.to_excel(writer, sheet_name=table, index=False)

    conn.close()
    logger.warning(f"✅ Converted {sqlite_file} → {excel_file}")