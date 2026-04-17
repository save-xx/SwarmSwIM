from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt


# "with_ranging"
# "no_range"

LOG_DIR = Path("logs")

NAV_FILE = LOG_DIR / "with_ranging" / "nav_log.csv"
COOP_FILE = LOG_DIR / "with_ranging" / "coop_log.csv"
COOP_DEBUG_FILE = LOG_DIR / "with_ranging" / "coop_update_debug_log.csv"

FIG_DIR = LOG_DIR / "with_ranging" / "figures"
FIG_DIR.mkdir(parents=True, exist_ok=True)


def save_plot(fig, name: str):
    fig.tight_layout()
    fig.savefig(FIG_DIR / name, dpi=200, bbox_inches="tight")
    plt.close(fig)


def plot_nav_errors(nav: pd.DataFrame):
    for agent, df in nav.groupby("agent"):
        df = df.sort_values("t")

        fig = plt.figure(figsize=(8, 4.5))
        plt.plot(df["t"], df["ex"], label="ex")
        plt.plot(df["t"], df["ey"], label="ey")
        plt.plot(df["t"], df["ez"], label="ez")
        plt.xlabel("time [s]")
        plt.ylabel("position error [m]")
        plt.title(f"{agent} position error components")
        plt.legend()
        plt.grid(True)
        save_plot(fig, f"{agent}_error_components.png")


def plot_nav_error_norm(nav: pd.DataFrame):
    fig = plt.figure(figsize=(8, 4.5))

    for agent, df in nav.groupby("agent"):
        df = df.sort_values("t")
        plt.plot(df["t"], df["e_norm"], label=agent)

    plt.xlabel("time [s]")
    plt.ylabel("||position error|| [m]")
    plt.title("Position error norm")
    plt.legend()
    plt.grid(True)
    save_plot(fig, "all_agents_error_norm.png")


def plot_traceP(nav: pd.DataFrame):
    fig = plt.figure(figsize=(8, 4.5))

    for agent, df in nav.groupby("agent"):
        df = df.sort_values("t")
        plt.plot(df["t"], df["traceP_pos"], label=f"{agent} traceP_pos")

    plt.xlabel("time [s]")
    plt.ylabel("trace(P_pos)")
    plt.title("Position covariance trace")
    plt.legend()
    plt.grid(True)
    save_plot(fig, "all_agents_traceP_pos.png")


def plot_body_velocity(nav: pd.DataFrame):
    for agent, df in nav.groupby("agent"):
        df = df.sort_values("t")

        fig = plt.figure(figsize=(8, 4.5))
        plt.plot(df["t"], df["u_hat"], label="u_hat")
        plt.plot(df["t"], df["v_hat"], label="v_hat")
        plt.xlabel("time [s]")
        plt.ylabel("body velocity [m/s]")
        plt.title(f"{agent} estimated body velocity")
        plt.legend()
        plt.grid(True)
        save_plot(fig, f"{agent}_body_velocity.png")


def plot_xy_trajectory(nav: pd.DataFrame):
    fig = plt.figure(figsize=(6, 6))

    for agent, df in nav.groupby("agent"):
        df = df.sort_values("t")
        plt.plot(df["x"], df["y"], label=f"{agent} true")
        plt.plot(df["x_hat"], df["y_hat"], linestyle="--", label=f"{agent} est")

    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("True vs estimated trajectories")
    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    save_plot(fig, "xy_trajectories_true_vs_est.png")


def plot_range_measurements(coop: pd.DataFrame):
    if coop.empty:
        return

    fig = plt.figure(figsize=(8, 4.5))

    for (rx, tx), df in coop.groupby(["receiver", "sender"]):
        df = df.sort_values("t")
        plt.plot(df["t"], df["range"], label=f"{rx}<-{tx}")

    plt.xlabel("time [s]")
    plt.ylabel("range [m]")
    plt.title("Received acoustic ranges")
    plt.legend(ncol=2)
    plt.grid(True)
    save_plot(fig, "ranges_over_time.png")


def plot_packet_age(coop: pd.DataFrame):
    if coop.empty:
        return

    fig = plt.figure(figsize=(8, 4.5))

    for (rx, tx), df in coop.groupby(["receiver", "sender"]):
        df = df.sort_values("t")
        plt.plot(df["t"], df["packet_age"], label=f"{rx}<-{tx}")

    plt.xlabel("time [s]")
    plt.ylabel("packet age [s]")
    plt.title("Packet age over time")
    plt.legend(ncol=2)
    plt.grid(True)
    save_plot(fig, "packet_age_over_time.png")


def plot_sender_covariance(coop: pd.DataFrame):
    if coop.empty:
        return

    fig = plt.figure(figsize=(8, 4.5))

    for sender, df in coop.groupby("sender"):
        df = df.sort_values("t")
        plt.plot(df["t"], df["sender_Pxx"], label=f"{sender} Pxx")
        plt.plot(df["t"], df["sender_Pyy"], linestyle="--", label=f"{sender} Pyy")

    plt.xlabel("time [s]")
    plt.ylabel("covariance")
    plt.title("Broadcast sender covariance")
    plt.legend(ncol=2)
    plt.grid(True)
    save_plot(fig, "sender_covariance_over_time.png")


def plot_coop_innovation(coop_debug: pd.DataFrame):
    if coop_debug.empty or "nu" not in coop_debug.columns:
        return

    fig = plt.figure(figsize=(8, 4.5))
    for (rx, tx), df in coop_debug.groupby(["receiver", "sender"]):
        df = df.sort_values("t")
        plt.plot(df["t"], df["nu"], label=f"{rx}<-{tx}")

    plt.xlabel("time [s]")
    plt.ylabel("innovation nu [m]")
    plt.title("Cooperative range innovation")
    plt.legend(ncol=2)
    plt.grid(True)
    save_plot(fig, "coop_innovation_over_time.png")


def plot_coop_nis(coop_debug: pd.DataFrame):
    if coop_debug.empty or "nis" not in coop_debug.columns:
        return

    fig = plt.figure(figsize=(8, 4.5))
    for (rx, tx), df in coop_debug.groupby(["receiver", "sender"]):
        df = df.sort_values("t")
        plt.plot(df["t"], df["nis"], label=f"{rx}<-{tx}")

    plt.xlabel("time [s]")
    plt.ylabel("NIS")
    plt.title("Normalized innovation squared")
    plt.legend(ncol=2)
    plt.grid(True)
    save_plot(fig, "coop_nis_over_time.png")


def plot_coop_Reff(coop_debug: pd.DataFrame):
    if coop_debug.empty or "R_eff" not in coop_debug.columns:
        return

    fig = plt.figure(figsize=(8, 4.5))
    for (rx, tx), df in coop_debug.groupby(["receiver", "sender"]):
        df = df.sort_values("t")
        plt.plot(df["t"], df["R_eff"], label=f"{rx}<-{tx}")

    plt.xlabel("time [s]")
    plt.ylabel("R_eff")
    plt.title("Effective cooperative measurement variance")
    plt.legend(ncol=2)
    plt.grid(True)
    save_plot(fig, "coop_Reff_over_time.png")


def plot_coop_rhat_vs_range(coop_debug: pd.DataFrame):
    if coop_debug.empty:
        return
    if "range" not in coop_debug.columns or "r_hat" not in coop_debug.columns:
        return

    for (rx, tx), df in coop_debug.groupby(["receiver", "sender"]):
        df = df.sort_values("t")

        fig = plt.figure(figsize=(8, 4.5))
        plt.plot(df["t"], df["range"], label="z")
        plt.plot(df["t"], df["r_hat"], label="r_hat")
        plt.xlabel("time [s]")
        plt.ylabel("range [m]")
        plt.title(f"Measured vs predicted range: {rx}<-{tx}")
        plt.legend()
        plt.grid(True)
        save_plot(fig, f"coop_range_vs_rhat_{rx}_{tx}.png")


def plot_coop_acceptance(coop_debug: pd.DataFrame):
    if coop_debug.empty or "accepted" not in coop_debug.columns:
        return

    fig = plt.figure(figsize=(8, 4.5))
    for (rx, tx), df in coop_debug.groupby(["receiver", "sender"]):
        df = df.sort_values("t")
        plt.plot(df["t"], df["accepted"], label=f"{rx}<-{tx}")

    plt.xlabel("time [s]")
    plt.ylabel("accepted")
    plt.title("Accepted/rejected cooperative updates")
    plt.legend(ncol=2)
    plt.grid(True)
    save_plot(fig, "coop_acceptance_over_time.png")


def main():
    if not NAV_FILE.exists():
        raise FileNotFoundError(f"Missing file: {NAV_FILE}")

    nav = pd.read_csv(NAV_FILE)

    if COOP_FILE.exists():
        coop = pd.read_csv(COOP_FILE)
    else:
        coop = pd.DataFrame()

    if COOP_DEBUG_FILE.exists():
        coop_debug = pd.read_csv(COOP_DEBUG_FILE)
    else:
        coop_debug = pd.DataFrame()

    plot_nav_errors(nav)
    plot_nav_error_norm(nav)
    plot_traceP(nav)
    plot_body_velocity(nav)
    plot_xy_trajectory(nav)

    if not coop.empty:
        plot_range_measurements(coop)
        plot_packet_age(coop)
        plot_sender_covariance(coop)

    if not coop_debug.empty:
        plot_coop_innovation(coop_debug)
        plot_coop_nis(coop_debug)
        plot_coop_Reff(coop_debug)
        plot_coop_rhat_vs_range(coop_debug)
        plot_coop_acceptance(coop_debug)

    print(f"Saved figures in: {FIG_DIR.resolve()}")


if __name__ == "__main__":
    main()