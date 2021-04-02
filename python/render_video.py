import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation

from PIL import Image

import sys
import os
import yaml

root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

# Dots per inch, represents the dot density in the output video.
DPI = 200


def load_config(path_to_config_file):
    """Load the config in yaml from config file."""

    with open(path_to_config_file) as file:
        config = yaml.load(file, Loader=yaml.FullLoader)

        assert(config["output_config"]["datalog_config"]["output_datalog"]
               and "The output_datalog flag is off! Make sure the simulation is run with output_datalog on in order to be able to render video!")
        assert(config["output_config"]["video_config"]["render_video"]
               and "The render_video flag is off! Make sure the simulation is run with render_video on in order to be able to render video!")

        return config


def convert_to_xs_and_ys(waypoint, lon_min, lon_max, lat_min, lat_max, w, h):
    """Convert from a list of positions to their x/y coordinates in the image."""

    xs = []
    ys = []

    for step in waypoint:
        xs.append((step["lon"] - lon_min) / (lon_max - lon_min) * w)
        ys.append((lat_max - step["lat"]) / (lat_max - lat_min) * h)

    return (xs, ys)


def convert_to_x_and_y(pos, lon_min, lon_max, lat_min, lat_max, w, h):
    """Convert from a pos to its x/y coordinates in the image."""

    x = (pos["lon"] - lon_min) / (lon_max - lon_min) * w
    y = (lat_max - pos["lat"]) / (lat_max - lat_min) * h

    return (x, y)


def get_color(id):
    """Get color of vehicle given its id. We only color the first five vehicles."""

    color = "0.50"
    if id == 0:
        color = "#dc241f"
    elif id == 1:
        color = "#9b0058"
    elif id == 2:
        color = "#0019a8"
    elif id == 3:
        color = "#0098d8"
    elif id == 4:
        color = "#b26300"

    return color


def main():
    # # Check command line arguments.
    # if (len(sys.argv) != 3):
    #     print("[ERROR] We need exact 2 arguments aside from the program name for correct execution! \n"
    #           "- Usage: python3 <prog name> <arg1> <arg2>. \n"
    #           "  <arg1> is the path to the platform config file. \n"
    #           "  <arg2> is the path to the background map image. \n"
    #           "- Example: python3 {} \"./config/platform_demo.yml\" \"./media/hongkong.png\"\n".format(sys.argv[0]))
    #     sys.exit(1)
    #

    # config = load_config(sys.argv[1])

    # Load config.
    config_file_path = root_path + "/config/platform_demo.yml"
    config = load_config(config_file_path)

    print("Loaded the config from {}.".format(config_file_path))

    # Load boundaries.
    lon_min = config["area_config"]["lon_min"]
    lon_max = config["area_config"]["lon_max"]
    lat_min = config["area_config"]["lat_min"]
    lat_max = config["area_config"]["lat_max"]

    print("Loaded boundary: lon_min={}, lon_max={}, lat_min={}, lat_max={}.".format(
        lon_min, lon_max, lat_min, lat_max))

    # Load background image.
    map_image_file_path = root_path + "/media/manhattan.png"
    img = Image.open(map_image_file_path)
    w, h = img.size

    print("Loaded background map image from {}: width={}, height={}".format(map_image_file_path, w, h))

    # Load other data from config.
    num_frames = int(config["simulation_config"]["simulation_duration_s"] / config["simulation_config"]
                     ["cycle_s"]) * config["output_config"]["video_config"]["frames_per_cycle"]
    frame_interval_ms = config["simulation_config"]["cycle_s"] * 1000\
                        / config["output_config"]["video_config"]["frames_per_cycle"]
    num_vehs = config["mod_system_config"]["fleet_config"]["fleet_size"]

    path_to_datalog = config["output_config"]["datalog_config"]["path_to_output_datalog"]
    fps = 1000 / frame_interval_ms * \
          config["output_config"]["video_config"]["replay_speed"]
    viedo_duration = num_frames / fps
    print("Rendering video of total {} frames (fps:{}, duration:{}s) using datalog from {}".format(
        num_frames,fps, viedo_duration, path_to_datalog))


    with open(path_to_datalog) as file:
        # Create the plot.
        fig = plt.figure(figsize=(w/DPI, h/DPI), dpi=DPI)
        ax = plt.axes(xlim=(0, w), ylim=(h, 0))
        ax.imshow(img)
        ax.spines['top'].set_visible(False)
        ax.spines['left'].set_visible(False)
        ax.spines['bottom'].set_visible(False)
        ax.spines['right'].set_visible(False)
        fig.subplots_adjust(left=0.00, bottom=0.00, right=1.00, top=1.00)

        # Handles for vehicles, the immediate next waypoint, the following waypoint, and further waypoints.
        vehs = []
        wp0 = []
        wp1 = []
        wp2 = []

        # Also plot the dispatched orders and the walked away orders.
        dispatched_orders = ax.plot(
            [], [], 'P', color='darkgreen', marker='P', markersize=12, alpha=0.8)[0]
        walked_away_orders = ax.plot(
            [], [], 'X', color='darkred', marker='X', markersize=8, alpha=0.8)[0]

        # A text box
        text = ax.text(0.05 * w, 0.05 * h, "text", horizontalalignment='left', verticalalignment='bottom', fontsize=13,
                       bbox=dict(facecolor='white', edgecolor='grey', boxstyle='round', alpha=0.7))

        for id in range(num_vehs):
            # Get the color of the current vehicle. We only color the first 5 vehicles for visibility.
            color = get_color(id)
            vehs.append(ax.plot([], [], color=color,
                                marker='o', markersize=8, alpha=1)[0])
            wp0.append(ax.plot([], [], linestyle='-',
                               linewidth=2, color=color, alpha=0.7)[0])
            wp1.append(ax.plot([], [], linestyle='--',
                               linewidth=2, color=color, alpha=0.7)[0])
            wp2.append(ax.plot([], [], linestyle=':',
                               linewidth=2, color=color, alpha=0.7)[0])

        def init():
            return vehs, wp0, wp1, wp2, dispatched_orders, walked_away_orders, text

        def animate(n):
            print("Rendering Frame {} / {} of video (fps:{}, duration:{}s)...".format(
                n + 1, num_frames, fps, viedo_duration))

            # Load the yaml of the current frame.
            string = ""
            while True:
                line = file.readline()
                if not line:
                    print("Reached the end of datalog file before expected! \n")
                    sys.exit(1)
                if line == "---\n":
                    break
                string += line
            frame = yaml.safe_load(string)
            system_time_ms = frame["system_time_ms"]

            # Render vehicles.
            vehicles = frame["vehicles"]
            for id, vehicle in enumerate(vehicles):
                vehs[id].set_data(convert_to_x_and_y(
                    vehicle["pos"], lon_min, lon_max, lat_min, lat_max, w, h))

                wp0[id].set_data([], [])
                wp1[id].set_data([], [])
                wp2[id].set_data([], [])

                if (vehicle["waypoints"] is None):
                    continue

                if (len(vehicle["waypoints"]) > 0):
                    xs, ys = convert_to_xs_and_ys(
                        vehicle["waypoints"][0], lon_min, lon_max, lat_min, lat_max, w, h)
                    wp0[id].set_data(xs, ys)

                if (len(vehicle["waypoints"]) > 1):
                    xs, ys = convert_to_xs_and_ys(
                        vehicle["waypoints"][1], lon_min, lon_max, lat_min, lat_max, w, h)
                    wp1[id].set_data(xs, ys)

                if (len(vehicle["waypoints"]) > 2):
                    xs, ys = convert_to_xs_and_ys(
                        vehicle["waypoints"][2], lon_min, lon_max, lat_min, lat_max, w, h)
                    wp2[id].set_data(xs, ys)

            # Render orders.
            dispatched_orders_xs = []
            dispatched_orders_ys = []
            walked_away_orders_xs = []
            walked_away_orders_ys = []

            total_orders_count = 0
            accepted_orders_count = 0
            completed_orders_count = 0

            if "orders" in frame:
                for order in frame["orders"]:
                    if (order["request_time_ms"] >= config["simulation_config"]["warmup_duration_s"] * 1000):
                        total_orders_count += 1

                        if order["status"] != "WALKAWAY":
                            accepted_orders_count += 1
                        if order["status"] == "DROPPED_OFF":
                            completed_orders_count += 1

                        if order["status"] == "WALKAWAY":
                            if order["request_time_ms"] >= system_time_ms - frame_interval_ms:
                                x, y = convert_to_x_and_y(
                                    order["origin"], lon_min, lon_max, lat_min, lat_max, w, h)
                                walked_away_orders_xs.append(x)
                                walked_away_orders_ys.append(y)
                        elif order["status"] == "DISPATCHED":
                            x, y = convert_to_x_and_y(
                                order["origin"], lon_min, lon_max, lat_min, lat_max, w, h)
                            dispatched_orders_xs.append(x)
                            dispatched_orders_ys.append(y)

            dispatched_orders.set_data(dispatched_orders_xs, dispatched_orders_ys)
            walked_away_orders.set_data(
                walked_away_orders_xs, walked_away_orders_ys)

            # Render text.
            text_str = "T = {}s\n{} requested orders ({} accepted, {} completed)".format(
                system_time_ms / 1000.0,
                total_orders_count,
                accepted_orders_count,
                completed_orders_count)
            text.set_text(text_str)

            return vehs, wp0, wp1, wp2, dispatched_orders, walked_away_orders, text

        anime = animation.FuncAnimation(
            fig, animate, init_func=init, frames=num_frames)

        # Set up formatting for the movie file and write.
        path_to_output_video = config["output_config"]["video_config"]["path_to_output_video"]
        Writer = animation.writers['ffmpeg']
        writer = Writer(fps=fps, metadata=dict(
            artist='mod-abm-2.0'), bitrate=1800)
        anime.save(path_to_output_video, writer=writer)

        print("Video saved at {}. Duration={}, FPS={}, DPI={}.".format(
            path_to_output_video, viedo_duration, fps, DPI))


if __name__ == "__main__":
    main()
