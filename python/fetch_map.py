import numpy as np

from PIL import Image
from six import BytesIO
from six.moves.urllib.request import urlopen, Request

import sys
import yaml


# Max number of tiles allowed in the map image to be fetched.
MAX_TILES = 128
# Size in pixels of the single tile.
TILE_SIZE = 256
# The url to the OpenStreetMap server.
TILE_SERVER = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"


def get_boundary_from_config(path_to_config_file):
    """Load the boundary of the simulated area from config file. """

    with open(path_to_config_file) as file:
        config = yaml.load(file, Loader=yaml.FullLoader)

        lon_min = config["area_config"]["lon_min"]
        lon_max = config["area_config"]["lon_max"]
        lat_min = config["area_config"]["lat_min"]
        lat_max = config["area_config"]["lat_max"]

        return (lon_min, lon_max, lat_min, lat_max)


def deg2num(latitude, longitude, zoom, do_round=True):
    """Convert from latitude and longitude to tile numbers.
    If do_round is True, return integers. Otherwise, return floating point values.
    Source: http://wiki.openstreetmap.org/wiki/Slippy_map_tilenames#Python
    """

    lat_rad = np.radians(latitude)
    n = 2.0 ** zoom
    if do_round:
        f = np.floor
    else:
        def f(x): return x
    xtile = f((longitude + 180.) / 360. * n)
    ytile = f((1.0 - np.log(np.tan(lat_rad) + (1 / np.cos(lat_rad))) / np.pi) /
              2. * n)
    if do_round:
        if isinstance(xtile, np.ndarray):
            xtile = xtile.astype(np.int32)
        else:
            xtile = int(xtile)
        if isinstance(ytile, np.ndarray):
            ytile = ytile.astype(np.int32)
        else:
            ytile = int(ytile)

    return (xtile, ytile)


def get_allowed_zoom(lon_min, lon_max, lat_min, lat_max, z=18):
    """Get the max allowed zoom based on the area boundary."""

    x0, y0 = deg2num(lat_min, lon_min, z)
    x1, y1 = deg2num(lat_max, lon_max, z)

    sx = abs(x1 - x0) + 1
    sy = abs(y1 - y0) + 1

    if sx * sy >= MAX_TILES:
        z = get_allowed_zoom(lon_min, lon_max, lat_min, lat_max, z - 1)

    return z


def fetch_tile(x, y, z):
    """Fetch tile (x, y) at zoom level z from OpenStreetMap's servers."""

    url = TILE_SERVER.format(x=x, y=y, z=z)

    print("Fetching tile from {}".format(url))

    req = Request(url, data=None, headers={'User-Agent': 'smopy'})
    png = BytesIO(urlopen(req).read())
    img = Image.open(png)
    img.load()

    return img


def main():
    # Check command line arguments.
    if (len(sys.argv) != 3):
        print("[ERROR] We need exact 2 arguments aside from the program name for correct execution! \n"
              "- Usage: python3 <prog name> <arg1> <arg2>. \n"
              "  <arg1> is the path to the platform config file. \n"
              "  <arg2> is the path to the output map image. \n"
              "- Example: python3 {} \"./config/platform_demo.yml\" \"./media/mahattan.png\"\n".format(sys.argv[0]))

        sys.exit(1)

    print("Loading the boundary of the simulated area from config at {}.".format(
        sys.argv[1]))

    # Load the boundary of the area.
    lon_min, lon_max, lat_min, lat_max = get_boundary_from_config(sys.argv[1])

    print("Loaded boundary: lon_min={}, lon_max={}, lat_min={}, lat_max={}.".format(
        lon_min, lon_max, lat_min, lat_max))

    # Get max allowed zoom based on the MAX_TILES.
    z = get_allowed_zoom(lon_min, lon_max, lat_min, lat_max)

    print("The max zoom for this area is z={}, using TILE_SIZE={} and MAX_TILES={}".format(
        z, TILE_SIZE, MAX_TILES))

    # Meat part of the code that gets the image from the tile server.

    # Get the rounded and unrounded tile numbers.
    x0, y0 = deg2num(lat_min, lon_min, z)
    x1, y1 = deg2num(lat_max, lon_max, z)
    x0f, y0f = deg2num(lat_min, lon_min, z, do_round=False)
    x1f, y1f = deg2num(lat_max, lon_max, z, do_round=False)

    # Swap the min/max in case they are flipped.
    x0, x1 = min(x0, x1), max(x0, x1)
    y0, y1 = min(y0, y1), max(y0, y1)
    x0f, x1f = min(x0f, x1f), max(x0f, x1f)
    y0f, y1f = min(y0f, y1f), max(y0f, y1f)

    # Number of tiles.
    sx = x1 - x0 + 1
    sy = y1 - y0 + 1

    # Fetch each of the tiles.
    img = Image.new('RGB', (sx*TILE_SIZE, sy*TILE_SIZE))
    for x in range(x0, x1 + 1):
        for y in range(y0, y1 + 1):
            px, py = TILE_SIZE * (x - x0), TILE_SIZE * (y - y0)
            img.paste(fetch_tile(x, y, z), (px, py))

    # Crop the image to originally requested boundary
    print("Resizing the downloaded image to match the requested boundary.")

    left = TILE_SIZE * (x0f - x0)
    right = TILE_SIZE * (sx - 1 + x1f - x1)
    top = TILE_SIZE * (y0f - y0)
    bottom = TILE_SIZE * (sy - 1 + y1f - y1)
    img = img.crop((left, top, right, bottom))

    img.save(sys.argv[2])

    print("Image written to path {}.".format(sys.argv[2]))


if __name__ == "__main__":
    main()
