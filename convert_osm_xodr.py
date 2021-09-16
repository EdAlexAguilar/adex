import pathlib
import os
import argparse
import sys
import glob


try:
    sys.path.append(
        glob.glob('C:\CARLA\CARLA_0.9.10\WindowsNoEditor\PythonAPI\carla\dist\carla-0.9.10-py3.7-win-amd64.egg')[0])
except IndexError:
    pass



import carla


# parser
parser = argparse.ArgumentParser()
parser.add_argument("--osm", type=pathlib.Path, help="path to the OSM file", required=True)
args = parser.parse_args()

# Read the .osm data
infile = args.osm.absolute()
with open(str(infile), 'r', encoding="utf8") as f:
    osm_data = f.read()

# Note: it works only if running from the `examples` directory
pwd = os.curdir
carla_client_api_dir = pathlib.Path(f"C:\CARLA\CARLA_0.9.10\WindowsNoEditor\PythonAPI\examples").absolute()
assert carla_client_api_dir.exists()
os.chdir(carla_client_api_dir)


# Define the desired settings.
settings = carla.Osm2OdrSettings()  # In this case, default values
xodr_data = carla.Osm2Odr.convert(osm_data, settings)

# save opendrive file
outfile = infile.with_suffix(".xodr")
with open(outfile, 'w') as f:
    f.write(xodr_data)

os.chdir(pwd)
