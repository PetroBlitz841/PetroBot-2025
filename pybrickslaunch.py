MAIN_HUB = "Controller3"
SECOND_HUB = "Controller4"
HUB_NAME = MAIN_HUB

import os
import subprocess

target = os.getenv("TARGET")
command = f"pybricksdev run ble --name {HUB_NAME} {target}"

try:
    subprocess.run(command, shell=True, check=True)
except subprocess.CalledProcessError:
    print("Erַror uploading code to hub")
    print("Make sure to set HUB_NAME and turn the hub on")
