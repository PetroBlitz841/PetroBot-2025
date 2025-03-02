import os
import subprocess

MAIN_HUB = "Controller3"
SECOND_HUB = "Controller4"
HUB_NAME = SECOND_HUB


target = os.getenv("TARGET")
command = f"pybricksdev run ble --name {HUB_NAME} {target}"

try:
    subprocess.run(command, shell=True, check=True)
except subprocess.CalledProcessError:
    print("Erַror uploading code to hub")
    print(
        "Make sure: \n - HUB_NAME is set \n - The controller is on \n - No program is running \n - BlueTooth is on (on your computer)"
    )
