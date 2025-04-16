import os
import shutil
import subprocess
import platform

if os.path.exists("build"):
    print("Cleaning build directory...")
    shutil.rmtree("build")
os.mkdir("build")
os.chdir("build")

print("Configuring the project with CMake...")
if platform.system() == "Windows":
    subprocess.check_call(["cmake", "-G", "Visual Studio 16 2019", ".."])
else:
    subprocess.check_call(["cmake", ".."])

print("Building the project...")
if platform.system() == "Windows":
    subprocess.check_call(["cmake", "--build", ".", "--config", "Release"])
else:
    subprocess.check_call(["cmake", "--build", "."])

print("Running the executable...")
if platform.system() == "Windows":
    subprocess.check_call(["Release\\coordination_manager.exe"])
else:
    subprocess.check_call(["./coordination_manager"])