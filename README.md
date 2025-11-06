# Blender Lidar Simulation Setup

This guide covers the setup and configuration required to run the lidar simulation project using Blender and VS Code.

## 1. Software Setup

### Blender
1.  Install Blender (this project uses version 4.2.13).
2.  To check the Python version Blender uses:
    * Open Blender.
    * Go to the **Scripting** tab.
    * The Python version will be visible in the interactive console on the left.

### Visual Studio Code (VS Code)
1.  Install VS Code.
2.  Open a terminal and run the following commands to get necessary software properties (if not already installed):
    ```bash
    sudo apt-get install software-properties-common
    sudo apt-get update
    ```

## 2. VS Code Configuration

Inside VS Code, install the following extensions:

1.  **Python** (by Microsoft)
2.  **Blender Development** (by Jacques Lucke)

To run Blender from VS Code:
* Open the VS Code command palette (Ctrl+Shift+P).
* Type and select `>Blender: Start`.

## 3. Addon Installation: Blender Range Scanner

1.  Download the `blainder-range-scanner` addon from its GitHub repository:
    * **Repo Link:** [https://github.com/ln-12/blainder-range-scanner](https://github.com/ln-12/blainder-range-scanner)
2.  Follow the installation steps in the repository's `README.md` to install the `.zip` folder as a Blender addon.



## 4. Convert `.xacro` to `.urdf` (Only if you use different models)

To use the `.urdf.xacro` files, you must first convert them to standard `.urdf` files using this ROS command:

```bash
rosrun xacro xacro -o [OUTPUT_PATH/my_robot.urdf] [INPUT_PATH/my_robot.xacro.urdf] robot_namespace:="" 
```



## 5. Recreating simulaiton

* Clone this repoository. 'cd Lidar-Placement-Blender'
* To run the existing blend file.(Right now only two blend files are there.)
    ```
    blender -b --python main.py -- --mode load  --scene twin7500_a320_ceo.blend --tug twin7500 --ac a320_ceo 
    ```
    ```
    blender -b(to run in background) --file name -- --mode(Create or load blend file ) --scene (name of the file) -- tug (tug_name) --ac (ac name)
    ```
* If error occurs 'blender not found'. Execute to make your blender global 
    ```
    echo 'export PATH=$PATH: path_to_blender' >> ~/.bashrc
    source ~/.bashrc
    ```
* Create a new blend file with different TUG and AC models.
Make sure to have stl files and urdf of Tugs and AC in the respective folders.
Execute
    ```
    blender --python main.py -- --mode create  --scene tug_AC.blend --tug tug_name --ac AC_name
    ```
You will see the blender scene ,manually add cube meshes at the desired location on the tug and save the blend file.(Making this also headless : in progress)
Then rerun the command with '-- mode load'


* The scene will automatically:
    * Load Tug and Aircraft 
    * Position and align both models
    * Prepare LiDAR sampling and scanning areas
    * Save or update scene4.blend automatically
