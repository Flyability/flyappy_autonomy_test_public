# Flyappy Autonomy Test Game

This repository contains the coding test for the Flyability Autonomy team.

## Game Description

Flyappy is in trouble again! This time it went into space and landed in an asteroid
belt. If Flyappy collides with the asteroids it would be fatal. Luckily Flyappy remembered
his laser scanner that provides distance measurements. It will give you its velocity and
the laserscans in return for an acceleration input. Flyappy only asks for 60 seconds of
your guidance. Help Flyappy go through as many asteroid lines as possible before the time
runs out!

![Flyappy](flyappy_cover.png)

## Getting Started

Clone the repository:
```
git clone https://github.com/Flyability/flyappy_autonomy_test_public.git
cd flyappy_autonomy_test_public
```

### Setup the running environment

*This game has been tested with Ubuntu 24.04 running ROS2 Jazzy and Python 3.12.*

There are two recommended options for running the game.
Either use Ubuntu 24.04 and add the necessary packages to your system to compile and run the game.
Or on any Linux distribution, use [Distrobox](https://github.com/89luca89/distrobox)
to run an Ubuntu 24.04 distribution inside your terminal.

#### Option 1 - Using your system

If you already have Ubuntu 24.04 on your system, great. If not, you can either install
it on your machine (dual-boot or full installation) or boot from an USB flash drive.
You can follow
[Ubuntu tutorial: Install Ubuntu desktop](https://ubuntu.com/tutorials/install-ubuntu-desktop).

If your system is running Windows 11, you can maybe try
[Ubuntu tutorial: Install Ubuntu on WSL2 on Windows 11 with GUI support](https://ubuntu.com/tutorials/install-ubuntu-on-wsl2-on-windows-11-with-gui-support).

Then, make sure ROS2 Jazzy is installed by following the
[ROS install guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).


#### Option 2 - Using Distrobox

First, install [Docker](https://docs.docker.com/engine/install/).

Build an image with Ubuntu 24.04 and ROS 2 Jazzy from the ``./Dockerfile``:
```
docker build . -f Dockerfile -t flyappy
```

Then, install a recent version of Distrobox:
```
git -C /tmp clone https://github.com/89luca89/distrobox.git
cd /tmp/distrobox
git checkout 1.8.0
sudo ./install
```

Create a Distrobox image from the previously built docker image.
The command takes around 2-3 minutes to complete and will ask to type
a new password (twice):
```
distrobox-create --name flyappy --image flyappy --home ~/.flyappy/ && distrobox enter flyappy -- bash -cl true
```

Now, you can enter the Distrobox whenever you need with:
```
distrobox enter flyappy --clean-path
```

Inside the image, you can install the python packages, run the game, build the C++ code,
and run your code.
You can still edit the code from your main distribution.


### Setting up python environment

Make sure python venv is installed:
```
sudo apt install python3.12-venv
```

Create a python virtual environment (with ROS2 packages available):
```
source /opt/ros/jazzy/setup.bash
python3 -m venv .venv --system-site-packages
```

Activate the python virtual environment:
```
source .venv/bin/activate
```

Install the main game:
```
pip install ./flyappy_main_game
```

### Run The Game

To run the game, launch (inside the python virtual environment):
```
flyappy_main_game
```

A GUI will become visible with the game start screen.

You can then add velocity in a wanted direction by pressing the arrow keys
&larr;&uarr;&darr;&rarr;.

Notice that it is not possible to go backwards and if Flyappy hits an obstacle the game
stops.

## Automate Flyappy

Now that we have gotten familiar with the game, we want to control Flyappy autonomously.
To do this, a Python and a C++ template have been provided.

For now the autonomy code does not do anything other than printing out some laser ray and end
game information. To start the game, press any arrow key.

### Python

Install the package (inside the python virtual environment):
```
pip install ./flyappy_autonomy_code_py
```

To run the automation code:
```
flyappy_autonomy_code_node
```

> [!TIP]
> You can install in editable mode with the pip option ``-e``
> to avoid installing every time you do a change in the Python code.

### C++

Compile the code:
```
cd flyappy_autonomy_code_cpp
source /opt/ros/jazzy/setup.bash
cmake --preset release
cmake --build --preset release
```

To run the automation code:
```
./build/release/flyappy_autonomy_code_node
```

> [!TIP]
> Unit tests can be run with:
> ```
> ctest --preset release
> ```
>
> You can also compile in Debug mode with ``--preset debug``

### Modifying the code

The templates are located in the
**flyappy_autonomy_code_cpp/** and **flyappy_autonomy_code_py/** folders.
Be aware that you are not meant to change the files in **flyappy_main_game/**.

For using python, modify (or add) any files in the **src/**, **tests/** folders.

For using C++, modify (or add) any files in the **src/**, **tests/** folders
and, if needed, the **CMakeLists.txt**.

Take your pick.

To get the state of Flyappy, its velocity and laserscans data are published on 2 ROS
topics. An acceleration command can be given on a ROS topic for actuating Flyappy.

The callbacks and publisher are provided in the code.

### Handing In

To hand in your game solution, please send your **flyappy_autonomy_test_public**
repository in a ZIP file by email.

### Other Information

I hope you will have fun solving this little game. If you have any questions or need
other game information either write us or look around in the **flyappy_main_game**
folder. Here is some other helpful information for solving the task.

* Scaling: 1 pixel = 0.01 meter
* Game and sensor update rates: 30 fps
* The velocity measurement is noise free
* Max acceleration x: 3.0 m/s^2
* Max acceleration y: 35.0 m/s^2
* Axis convention: x &rarr;, y &uarr;
* [LaserScan message definition](https://docs.ros.org/en/jazzy/p/sensor_msgs/interfaces/msg/LaserScan.html)

| Value        | Unit               | Topic               |
| ------------- |:-----------------:| :------------------:|
| Velocity      | m/s               | /flyappy_vel        |
| Acceleration  | m/s^2             | /flyappy_acc        |
| LaserScan     | Radians, meters   | /flyappy_laser_scan |
| GameEnded     | No unit           | /flyappy_game_ended |
