# cassierl

This repository contains reinforcement learning algorithms implemented for Cassie, a bipedal walking robot of Oregon State University.

## Installing Libraries

In order to compile the files in `src`, you need to clone [this repository](https://github.com/CassieRL/ThirdParty) which contains third-party libraries and follow the instruction to install.

## How to Set Paths to Your Own Files

- In `src/Cassie2d/Cassie2d.cpp`

Change the value of `const char* MUJOCO_LICENSE_PATH` to where your `mjkey.txt` is.

Change the value of `const char* XML_FILE_PATH` to where your `cassie2d_stiff.xml` is.

These two constants are located near the beginning of the file.

- In `src/Cassie2d/RobotInterface.h`:

Change the value of `static const std::string xml_model_filename` to where your `cassie2d_stiff.xml` is. Its value should match the value of the XML file path in `Cassie2d.cpp` you changed earlier.

## How to Fix Import Errors when `python3 cassie2d.py`

If you got into import error(s) when trying to run `cassie2d.py`, follow these steps, **depending on what modules you don't have**, to fix the problem:

- **No modules named `rllab`**

`cd` somewhere else outside of this repository

Execute the following commands

    git clone https://github.com/rll/rllab.git
    cd rllab
    sudo python3 setup.py install

Then, you can import `rllab` and run.

- **No modules named `theano`**: `sudo pip3 install theano`
- **No modules named `path`**: `sudo pip3 install path.py`
- **No modules named `cached_property`**: `sudo pip3 install cached_property`

After these fixes, there shouldn't be any other problems. If you still encounter import errors, try to look for what modules your python can't import, then do `sudo pip3 install MODULE_NAME`. This usually works, but not always. **Make sure to use `pip3` because we are doing `python3`**.

## How to `make` and execute

From the root directory `cassierl`, execute the following commands:

    cd src
    make clean
    make

This will take a while to compile. After it's done, from the current `src` directory,

    cd ../rllab/envs
    python3 squatting.py

This should open a new MuJoCo window where Cassie squats up and down. You're done!

## Some useful documents to understand the reinforcement learning setup (not needed for setup)

The Docs folder contains a writeup, and presentation pdf file. For the presentation with videos, [this link](https://docs.google.com/presentation/d/1wqTQ8Lswg40DpZgtPWiQ-hja9CdKYFQIVGe_ChCKp2A/edit#slide=id.p3)

Please note: there might be some mistakes or changes that have been made since the report/presentation was made.
