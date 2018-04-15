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

## How to install necessary tools to run RL algorithms

1. `git fetch` and `git pull` to keep your `cassierl` up-to-date with the current `master` branch.

2. Download and install Anaconda Distribution - **Python 2.7 version** - [here](https://www.anaconda.com/download). This will take a while to install.

3. `cd` to your `$workplacePath` which was already defined in your `.bashrc`. If you defined it right, **this is the parent directory of this repository**.

4. `git clone https://github.com/rll/rllab.git`. This will clone [RL Lab](https://github.com/rll/rllab)'s repository to your `$workplacePath` directory if you haven't done so already.

5. `cd rllab`

6. `sudo python3 setup.py install`. This will add `rllab` to your Python packages.

7. Stay where you are in the `rllab` repository and `sudo ./scripts/setup_linux.sh`.

8. Then, `sudo cp -rf scripts/  /usr/local/lib/python3.5/dist-packages/rllab-0.1.0-py3.5.egg/`. This will copy the `scripts` folder inside `rllab` to the `rllab` folder inside Python packages.

9. `sudo vim /usr/local/lib/python3.5/dist-packages/rllab-0.1.0-py3.5.egg/rllab/misc/instrument.py`. There is a syntax error in this file.

10. Go to **line 972** and **delete the trailing comma after `config.AWS_EXTRA_CONFIGS`**.

11. Now, `cd` back to your `cassierl` repository. Then, `cd rllab/envs`.

12. Try `source activate rllab3`. This will add a `(rllab3)` as a prefix to your command-line prompt.

13. `python3 trpo_cassie.py`. When you first run this thing, it will recommend you to edit your personal config file in rllab. Then, it will terminate. This is fine. Run `sudo chmod +x /usr/local/lib/python3.5/dist-packages/rllab-0.1.0-py3.5.egg/rllab/config_personal.py`. This will allow rllab to execute the newly created personal config.

14. Then, try to run it again - `python3 trpo_cassie.py`.

There may be some errors before you get the TRPO algorithm to run. Just import every Python modules on the way. In case of the module `downsample`, do the following:

    sudo pip3 install --upgrade https://github.com/Theano/Theano/archive/master.zip

    sudo pip3 install --upgrade https://github.com/Lasagne/Lasagne/archive/master.zip

If, in any chance, the program requires you to install the same things to your **Python 2**, then **repeat from steps 6, but replace `python3` with `python2` and `pip3` with `pip2` in every command you execute**.

If you are able to run `trpo_cassie.py`. It will be like this:

![trpo_cassie.py running](https://i.imgur.com/Rm82QD0.png)

Let it run for a few iterations. Then `Ctrl C` in your terminal to turn it off. Now, to make sure it is **running _and_ logging information** on your computer. Go to the `rllab` repository (not the one in `cassierl` but the one you use to install `rllab` module from step 6). Look for the folder `data`. Dig into that folder and check if there is any log file. If so, then you have set things up correctly for TRPO.

## How many iteration do I need?

Note at first Cassie will fall all over the place. After about 200 iterations or more, she will be able to not falling sideway. Observe carefully, you'll see she will mangage to fall in the up-down direction perpendicular to the ground. This is progress.

However, Yathartha said we need at least 1000 to 1500 iteration for Cassie to grasp how to stand. She will eventually fall, but her standing will last much longer.

## Some useful documents to understand the reinforcement learning setup (not needed for setup)

The Docs folder contains a writeup, and presentation pdf file. For the presentation with videos, [this link](https://docs.google.com/presentation/d/1wqTQ8Lswg40DpZgtPWiQ-hja9CdKYFQIVGe_ChCKp2A/edit#slide=id.p3)

Please note: there might be some mistakes or changes that have been made since the report/presentation was made.
