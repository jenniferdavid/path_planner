Quick setup
===========

This is under construction and likely to change fast. But for now the
idea is to set up a new catkin workspace and use the contents of this
repository as the `src/` subdirectory of that workspace.

    mkdir -p /path/to/workspace
    cd /path/to/workspace
    git clone git@github.com:poftwaresatent/cargo-ants-ros.git src
    cd src
    catkin_init_workspace
    git clone https://github.com/poftwaresatent/estar2.git
    git clone https://github.com/poftwaresatent/sfl2.git
    cd ..
    catkin_make

You should end up with usable ROS nodes etc under `devel/`. If it
fails to build due to message headers not being generated, repeatedly
run `catkin_make` until it works.

3rd Party Components
--------------------

The components related to path planning depend on [E* Version
2][estar2] and [Sunflower Version 2][sfl2] in order to avoid
reinventing the wheel.

[sfl2]: https://github.com/poftwaresatent/sfl2
[estar2]: https://github.com/poftwaresatent/estar2

Simulator
---------

The Nepumuk simulator comes bundled with [Sunflower][sfl2]. You can
test it by running one of the older demos that do not depend on ROS,
e.g.

    ./devel/lib/sfl2/nepumuk -c src/sfl2/apps/expo1.yaml

In the simulator, press SPACE for step-by-step mode, 'c' for
continuous simulation, and 'q' to quit.

Running the Cargo-ANTs mockup entails the following (subject to
change, please check the commit log if these instructions do not
work).

*Path Adaptor Test*

    roscore
    ./devel/lib/cargo_ants_path_adaptor/path_adaptor alice bob
    ./devel/lib/sfl2/nepumuk -c src/sfl2/ros/cargo-ants.yaml
    rostopic pub -1 bob/path cargo_ants_msgs/Path '{path_id: 1, goals: [{gx: 40, gy: -5, gth: -1.5, dr: 1, dth: 0.1}, {gx: 40, gy: 30, gth: -0.7, dr: 0.1, dth: 0.1}]}'
    rostopic pub -1 alice/path cargo_ants_msgs/Path '{path_id: 1, goals: [{gx: -5, gy: 30, gth: 2, dr: 1, dth: 0.1}, {gx: 35, gy: 18, gth: -1, dr: 0.1, dth: 0.1}]}'

If the rostopic command fails to find the cargo_ants_msgs/Path type, then you need to source the setup.bash (or other -- depending on your shell) underneath the devel directory in the cargo-ants-ros workspace.

*Path Planner (and Path Adaptor) Test*

    roscore
    ./devel/lib/cargo_ants_path_planner/path_planner alice bob
    ./devel/lib/cargo_ants_path_adaptor/path_adaptor alice bob
    ./devel/lib/sfl2/nepumuk -c src/sfl2/ros/cargo-ants.yaml
    rostopic pub -1 /task cargo_ants_msgs/Task '{task_id: 1, vehicle: alice, goals: [{gx: 0, gy: 40, gth: -1.5, dr: 1, dth: 0.1}, {gx: 15, gy: -5, gth: 0, dr: 1, dth: -1}, {gx: 40, gy: 20, gth: -0.7, dr: 1, dth: 0.1}]}'
    rostopic pub -1 /task cargo_ants_msgs/Task '{task_id: 1, vehicle: bob, goals: [{gx: 20, gy: -6, gth: 0, dr: 1, dth: -1}, {gx: 30, gy: -5, gth: 0, dr: 1, dth: -1}, {gx: 40, gy: -6, gth: 0, dr: 1, dth: -1}, {gx: 45, gy: 5, gth: 0, dr: 1, dth: -1}, {gx: 40, gy: 15, gth: 0, dr: 1, dth: -1}]}'

Updating Instructions
---------------------

For the full setup, you need to separately update estar2 and sfl2 (for now at least).

    cd src/estar2
    git pull
    cd ../sfl2
    git pull
    cd ../..
    catkin_make
