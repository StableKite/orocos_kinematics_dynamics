# Kinematics and Dynamics Library

[![CI](https://github.com/orocos/orocos_kinematics_dynamics/workflows/CI/badge.svg)](https://github.com/orocos/orocos_kinematics_dynamics/actions)

Orocos project to supply RealTime usable kinematics and dynamics code,
it contains code for rigid body kinematics calculations and
representations for kinematic structures and their inverse and forward
kinematic solvers.

The C++ library is located in the `orocos_kdl` folder. The installation instructions can be found in
[INSTALL.md](orocos_kdl/INSTALL.md).

The python bindings, are located in the `python_orocos_kdl` folder. The installation instructions can be found in
[INSTALL.md](python_orocos_kdl/INSTALL.md).

Always use the same version of the C++ library and the python bindings. As a mismatch between these two can cause many issues.

With a free-threaded CPython build, PyKDL keeps the GIL disabled at import.
Use a pybind11 release that supports your Python version and free threading.
Threads can run independent solvers concurrently, including solvers that share
a chain that stays read-only during those calls. Keep the solver workspaces,
their nested solvers, and writable output objects local to each thread. Use
external synchronization when sharing them or accessing any object that another
thread can mutate. This includes borrowed members such as `Frame.p` and shared
arrays or lists used as inputs or outputs.

Also when using ROS/catkin, it is preferred to use the catkin installation method over the `cmake/make` method.

There will be no ROS Noetic release.
