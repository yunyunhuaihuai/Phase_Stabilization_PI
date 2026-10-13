# Dependency notices

This repository does not vendor third-party libraries. Existing copyright and
license notices in retained source files must be preserved. A project-wide
license has not been assigned.

The hardware build links the Red Pitaya SDK supplied by the operator. Python
hardware tools optionally import PySerial, PyVISA, PyVISA-py and Paramiko. The
optional plotting command imports Matplotlib. Install and redistribute these
dependencies according to their own copyright and license declarations.

The compiler, CMake, Python runtime, Git and CI actions are external build tools.
Their distributions and license files are not included in this repository.
