# cpp-pips

Physics Informed Program Synthesis
 

## Make a Docker image (recommended)

1. Install [Docker](https://www.docker.com/get-started)
2. From this directory, run `git submodule update --init --recursive`. This will download copies of some dependencies.
2. From this directory, run `docker build . -t cpp-pips`. This will build a new Docker image with PIPS and its dependencies ready to go.
3. Run `docker run -it cpp-pips /bin/bash`. This will launch the cpp-pips image and put you in a Bash shell.

## Build it for yourself

### Dependencies
1. [ROS](http://wiki.ros.org/ROS/Installation) (most tested with Melodic, can be made to work on Noetic without too much trouble)
2. [tf](http://wiki.ros.org/tf) (`apt-get install ros-melodic-tf`)
3. [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) (`apt-get install libeigen3-dev`)
4. [Google Logging Library](https://github.com/google/glog) (`apt-get install libgoogle-glog-dev`)
5. [GoogleTest](https://github.com/google/googletest/blob/master/googletest/README.md) (the Ubuntu package doesn't distribute binaries, see https://www.eriksmistad.no/getting-started-with-google-test-on-ubuntu/ for build instructions)
6. [Z3](https://github.com/Z3Prover/z3/releases) (version 4.8.9)

### Compilation
Add the path to cpp-pips to your $ROS_PACKAGE_PATH, then run `make`.

## Example Usage

A description of an example toy problem, how to define the library for it, and the format of the input / output JSON files, followed by how to run the program and understand the output


## Extended Examples

Include JSON files for larger problems from actual experiments, and how to run them
