# PIPS

Physics Informed Program Synthesis

## Related Publications
[SRTR](https://arxiv.org/pdf/1802.01706.pdf) Jarrett Holtz, Arjun Guha, and Joydeep Biswas. Interactive robot transition repair with SMT. IJCAI. 2018.

[PIPS](https://arxiv.org/pdf/2008.04133.pdf) Jarrett Holtz, Arjun Guha, and Joydeep Biwas. Robot Action Selection Learning via Layered Dimension Informed Program Synthesis. CORL. 2020.

[IDIPS](https://arxiv.org/pdf/2103.04880.pdf) Jarrett Holtz, Simon Andrews, Arjun Guha, and Joydeep Biswas. Iterative Program Synthesis for Adaptable Social Navigation. IROS. 2021.
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

### Defining a problem
cpp-pips helps you find state transition functions. For example, suppose you have a robot driving around humans. If the distance between the robot and its nearest human is less than 1, it should stop. Otherwise, it should drive. The transitions and conditions would look something like this:

| Transition   | Condition |
|--------------|-----------|
| STOP -> STOP | d < 1     |
| STOP -> GO   | d >= 1    |
| GO -> STOP   | d < 1     |
| GO -> GO     | d >= 1    |

To use cpp-pips to solve this problem, you will need
 1. a "library" of functions for the synthesizer to consider using, and
 2. a bunch of examples.

### Defining a library
A library is a JSON file that tells cpp-pips which functions it can use and on what types of data.

The file is a list of objects. Each object has the following fields:
 * `op` (string): The operation to be run. See below for a list of implemented operations. A list of implemented library functions is in src/ast/library_functions.hpp.
 * `inputType` (list of strings): The allowed types for the inputs to the operation. Valid types are NUM, BOOL, STATE (string), VEC (2D float vector)
 * `inputDim` (list of lists of integers): The allowed dimensionalities for the inputs to the operation. Should have 3 elements. The first represents the power of distance, the second time, and the third mass. For example, m/s would be [1, -1, 0].
 * `outputType` (string): The type of the operation's output.
 * `outputDim` (list of integers): The dimensionality of the output.

For example, if we wanted the synthesizer to be able to add masses and multiply times we would have

```json
[
    {
        "op": "Plus",
        "inputType": ["NUM", "NUM"],
        "inputDim": [[1, 0, 0], [1, 0, 0]],
        "outputType": "NUM",
        "outputDim": [1, 0, 0]
    },
    {
        "op": "Times",
        "inputType": ["NUM", "NUM"],
        "inputDim": [[0, 0, 1], [0, 0, 1]],
        "outputType": "NUM",
        "outputDim": [0, 0, 2]
    }
]
```

This simple example file will suffice for the scenario described above, since we don't actually need any operations for a correct solution.

### Making examples
An example file is a JSON file tht records
 1. what state transitions were made, and
 2. what the state of the robot and the world was at the time each transition was made.
Every example file is a JSON file that is a list of JSON objects. Each JSON object represents one single example.

One such object has variables representing the state, as well as two special variables named "start" and "output" which are the initial and final states of the transition respectively. For example, here's what one object might look like
```json
{
    "distance_to_human": {
        "dim": [1, 0, 0],
        "type": "NUM",
        "name": "distance_to_human",
        "value": 0.95
    },
    "start": {
        "dim": [0, 0, 0],
        "type": "STATE",
        "name": "output",
        "value": "GO"
    },
    "output": {
        "dim": [0, 0, 0],
        "type": "STATE",
        "name": "output",
        "value": "STOP"
    }
}
```
This object is an example of the robot transition from GO to STOP when the nearest human is 0.95 meters away.

Of course, you would need many more examples to get any good results. A sample example file `toy.json` and a script for generating more like it `toy.py` are available in the examples directory.

### Running
With your library file (let's call it `ops/social_test.json`) and your examples file (let's call it `examples/toy.json`) its time to run cpp-pips. In a terminal, type:

` ./bin/ldips -debug -ex_file examples/toy.json -lib_file ops/social_ref.json`. 

You should see output similar to the following:
```
----- Transition Demonstrations -----
GO->GO : 38
STOP->GO : 31
GO->STOP : 17
STOP->STOP : 14

----Roots----
distance_to_human [1, 0, 0]

----Transitions----
STOP->STOP
GO->STOP
STOP->GO
GO->GO

----Library----
SqDist: VEC [1, 0, 0], VEC [1, 0, 0] --> NUM [1, 0, 0]
SqDist: VEC [1, -1, 0], VEC [1, -1, 0] --> NUM [1, -1, 0]
Minus:  NUM [0, 0, 0], NUM [0, 0, 0] --> NUM [0, 0, 0]
Minus:  VEC [1, 0, 0], VEC [1, 0, 0] --> VEC [1, 0, 0]
Angle:  VEC [1, 0, 0] --> NUM [0, 0, 0]
Angle:  VEC [1, -1, 0] --> NUM [0, 0, 0]

---- Number of Features Enumerated ----
1


----- STOP->STOP -----
Score: 1
Final Solution: Lt(fX1=[distance_to_human], pX1=[1.004655])

----- GO->STOP -----
Score: 1
Final Solution: Lt(fX1=[distance_to_human], pX1=[1.096597])

----- STOP->GO -----
Score: 1
Final Solution: Gt(fX1=[distance_to_human], pX1=[0.834170])

----- GO->GO -----
Score: 1
Final Solution: And(Gt(fX2=[distance_to_human], pX2=[0.100041]), Gt(fX1=[distance_to_human], pX1=[1.009065]))

Run-time stats for PredicateL2 : mean run time = 14.034319 ms, invocations = 8
Run-time stats for MakeSMTLIBProblem : mean run time = 0.176692 ms, invocations = 52
Run-time stats for SolveSMTLIBProblem : mean run time = 11.326627 ms, invocations = 48
Run-time stats for CheckModelAccuracy : mean run time = 0.045132 ms, invocations = 46
Run-time stats for GetLegalOperations : mean run time = 0.001569 ms, invocations = 2
Run-time stats for RecEnumerate : mean run time = 24.785377 ms, invocations = 1
Run-time stats for UpdateList : mean run time = 0.000371 ms, invocations = 2
Run-time stats for Enum : mean run time = 0.001843 ms, invocations = 3
Run-time stats for CalcSigs : mean run time = 8.244096 ms, invocations = 3
```

Not bad! Maybe with a few more examples we could get exactly the results we're looking for...

## Extended Examples
Try some real world social navigation scenarios! A library for this purpose (`ops/social_test.json`) and some recorded demonstrations (`examples/del_N.json`) are included in this repository.

## Adapting a Policy with IDIPS
Iterative Dimension Informed Program Synthesis (IDIPS) minimally adapts an existing policy to best accomodate new demonstrations. Using IDIPS is very similary to using PIPS/LDIPS as demonstrated above, you still require a library json file, and json file filled with examples. In addition, IDIPS requires the path to a policy as output by PIPS, provided with the `flag -sketch-dir`. 

In this repo we have provided an additional toy example `toy-2.json` that represents a second reason for our example agent to stop. In this case, we have created a new observation representing the position of a truck, and demonstrations that show the agent stopping when the truck is within a certain distance. To apply IDIPs to synthesize new rules for this previously unseen information, call the following in your terminal:

`./bin/idips -debug -lib_file ops/exampl_lib.json -ex_file examples/toy-2.json -sketch_dir synthd/pips_output/`

You should see output similar to the following, where IDIPs has synthesized new clauses for the previous predicates based on the truck_pose:
```
----- Transition Demonstrations -----
GO->STOP : 43
STOP->STOP : 35
STOP->GO : 12
GO->GO : 10

Examples Loaded: 100

----Roots----
truck_pose [1, 0, 0]
start [0, 0, 0]
distance_to_human [1, 0, 0]

Prog Size: 2
----Transitions----
GO->GO
STOP->GO
STOP->STOP
GO->STOP

----Library----
SqDist:	VEC [1, 0, 0], VEC [1, 0, 0] --> NUM [1, 0, 0]
Abs:	NUM [0, 0, 0] --> NUM [0, 0, 0]
Norm:	VEC [1, 0, 0] --> NUM [1, 0, 0]
Norm:	VEC [1, -1, 0] --> NUM [1, -1, 0]
Plus:	NUM [1, 0, 0], NUM [1, 0, 0] --> NUM [1, 0, 0]
Times:	NUM [0, 0, 1], NUM [0, 0, 1] --> NUM [0, 0, 2]

---- Features Synthesized ----
truck_pose [1, 0, 0]
start [0, 0, 0]
distance_to_human [1, 0, 0]
Norm(truck_pose)
start [0, 0, 0]
SqDist(truck_pose, truck_pose)
Plus(distance_to_human, distance_to_human)
Plus(distance_to_human, Norm(truck_pose))
Plus(Norm(truck_pose), Norm(truck_pose))

---- Number of Features Enumerated ----
9


Num Examples: 100
----- STOP->GO -----
Initial Program:>(distance_to_human, 0.899374)
Initial Score: 0.255319
Pos: 0 Neg: 35
>(distance_to_human, 0.899374) && >(, )
PosE: 12 NegE: 35
Current Sketch: >(distance_to_human, 0.899374) && >(, )
Num Examples: 47
Final Solution: >(distance_to_human, 0.899374) && >(Norm(truck_pose), 8.837197)
Final Score: 1

----- GO->STOP -----
Initial Program:<(distance_to_human, 1.227231)
Initial Score: 0.188679
Pos: 43 Neg: 0
<(distance_to_human, 1.227231) || >(, )
PosE: 43 NegE: 10
Current Sketch: <(distance_to_human, 1.227231) || >(, )
Num Examples: 53
Pos: 43 Neg: 0
<(distance_to_human, 1.227231) || <(, )
PosE: 43 NegE: 10
Current Sketch: <(distance_to_human, 1.227231) || <(, )
Num Examples: 53
Final Solution: <(distance_to_human, 1.227231) || <(Norm(truck_pose), 9.568246)
Final Score: 1

Run-time stats for PredicateL2 : mean run time = 28.392529 ms, invocations = 3
Run-time stats for MakeSMTLIBProblem : mean run time = 0.271669 ms, invocations = 18
Run-time stats for SolveSMTLIBProblem : mean run time = 3.351814 ms, invocations = 18
Run-time stats for CheckModelAccuracy : mean run time = 0.075824 ms, invocations = 20
Run-time stats for GetLegalOperations : mean run time = 0.001352 ms, invocations = 7
Run-time stats for RecEnumerate : mean run time = 42.002851 ms, invocations = 1
Run-time stats for UpdateList : mean run time = 0.000123 ms, invocations = 7
Run-time stats for Enum : mean run time = 0.006168 ms, invocations = 2
Run-time stats for CalcSigs : mean run time = 20.983932 ms, invocations = 2
```

## License

This project is released under the MIT License. Please review the [License file](LICENSE) for more details.
