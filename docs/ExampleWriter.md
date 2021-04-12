example_writer.hpp
==================

example_writer.hpp is a zero-dependency header-only C++ library for easily
generating examples for PIPS from your C++ programs.

Using example_writer.hpp
------------------------

To get started, just put a copy of example_writer.hpp anywhere in your project
and #include it.

```c++
#include "example_writer.hpp"
```

Then construct an ExampleWriter object with the name desired output file:
```c++
pips::ExampleWriter ew("my_example.json");
```

A "step" is a set of variables associated with a certain timestep. To use
ExampleWriter, you must create a step, add variables, and close the step.
Here's a toy example of that working:
```c++
for (int i = 0; i < 3; ++i) {
    int dims[3] = {0, 0, 0};
    ew.InitStep();
    ew.AddNumber("distance_to_robot", dims, i);
    ew.AddNumber("distance_to_wall", dims, 10-i);
    ew.AddState("output", "go");
    ew.CloseStep();
}
```

Your output JSON file should look like this:
```json
[
  {
    "distance_to_robot": {
      "dim": [0, 0, 0],
      "type": "NUM",
      "name": "distance_to_robot",
      "value": 0.000000
    },
    "distance_to_wall": {
      "dim": [0, 0, 0],
      "type": "NUM",
      "name": "distance_to_wall",
      "value": 10.000000
    },
    "output": {
      "dim": [0, 0, 0],
      "type": "STATE",
      "name": "output",
      "value": "go"
    }
  },
  {
    "distance_to_robot": {
      "dim": [0, 0, 0],
      "type": "NUM",
      "name": "distance_to_robot",
      "value": 1.000000
    },
    "distance_to_wall": {
      "dim": [0, 0, 0],
      "type": "NUM",
      "name": "distance_to_wall",
      "value": 9.000000
    },
    "output": {
      "dim": [0, 0, 0],
      "type": "STATE",
      "name": "output",
      "value": "go"
    }
  },
  {
    "distance_to_robot": {
      "dim": [0, 0, 0],
      "type": "NUM",
      "name": "distance_to_robot",
      "value": 2.000000
    },
    "distance_to_wall": {
      "dim": [0, 0, 0],
      "type": "NUM",
      "name": "distance_to_wall",
      "value": 8.000000
    },
    "output": {
      "dim": [0, 0, 0],
      "type": "STATE",
      "name": "output",
      "value": "go"
    }
  }
]
```