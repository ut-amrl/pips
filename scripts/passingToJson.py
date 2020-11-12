#!/usr/bin/env python3
import sys
import json

# Check for input file
if (len(sys.argv) != 2):
  print(len(sys.argv))
  raise SystemExit

dataFile = sys.argv[1]
outputFile = "window_subsampled.txt"

# Passing
# count, start, current_speed_, pose.x, pose.y, pose.angle, cruise_speed,
# other_lead.x, other_lead.y, other_lead.speed, other_rear.x, other_rear.y, other_rear.speed
# same_lead.x, same_lead.y, same_lead.speed, same_rear.x, same_rear.y, same_rear.speed, state_name

examples = []
with open(dataFile, "r") as data:
  for entry in data:
    example = {}
    # Split the line on commas
    line = entry.split(',')
    example['start'] = {}
    example['start']['name'] = 'start'
    example['start']['value'] = line[1]
    example['start']['type'] = 'STATE'
    example['start']['dim'] = [0, 0, 0]

    example['speed'] = {}
    example['speed']['name'] = 'speed'
    example['speed']['value'] = float(line[2])
    example['speed']['type'] = 'NUM'
    example['speed']['dim'] = [1, -1, 0]

    example['pose'] = {}
    example['pose']['name'] = 'pose'
    example['pose']['value'] = [float(line[3]), float(line[4])]
    example['pose']['type'] = 'VEC'
    example['pose']['dim'] = [1, 0, 0]

    example['angle'] = {}
    example['angle']['name'] = 'angle'
    example['angle']['value'] = float(line[5])
    example['angle']['type'] = 'NUM'
    example['angle']['dim'] = [0, 0, 0]

    example['cruise_speed'] = {}
    example['cruise_speed']['name'] = 'cruise_speed'
    example['cruise_speed']['value'] = float(line[6])
    example['cruise_speed']['type'] = 'NUM'
    example['cruise_speed']['dim'] = [1, -1, 0]

    example['other_lead'] = {}
    example['other_lead']['name'] = 'other_lead'
    example['other_lead']['value'] = [float(line[14]), float(line[15])]
    example['other_lead']['type'] = 'VEC'
    example['other_lead']['dim'] = [1, 0, 0]

    example['other_lead_speed'] = {}
    example['other_lead_speed']['name'] = 'other_lead_speed'
    example['other_lead_speed']['value'] = float(line[18])
    example['other_lead_speed']['type'] = 'NUM'
    example['other_lead_speed']['dim'] = [1, -1, 0]

    example['other_rear'] = {}
    example['other_rear']['name'] = 'other_rear'
    example['other_rear']['value'] = [float(line[16]), float(line[17])]
    example['other_rear']['type'] = 'VEC'
    example['other_rear']['dim'] = [1, 0, 0]

    example['other_rear_speed'] = {}
    example['other_rear_speed']['name'] = 'other_rear_speed'
    example['other_rear_speed']['value'] = float(line[19])
    example['other_rear_speed']['type'] = 'NUM'
    example['other_rear_speed']['dim'] = [1, -1, 0]

    example['same_lead'] = {}
    example['same_lead']['name'] = 'same_lead'
    example['same_lead']['value'] = [float(line[28]), float(line[29])]
    example['same_lead']['type'] = 'VEC'
    example['same_lead']['dim'] = [1, 0, 0]

    example['same_lead_speed'] = {}
    example['same_lead_speed']['name'] = 'same_lead_speed'
    example['same_lead_speed']['value'] = float(line[32])
    example['same_lead_speed']['type'] = 'NUM'
    example['same_lead_speed']['dim'] = [1, -1, 0]

    example['same_rear'] = {}
    example['same_rear']['name'] = 'same_rear'
    example['same_rear']['value'] = [float(line[30]), float(line[31])]
    example['same_rear']['type'] = 'VEC'
    example['same_rear']['dim'] = [1, 0, 0]

    example['same_rear_speed'] = {}
    example['same_rear_speed']['name'] = 'same_rear_speed'
    example['same_rear_speed']['value'] = float(line[33])
    example['same_rear_speed']['type'] = 'NUM'
    example['same_rear_speed']['dim'] = [1, -1, 0]

    example['output'] = {}
    example['output']['name'] = 'output'
    example['output']['value'] = line[-1]
    example['output']['type'] = 'STATE'
    example['output']['dim'] = [0, 0, 0]

    examples.append(example)

with open("passing.json", "w") as output:
  output.write(json.dumps(examples, indent=4, sort_keys=True))
