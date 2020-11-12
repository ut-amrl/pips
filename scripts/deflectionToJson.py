#!/usr/bin/env python3
import sys
import json

# Check for input file
if (len(sys.argv) != 2):
  print(len(sys.argv))
  raise SystemExit

dataFile = sys.argv[1]
outputFile = "window_subsampled.txt"

# Deflection
# 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16
# count, last_state, current_pose.x, current_pose.y, current_pose.angle,
# current_vel.x, current_vel.y, current_vel.angle, ball_pose.x, ball_pose.y,
# ball_vel.x, ball_vel.y, kick_count_, angle_diff, x_dist_intercept, y_dist_intercept,
# time_diff, interception_point.x, interception_point.y, output_state

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

    example['pose'] = {}
    example['pose']['name'] = 'pose'
    example['pose']['value'] = [float(line[2]), float(line[3])]
    example['pose']['type'] = 'VEC'
    example['pose']['dim'] = [1, 0, 0]

    example['angle'] = {}
    example['angle']['name'] = 'angle'
    example['angle']['value'] = float(line[4])
    example['angle']['type'] = 'NUM'
    example['angle']['dim'] = [0, 0, 0]

    example['vel'] = {}
    example['vel']['name'] = 'vel'
    example['vel']['value'] = [float(line[5]), float(line[6])]
    example['vel']['type'] = 'VEC'
    example['vel']['dim'] = [1, -1, 0]

    example['vel_angle'] = {}
    example['vel_angle']['name'] = 'vel_angle'
    example['vel_angle']['value'] = float(line[7])
    example['vel_angle']['type'] = 'NUM'
    example['vel_angle']['dim'] = [0, 0, 0]

    example['ball_pose'] = {}
    example['ball_pose']['name'] = 'ball_pose'
    example['ball_pose']['value'] = [float(line[8]), float(line[9])]
    example['ball_pose']['type'] = 'VEC'
    example['ball_pose']['dim'] = [1, 0, 0]

    example['ball_vel'] = {}
    example['ball_vel']['name'] = 'ball_vel'
    example['ball_vel']['value'] = [float(line[10]), float(line[11])]
    example['ball_vel']['type'] = 'VEC'
    example['ball_vel']['dim'] = [1, -1, 0]

    example['kick_count'] = {}
    example['kick_count']['name'] = 'kick_count'
    example['kick_count']['value'] = float(line[12])
    example['kick_count']['type'] = 'NUM'
    example['kick_count']['dim'] = [0, 0, 0]

    example['angle_diff'] = {}
    example['angle_diff']['name'] = 'angle_diff'
    example['angle_diff']['value'] = float(line[13])
    example['angle_diff']['type'] = 'NUM'
    example['angle_diff']['dim'] = [0, 0, 0]

    example['dist_from_intercept'] = {}
    example['dist_from_intercept']['name'] = 'dist_from_intercept'
    example['dist_from_intercept']['value'] = [float(line[14]), float(line[15])]
    example['dist_from_intercept']['type'] = 'VEC'
    example['dist_from_intercept']['dim'] = [1, 0, 0]

    example['time_diff'] = {}
    example['time_diff']['name'] = 'time_diff'
    example['time_diff']['value'] = float(line[16])
    example['time_diff']['type'] = 'NUM'
    example['time_diff']['dim'] = [0, 1, 0]

    example['intercept_point'] = {}
    example['intercept_point']['name'] = 'intercept_point'
    example['intercept_point']['value'] = [float(line[17]), float(line[18])]
    example['intercept_point']['type'] = 'VEC'
    example['intercept_point']['dim'] = [1, 0, 0]

    example['output'] = {}
    example['output']['name'] = 'output'
    example['output']['value'] = line[-1]
    example['output']['type'] = 'STATE'
    example['output']['dim'] = [0, 0, 0]

    examples.append(example)

with open("deflection.json", "w") as output:
  output.write(json.dumps(examples, indent=4, sort_keys=True))
