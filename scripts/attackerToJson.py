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
# ball_vel.x, ball_vel.y,
# target_angle, kick_count_, attacker_dist, ball_speed, intersect_distance,
# robot_prime_vel.x, robot_prime_vel.y, ball_prime_vel.x, ball_prime_vel.y
# robot_to_ball_x, robot_to_ball.y, relative_vel.x, relative_vel.y,
# angle_diff, angular_vel, radial_dist, relative_vel, output_state

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
    example['kick_count']['name'] = 'target_angle'
    example['kick_count']['value'] = float(line[12])
    example['kick_count']['type'] = 'NUM'
    example['kick_count']['dim'] = [0, 0, 0]

    example['kick_count'] = {}
    example['kick_count']['name'] = 'kick_count'
    example['kick_count']['value'] = float(line[13])
    example['kick_count']['type'] = 'NUM'
    example['kick_count']['dim'] = [0, 0, 0]

    example['prime_vel'] = {}
    example['prime_vel']['name'] = 'prime_vel'
    example['prime_vel']['value'] = [float(line[17]), float(line[18])]
    example['prime_vel']['type'] = 'VEC'
    example['prime_vel']['dim'] = [1, 0, 0]

    example['ball_prime_vel'] = {}
    example['ball_prime_vel']['name'] = 'ball_prime_vel'
    example['ball_prime_vel']['value'] = [float(line[19]), float(line[20])]
    example['ball_prime_vel']['type'] = 'VEC'
    example['ball_prime_vel']['dim'] = [1, 0, 0]

    example['output'] = {}
    example['output']['name'] = 'output'
    example['output']['value'] = line[-1]
    example['output']['type'] = 'STATE'
    example['output']['dim'] = [0, 0, 0]

    examples.append(example)

with open("attacker.json", "w") as output:
  output.write(json.dumps(examples, indent=4, sort_keys=True))
