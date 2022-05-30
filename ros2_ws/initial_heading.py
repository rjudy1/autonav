# This computes the initial heading based upon the iPhone compass
# reading. The reading gives degrees clockwise from north but the
# method used by latitude longitude would give heading counterclockwise
# from east. This should be entered into the params.yml under fusion
# initial heading

#!/usr/bin/env python3

import math
import sys

if len(sys.argv) > 1:
  angle = float(sys.argv[1])
else:
  angle = int(input("angle: "))

heading = (360 - angle + 90) * math.pi / 180
if angle > math.pi:
  heading -= math.pi * 2

print(heading)
