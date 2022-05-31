# This computes the initial heading based upon the iPhone compass
# reading. This should be entered into the params.yml under fusion
# initial heading

#!/usr/bin/env python3

import math
import sys

if len(sys.argv) > 1:
  angle = float(sys.argv[1])
else:
  angle = int(input("angle: "))

heading = angle * math.pi / 180
if angle > 180:
  heading -= math.pi * 2

print(heading)
