#!/usr/bin/env python3

"""
This file is part of the ddcontroller library (https://github.com/ansarid/ddcontroller).
Copyright (C) 2022  Daniyal Ansari

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
"""

import time
from as5048b import AS5048B

# Create encoder object
l_encoder = AS5048B(0x40, bus=9)
r_encoder = AS5048B(0x41, bus=9)

try:

    # Create infinite loop
    while True:

        # Read the encoder position
        l_position = l_encoder.read_position()
        r_position = r_encoder.read_position()

        # Read the encoder angle
        l_angle = l_encoder.read_angle()
        r_angle = r_encoder.read_angle()

        # Print out the encoder position and angle
        print(f"Left angle: {round(l_angle, 3)} rad\tpos: {l_position}\t Right angle: {round(r_angle, 3)} rad\tpos: {r_position}")

        # Run loop at 50Hz
        time.sleep(1/50)

except KeyboardInterrupt:
    pass

finally:
    pass