#!/usr/bin/env python3

import math

# convert hours minutes seconds coordinates to degrees
def dms_to_dmm(dms):
    minp = dms.split('.')
    degrees = math.trunc(float(dms))
    minutes = int(minp[1][:2])
    seconds = str(float(f'{minp[1][2:4]}.{minp[1][4:]}')/60)[2:]
    return float(f'{degrees}.{minutes}{seconds}')


lat, lon = (input("Enter provided coordinates, comma separated: ")).split(',')
print(dms_to_dmm(lat), dms_to_dmm(lon))

