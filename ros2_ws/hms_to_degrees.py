#!/usr/bin/env python3

# convert hours minutes seconds coordinates to degrees
def hms_to_degrees(hms):
    hms = hms.split(' ')
    hours = int(hms[0])
    if len(hms) == 4 and (hms[3] == 'S' or hms[3] == 'W'):
        hours *= -1

    return f"{hours + int(hms[1]) / 60 + int(hms[2]) / 3600}"


hms = input("Enter space separated coordinates of latitude: ")
print(hms_to_degrees(hms))

hms = input("Enter space separated coordinates of latitude: ")
print(hms_to_degrees(hms))
