import cmath
import math

def sub_angles(x, y):
    a = (x - y + 2 * cmath.pi) % (2 * cmath.pi)
    if a > cmath.pi:
        a -= 2 * cmath.pi
    return a

encoder_weight = .9
x = float( input("encoder: "))
y = float(input("gps: "))
z = x
diff = sub_angles(x, y)
print(diff)
z = (x - diff*(1-encoder_weight)) % (math.pi*2)
if z > math.pi:
    z=z - 2*math.pi
    
print(z)
