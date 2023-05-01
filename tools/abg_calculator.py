################################
# File: abg_calculator.py
# Purpose: calculate the abg parameters according to the method found in J. E. Gray and W. Murray, 
#          "A derivation of an analytic expression for the tracking index for the alpha-beta-gamma 
#          filter," in IEEE Transactions on Aerospace and Electronic Systems, vol. 29, no. 3, pp. 
#          1064-1065, July 1993, doi: 10.1109/7.220956.
# Date Created: 11 Feb 2023
################################

import math

T = float(input("Enter time step in seconds: "))
sigma_w = float(input("Enter process std dev: "))
sigma_v = float(input("Enter noise std dev: "))

# intermediate calculations
l = sigma_w / sigma_v * pow(T,2)
b = l/2 - 3
c = l/2 + 3
d = -1
p = c - pow(b,2)/3
q = 2*pow(b,3)/27 - b*c/3 + d
v = math.sqrt(pow(q,2) + 4*pow(p,3)/27)
z = - pow((q + v/2),(1/3))
s = z - p/3/z - b/3

alpha = 1 - pow(s,2)
beta  = 2*pow(1-s,2)
gamma = pow(beta,2)/2/alpha

print("a = ", alpha, ", b = ", beta, ", g = ", gamma)