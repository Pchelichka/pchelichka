from scipy import signal
import numpy as np
import math

wc = 2 * np.pi * 20
num = wc
den = [1, wc]

lowPass = signal.TransferFunction(num, den)

print(lowPass)

dt = 1 / 30
discreteLowpass = lowPass.to_discrete(dt, method='gbt', alpha=0.5)
print(discreteLowpass)

b = discreteLowpass.num
a = -discreteLowpass.den
print(b)
print(a[1:])