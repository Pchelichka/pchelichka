import numpy as np
from scipy.signal import savgol_coeffs

print(f'Position coefficients: ', savgol_coeffs(13, 7, deriv=0, pos=6, use ='dot'))
print(f'First derivative coefficients: ', savgol_coeffs(13, 7, deriv=1, pos=6, use ='dot'))
print(f'Second derivative coefficients: ', savgol_coeffs(13, 7, deriv=2, pos=6, use ='dot'))