from collections import deque
import numpy as np
from scipy.signal import savgol_coeffs

WINDOW_LENGTH = 13
coeff0 = savgol_coeffs(WINDOW_LENGTH, 3, deriv=0, pos=6, use ='dot')
coeff1 = savgol_coeffs(WINDOW_LENGTH, 3, deriv=1, pos=6, use ='dot')
coeff2 = savgol_coeffs(WINDOW_LENGTH, 3, deriv=2, pos=6, use ='dot')

class SavgolFilteredData:
	def __init__(self):
		self.window = deque([])
		self.window_length = WINDOW_LENGTH
		self.value = None
		self.first_derivative_value = None
		self.second_derivative_value = None
		# self.coeff0 = [-0.04761905, 0.0952381, 0.02380952, -0.0952381, -0.0952381, 0.19047619, 0.92857143] 
		# self.coeff1 = [-0.30555556, 0.48412698, 0.30555556, -0.28571429, -0.73412698, -0.48412698, 1.01984127]
		# self.coeff2 = [-0.38095238, 0.5, 0.42857143, -0.0952381, -0.57142857, -0.5, 0.61904762]
		# self.coeff0 = [-0.00699301, 0.04195804, -0.05594406, -0.05594406, 0.08391608, 0.27972028, 0.3986014, 0.33566434, 0.09090909, -0.15384615, 0.04195804] 
		# self.coeff1 = [-0.05305944, 0.13758741, 0.00370047, -0.14941725, -0.18846154, -0.1037296, 0.03927739, 0.14825175, 0.15247669, 0.05122378, -0.03784965]
		# self.coeff2 = [ 0.0152972, -0.07604895, 0.0625, 0.13286713, 0.06002331, -0.08974359, -0.19055944, -0.13869464, 0.06570513, 0.23688811, -0.07823427]
		self.coeff0 = coeff0
		self.coeff1 = coeff1
		self.coeff2 = coeff2
		self.filtering = False

	def add(self, value: int, sample_time: float=1):
		self.window.append(value)
		self.value = value
		while len(self.window) > self.window_length:
			self.window.popleft()
		if len(self.window) == self.window_length:
			self.filtering = True
			self.value = 0
			self.first_derivative_value = 0
			self.second_derivative_value = 0
			for i, x in enumerate(self.window):
				self.value += x * self.coeff0[i]
				self.first_derivative_value += x * self.coeff1[i]
				self.second_derivative_value += x * self.coeff2[i]
			self.first_derivative_value /= sample_time	
			self.second_derivative_value /= sample_time * sample_time