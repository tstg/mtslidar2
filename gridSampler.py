import numpy as np


class GridSampler:

	def __init__(self):
		self.r_fp = 1
		self.num = 100

	def getSample(self):
		for i in np.linspace(-1, 1, self.num, endpoint=False):
			for j in np.linspace(-1, 1, self.num, endpoint=False):
				if i * i + j * j < 1:
					sample = np.array([i, j])
					yield sample
