import numpy as np


class GridSampleSampler:

	def __init__(self):
		self.r_fp = 1
		self.num = 50

	def getSample(self):
		for i in np.linspace(-1, 1, self.num, endpoint=False):
			for j in np.linspace(-1, 1, self.num, endpoint=False):
				for k in np.linspace(-1, 1, self.num, endpoint=False):
					for l in np.linspace(-1, 1, self.num, endpoint=False):
						sample = np.array([i, j])
						if np.linalg.norm(sample) < 1:
							yield sample
