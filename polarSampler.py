import numpy as np

# footprint radius
R = 1.0


class PolarSampler:
	'''

	Reference: Wang Y, 2015.
	'''
	def __init__(self):
		self.rNum = 50
		self.tNum = 50

	def getSample(self):
		for r in np.linspace(R / self.rNum, R, self.rNum):
			for t in np.linspace(0, 2 * np.pi, self.tNum, endpoint=False):
				x = r * np.cos(t)
				y = r * np.sin(t)

				sample = np.array([x, y])
				yield sample