import os 
import numpy as np

import const 

INDEX_OF_TIME = 3
INDEX_OF_ENERGY = 4
INDEX_OF_DEPTH = 5


class WaveformRecorder:
	""" Accumulate energy in bins immidiately when commit is called
		output the accumulation result file
		save total and 1st.
	"""

	def __init__(self):
		self.outPath = 'out/'
		self.label = 'record'

		# accumulate
		self.energys = []
		self.firstEnergys = []
		self.firstEnergy = 0
		self.totalEnergy = 0
		self.altitude = 10000
		self.dt = 1e-9
		self.height = 15
		self.bins = []
		self.times = []
		self.highest = 2 * (self.altitude + self.height) / const.LIGHT_SPEED * 1e9
		self.lowest = 2 * (self.altitude - self.height) / const.LIGHT_SPEED * 1e9

		self.configure()

	def configure(self):
		""" Get intervals
		"""
		self.firstEnergy = 0
		self.totalEnergy = 0

		self.bins = []
		self.bins.append(0)
		time = self.dt * 1e9
		while time * const.LIGHT_SPEED / 1e9 <= self.height:
			self.bins.append(time)
			self.bins.append(-time)
			time += self.dt * 1e9
		self.bins.sort()

		numBin = len(self.bins)
		self.bins = np.array(self.bins)
		self.times = np.array(self.bins)
		self.bins = self.bins + 2 * self.altitude / const.LIGHT_SPEED * 1e9  # - self.dt * 1e9 * 0.5

		self.energys = np.zeros(numBin)
		self.firstEnergys = np.zeros(numBin)

		self.highest = 2 * (self.altitude + self.height) / const.LIGHT_SPEED * 1e9
		self.lowest = 2 * (self.altitude - self.height) / const.LIGHT_SPEED * 1e9

	def commit(self, record):
		length = record[INDEX_OF_TIME] / const.LIGHT_SPEED * 1e9
		energy = record[INDEX_OF_ENERGY]

		self.totalEnergy += energy
		if not self.lowest <= length <= self.highest:
			return

		idx = self.lower_bound(self.bins, length) - 1
		if record[INDEX_OF_DEPTH] == 0:
			self.firstEnergy += energy
			self.firstEnergys[idx] += energy
		self.energys[idx] += energy

	def convolve(self, accumulation):
		pulse = self.getPulse()
		convolvedWaveform = np.convolve(accumulation, pulse, 'same')
		return convolvedWaveform

	def save(self):
		energys = np.array([self.times, self.energys]).T
		firstEnergys = np.array([self.times, self.firstEnergys]).T
		convolvedEnergys = np.array([self.times, self.convolve(self.energys)]).T
		convolvedFirstEnergys = np.array([self.times, self.convolve(self.firstEnergys)]).T
		np.savetxt(self.outPath + 'accum-' + self.label + '.txt', energys)
		np.savetxt(self.outPath + 'accum1st-' + self.label + '.txt', firstEnergys)
		np.savetxt(self.outPath + 'conv-' + self.label + '.txt', convolvedEnergys)
		np.savetxt(self.outPath + 'conv1st-' + self.label + '.txt', convolvedFirstEnergys)

	def getPulse(self):
		numberOfSigma = 3
		relativePower = 0.5
		durationAtRelativePower = 2.0  # half pulse duration at relative power [ns]
		sigmaPulse = durationAtRelativePower / np.sqrt(2.0) / np.sqrt(-np.log(relativePower))
		print('sigmaPulse = %f' % sigmaPulse)
		n = int(np.round((2 * numberOfSigma * sigmaPulse / (self.dt * 1e9) - 1) / 2))
		print('n = %d' % n)
		x = np.linspace(-numberOfSigma * sigmaPulse, numberOfSigma * sigmaPulse, 2 * n + 1)
		pulse = np.exp(-0.5 * x * x / (sigmaPulse * sigmaPulse))
		pulse = pulse / np.sum(pulse)
		return pulse

	@staticmethod
	def lower_bound(nums, target):
	    low, high = 0, len(nums)-1
	    pos = len(nums)
	    while low<high:
	        mid = (low+high)/2
	        if nums[mid] < target:
	            low = mid+1
	        else:#>=
	            high = mid
	            #pos = high
	    if nums[low]>=target:
	        pos = low
	    return pos




