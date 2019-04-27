import numpy as np

from tracer import Tracer
from lidar import Lidar
from recorder import Recorder
from gridSampler import GridSampler


class Scene:

	def __init__(self):
		self.sampler = GridSampler()
		self.recorder = Recorder()
		self.lidar = Lidar()
		self.tracer = Tracer(self.lidar, self.recorder)

	def render(self):

		samples = [sample for sample in self.sampler.getSample()]
		# self.saveSample(samples)
		weights = self.getWeights(samples)
		num = len(samples)
		k = 0
		for sample, weight in zip(samples, weights):

			if k % 10000 == 0:
				print('%d / %d' % (k, num))
			k += 1

			ray = self.lidar.generateRay(sample)
			# initialize the energy of the ray
			ray.energy = self.lidar.pulseEnergy * weight
			self.tracer.trace(ray, 0, 0)

		# records = np.array(self.tracer.records)
		# np.savetxt('records.txt', records)
		self.recorder.save()

	def getWeights(self, samples):
		sampleRadii = np.array([np.linalg.norm(sample) for sample in samples])
		weights = np.exp(-sampleRadii * sampleRadii * 0.5 / self.lidar.sigmaBeam2)
		weights = weights / np.sum(weights)
		return weights

	def saveSample(self, sample):
		npsample = np.array(sample)
		np.savetxt('imPulseFile.txt', npsample)

	def renderUsingSimulatedPhoton(self):
		""" According to DART manual.

		:return:
		"""
		minPerSubcenter = 5

		samples = self.sampler.getSample()
		weights = self.getWeights(samples)
		minWeight = min(weights)

		# get photon number per subcenter
		subcenterPhotonNumbers = []
		for weight in weights:
			subcenterPhotonNumber = int(weight / minWeight * minPerSubcenter)  # TODO little number may not be divisor

			subcenterPhotonNumbers.append(subcenterPhotonNumber)

		# be sure that subcenter with smallest weight has minimun photon
		# and keep gaussian shape pulse

		actualPhotonNumber = sum(subcenterPhotonNumbers)
		#
		energy = self.lidar.pulseEnergy / actualPhotonNumber

		for sample, subcenterPhotonNumber in zip(samples, subcenterPhotonNumbers):
			for i in range(subcenterPhotonNumber):
				ray = self.lidar.generateRay(sample)
				ray.energy = energy
				self.tracer.trace(ray, 0, 0)

	def renderUsingMultiLaunch(self):
		n = 5

		samples = [sample for sample in self.sampler.getSample()]
		# self.saveSample(samples)
		weights = self.getWeights(samples)

		num = len(samples)

		k = 0
		for sample, weight in zip(samples, weights):

			if k % 10000 == 0:
				print('%d / %d' % (k, num))
			k += 1

			for i in range(n):
				ray = self.lidar.generateRay(sample)
				# initialize the energy of the ray
				ray.energy = self.lidar.pulseEnergy * weight / n
				self.tracer.trace(ray, 0, 0)

		# records = np.array(self.tracer.records)
		# np.savetxt('records.txt', records)
		self.recorder.save()