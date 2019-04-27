import numpy as np
from mitsuba.core import *

import transform



class Lidar:

	def __init__(self):
		self.position = np.array([0, 10000, 0, 1])  # (meter)
		self.aperture = 0.1  # telescope aperture area (square meter)
		self.divergence = 1.2 * 1e-3  # half angle (radians)
		self.fov = 1.5 * 1e-3  # half angle (radians)
		self.lookAt = np.array([0, -1, 0, 0])

		# calculate the convergence point
		self.r = np.sqrt(self.aperture / np.pi)  # telescope radius
		self.convergence = self.position - self.r / np.tan(self.fov) * self.lookAt

		# rotationMatrix for transforming ray to world
		self.rotationMatrix = np.eye(4)

		# pulse
		self.pulseEnergy = 1e-3
		# self.sigmaBeam2 = 1.0 / 9
		self.fractionAtRadius = 0.368
		self.sigmaBeam2 = - 0.5 / np.log(self.fractionAtRadius)

		# auxiliary
		self.cosFov = np.cos(self.fov)

	def configure(self):
		''' Configure lidar.

		Called once after construction of Lidar instance.
		'''

		# normalize lookAt vector
		self.lookAt = self.lookAt / np.linalg.norm(self.lookAt)

		# calculate convergence point
		self.r = np.sqrt(self.aperture / np.pi)
		self.convergence = self.position - self.r / np.tan(self.fov) * self.lookAt

		# calculate the rotation matrix
		z = np.array([0, -1, 0])
		n = self.lookAt[:3]

		self.rotationMatrix = np.eye(4)
		axis = np.cross(z, n)
		if np.abs(np.dot(axis, axis)) > 0:
			degrees = np.arccos(np.dot(n, z))
			degrees = np.rad2deg(degrees)

			self.rotationMatrix[:3, :3] = transform.rotate(degrees, axis)

		# auxiliary
		self.cosFov = np.cos(self.fov)
		self.sigmaBeam2 = - 0.5 / np.log(self.fractionAtRadius)

	def generateRay(self, sample):
		# transform unit circle to ground
		ALTITUDE = self.position[1]
		sample = np.array(sample * np.tan(self.divergence) * ALTITUDE)
		# target = np.array([sample[0], sample[1], 0, 1])
		target = np.array([sample[0], 0, sample[1], 1])
		direction = np.array(target - self.position)

		# rotate the ray
		direction = np.matmul(self.rotationMatrix, direction)
		direction = direction / np.linalg.norm(direction)

		# instantialize a mitsuba ray
		ray = Ray()
		ray.setOrigin(Point(self.position[0], self.position[1], self.position[2]))
		ray.setDirection(Vector(direction[0], direction[1], direction[2]))
		# The energy of the ray has not been initialized yet.

		return ray
