import re
import numpy as np
from mitsuba.core import *

from support import patchGenerator
import const


class Tracer:
    """ Trace a sample ray.

	"""

    def __init__(self, lidar, recorder):
        self.maxDepth = 1
        self.scene = None  # mitsuba scene
        # self.records = []  # should be replaced by a Recorder object soon
        self.lidar = lidar
        self.recorder = recorder

        self.directions = None
        self.numberOfDirection = 0

    def configure(self):
        self.directions = [direction for direction in patchGenerator.generatePatchesFromFile('directions.txt')]
        self.numberOfDirection = len(self.directions)

    # print(self.directions)

    def trace(self, ray, depth, distance):
        """ Trace a sample ray.

		Args:
			ray: mitsuba ray with energy
			depth: recurrent depth
			distance: corresponding to time lag from emitting to receiving
		"""

        if depth > self.maxDepth:
            return

        intersection = self.intersect(ray)
        if not intersection:
            # print('tracer no intersection');
            return

        eventRandom = np.random.uniform()
        propability = intersection['reflectance'] / (intersection['reflectance'] + intersection['transmittance'])
        scatterType = ['transmittance', 'reflectance'][eventRandom < propability]

        # if scatterType == 'reflectance':
        # is there energy sent to sensor ?
        returnEnergy = self.getReturnEnergy(ray, depth, intersection)
        # print('tracer returnEnergy', returnEnergy)
        if returnEnergy > 0:
            vectorToTelescope = self.lidar.position - intersection['point']
            returnLength = distance + intersection['length'] + np.linalg.norm(vectorToTelescope)
            # deltaTime = returnLength / const.LIGHT_SPEED
            deltaTime = returnLength
            self.receiveEnergy(intersection, deltaTime, returnEnergy, depth)

        # calculate new ray
        if intersection[scatterType] > 0:
            newRay = self.getScatterRay(intersection, scatterType)
            # scatterEnergy = ray.energy - returnEnergy
            # scatterEnergy = ray.energy * intersection[scatterType]
            scatterEnergy = ray.energy * (intersection['reflectance'] + intersection['transmittance'])
            newRay.energy = scatterEnergy

            # calculate new distance
            newDistance = distance + intersection['length']

            # recurrent ray tracing
            self.trace(newRay, depth + 1, newDistance)

    def getReturnEnergy(self, ray, depth, intersection):
        """Get return Energy.
		If the intersection point in FOV and the ray is not be blocked, 
		return a non-zero energy, otherwise return 0.

		Args:
			ray: incident ray
			depth: depth
			intersection: intersection

		Return:
			energy: 0 or energy received by sensor.
		"""

        # calculate the ray to lidar
        vectorToLidar = self.lidar.convergence - intersection['point']
        vectorToLidar = vectorToLidar / np.linalg.norm(vectorToLidar)

        # out of fov ?
        # both of the vector should be unit vector.
        if np.dot(-vectorToLidar, self.lidar.lookAt) < self.lidar.cosFov:
            # print('calculateReturnEnergy: out of fov')
            return 0

        vectorToTelescope = self.lidar.position - intersection['point']
        if np.arctan(self.lidar.r / np.linalg.norm(vectorToTelescope)) >= self.lidar.fov:
            # print('calculateReturnEnergy: smaller')
            return 0

        # intersect at the first time, sensor must can be seen.
        if depth == 0:
            energy = self.calculateReturnEnergy(ray, intersection)
            return energy

        # ray to lidar is blocked ?
        wiWorld = intersection['mtsIts'].shFrame.toWorld(intersection['mtsIts'].wi)
        wi = np.array([wiWorld[0], wiWorld[1], wiWorld[2], 0])
        if np.dot(wi, vectorToLidar) < 0:
            return 0

        rayToLidar = Ray()
        rayToLidar.setOrigin(Point(intersection['point'][0],
                                   intersection['point'][1], intersection['point'][2]))
        rayToLidar.setDirection(Vector(vectorToLidar[0],
                                       vectorToLidar[1], vectorToLidar[2]))
        intersectionReturn = self.scene.rayIntersect(rayToLidar)
        if intersectionReturn == None:
            energy = self.calculateReturnEnergy(ray, intersection)
            return energy

        return 0

    def calculateReturnEnergy(self, ray, intersection):
        """ Just calculate return energy without judge blocking

		Args: 
			ray: incident ray for its energy
			intersection: intersection information
		"""
        vectorToTelescope = self.lidar.position - intersection['point']
        distance2 = np.dot(vectorToTelescope, vectorToTelescope)
        apertureStereoAngle = self.lidar.aperture / distance2  # * (-np.dot(self.lidar.lookAt, vectorToTelescope / np.linalg.norm(vectorToTelescope)))
        wiWorld = intersection['mtsIts'].shFrame.toWorld(intersection['mtsIts'].wi)
        energy = apertureStereoAngle * intersection['reflectance'] / np.pi * ray.energy  # * dot(normalize(wiWorld), intersection['normal'])
        # print(wiWorld, intersection['normal'])
        return energy

    def receiveEnergy(self, intersection, deltaTime, energy, depth):
        x, y, z = intersection['point'][:3]
        record = [x, y, z, deltaTime, energy, depth]
        # self.records.append(record)
        self.recorder.commit(record)

    def getScatterRay(self, intersection, scatterType):
        """ Return scatter with origin and direction

		"""
        # print('tracer getScatterRay')
        # print('geo s', intersection['mtsIts'].geoFrame.s)
        # print('geo t', intersection['mtsIts'].geoFrame.t)
        # print('geo n', intersection['mtsIts'].geoFrame.n)
        # print('sh s', intersection['mtsIts'].shFrame.s)
        # print('sh t', intersection['mtsIts'].shFrame.t)
        # print('sh n', intersection['mtsIts'].shFrame.n)
        scatterRay = Ray()

        s = intersection['mtsIts'].shFrame.s
        t = intersection['mtsIts'].shFrame.t
        n = intersection['normal']

        # origin
        point = intersection['point']
        scatterRay.setOrigin(Point(point[0], point[1], point[2]))

        # dirction
        azimuth = 2 * np.pi * np.random.uniform()
        # zenith = 0.5 * np.pi * np.random.uniform()
        zenith = np.arccos(np.random.uniform())
        # zenith = np.pi * (2 * np.random.uniform() - 1)
        sgn = {'reflectance': 1, 'transmittance': -1}
        x = np.sin(zenith) * np.cos(azimuth)
        # y = np.sin(zenith) * np.sin(azimuth) * sgn[scatterType]
        y = np.sin(zenith) * np.sin(azimuth)
        z = np.cos(zenith) * sgn[scatterType]
        # z = np.cos(zenith)

        # u = int(np.random.uniform() * self.numberOfDirection)
        # x = self.directions[u][0]
        # y = self.directions[u][1]
        # z = self.directions[u][2] * sgn[scatterType]

        # square
        # x, y = self.squareToUniformDiskConcentric(np.random.uniform(), np.random.uniform())
        # z = np.sqrt(1 - x * x - y * y)
        # if (z < const.EPS):
        #     z = const.EPS
        # z *= sgn[scatterType]

        # is the normal incorrect ?
        wiWorld = intersection['mtsIts'].shFrame.toWorld(intersection['mtsIts'].wi)
        if dot(wiWorld, n) < 0:
            n = -Vector(n)

        # s = Vector(1, 0, 0)
        # t = normalize(cross(n, s))

        direction = s * x + t * y + n * z
        # direction = n

        # direction
        # direction = self.directions[int(np.random.uniform() * self.numberOfDirection)]
        # direction = Vector(direction[0], direction[2], direction[1])
        scatterRay.setDirection(direction)
        return scatterRay

    def intersect(self, ray):
        mtsIts = self.scene.rayIntersect(ray)
        if mtsIts == None:
            return None

        # length
        length = mtsIts.wi.length() * mtsIts.t

        # point
        point = np.array([mtsIts.p[0], mtsIts.p[1], mtsIts.p[2], 1])

        # normal
        mtsNormal = mtsIts.geoFrame.n
        # normal = np.array([mtsNormal[0], mtsNormal[1], mtsNormal[2], 0])
        wi = mtsIts.shFrame.toWorld(mtsIts.wi)
        # print('%f, %f, %f' % (wi[0], wi[1], wi[2]))
        if dot(mtsNormal, mtsIts.shFrame.toWorld(mtsIts.wi)) < 0:
            mtsNormal = -mtsNormal
        # print('mtsNormal', mtsNormal)

        # reflection, transmittance
        regex = r'\[\d+\.?\d*, \d+\.?\d*, \d+\.?\d*\]'
        bsdf = mtsIts.shape.getBSDF().__str__()

        # print(('tracer', bsdf))

        matched = map(eval, re.findall(regex, bsdf))

        # print(('tracer matched', matched))

        # reflectance, transmittance = map(lambda x: x[0], matched)
        reflectance = matched[0][0]
        transmittance = matched[0][1]

        # add into intersection
        intersection = {}
        intersection['length'] = length
        # intersection['point'] = point + np.array([mtsNormal[0] * const.EPS, mtsNormal[1] * const.EPS, mtsNormal[2] * const.EPS, 0])
        intersection['point'] = point
        # intersection['normal'] = normal
        intersection['normal'] = mtsNormal
        intersection['reflectance'] = reflectance
        intersection['transmittance'] = transmittance
        intersection['mtsIts'] = mtsIts

        return intersection

    def squareToUniformDiskConcentric(self, x, y):
        r1 = 2.0 * x - 1.0
        r2 = 2.0 * y - 1.0

        phi = 0
        r = 0
        if r1 != 0 or r2 != 0:
            # if r1 * r1 > r2 * r2:
            #     r, phi = r1, 0.25 * np.pi * r2 / r1
            # else:
            #     r, phi = r2, 0.5 * np.pi * r1 / r2
            r, phi = [(r2, 0.5 * np.pi - 0.25 * np.pi * r1 / r2), (r1, 0.25 * np.pi * r2 / r1)][r1 * r1 > r2 * r2]

        return r * np.cos(phi), r * np.sin(phi)
