import numpy as np

class Spherical(object):
    def __init__(self, radial = 1.0, polar = 0.0, azimuthal = 0.0):
        self.radial = radial
        self.polar = polar
        self.azimuthal = azimuthal

    def toCartesian(self):
        r = np.sin(self.azimuthal) * self.radial
        x = np.cos(self.polar) * r
        y = np.sin(self.polar) * r
        z = np.cos(self.azimuthal) * self.radial
        return x, y, z


def generatePatchesFromFile(filename):
    d = np.loadtxt(filename)
    zeniths = np.deg2rad(d[:, 0])
    azimuths = np.deg2rad(d[:, 1])
    s = Spherical()
    for zenith, azimuth in zip(zeniths, azimuths):
        s.polar = azimuth
        s.azimuthal = zenith
        # print('%f, %f, %f;' % s.toCartesian())
        yield s.toCartesian()

def generatePatches(limit):
    """
    n = floor(sqrt(limit/2))
    actual number of direction = 1 + 2 * n * (n + 1)
    """
    s = Spherical()

    # n = int(np.ceil(np.sqrt((limit - 2) / 4)))
    n = int(np.sqrt(limit / 2))
    # azimuthal = 0.5 * np.pi / n
    azimuthal = 0.5 * np.pi / (n - 1)

    # for a in range(-n, n + 1):
    for a in range(n):
        s.polar = 0
        # size = (n - abs(a)) * 4 or 1
        size = 4 * n or 1
        polar = 2 * np.pi / size
        for i in range(size):
            yield s.toCartesian()
            s.polar += polar
        s.azimuthal += azimuthal
