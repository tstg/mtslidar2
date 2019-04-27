import os
import numpy as np

import const

INDEX_OF_TIME = 3
INDEX_OF_ENERGY = 4


class Recorder:

    def __init__(self):
        self.filename = 'records.txt'
        self.records = []

    def commit(self, record):
        self.records.append(record)

    def save(self):
        records = np.array(self.records)
        np.savetxt(self.filename, records)

    def convolve(self, filename, out, altitude=10000, dt_mes=1e-9, height=8):
        """ get the convolved waveform, return the result in an array, and save it in a file.

            :param filename: File created by Tracer
            :param out: the name of the output file
            :param altitude: lidar's altitude [m]
            :param dt_mes: Delta time of measurement, according the acquiring rate of the lidar [s]
            :param height: result above and under the ground [m]
            :return:
            """

        # b = const.LIGHT_SPEED * dt_mes  # width of bin in terms of length [m]
        # num_bin = int(np.ceil(height / b) * 2)
        # bins = np.linspace(altitude * 2 - b * (num_bin // 2), altitude * 2 + b * (num_bin // 2), num_bin, endpoint=False)
        bins = []
        bins.append(0)
        time = dt_mes * 1e9
        while time * const.LIGHT_SPEED / 1e9 <= height:
            bins.append(time)
            bins.append(-time)
            time += dt_mes * 1e9
        bins.sort()
        num_bin = len(bins)
        bins = np.array(bins)
        times = np.array(bins)
        bins = bins + 2 * altitude / const.LIGHT_SPEED * 1e9  # - dt_mes * 1e9 / 2
        print('convolveUsingTime')
        # print(bins)

        f = np.zeros(num_bin)  # contain the accumulated energy input

        # read file
        data = np.array(self.records)
        if len(data.shape) == 1:
            data = np.array([data])

        length = data[:, INDEX_OF_TIME] / const.LIGHT_SPEED * 1e9
        energy = data[:, INDEX_OF_ENERGY]
        # print('convolveUsingTime')
        # print(length)

        d = np.array([length, energy]).T

        # accumulate the energy, assign to variable `f`
        l = 2 * (altitude - height) / const.LIGHT_SPEED * 1e9
        r = 2 * (altitude + height) / const.LIGHT_SPEED * 1e9
        for i in d:
            if not l <= i[0] <= r:
                continue
            # idx = np.searchsorted(bins, i[0]) - 1  # lower bound better
            # idx, ansr = searchRange(bins, i[0])
            idx = self.lower_bound(bins, i[0]) - 1
            if idx != 16:
                print(idx)
            f[idx] += i[1]

        # convolve
        n = 5  # hard code
        x = np.linspace(-3, 3, 2 * n + 1)
        pulse = np.exp(-x * x / 2)
        pulse = pulse / np.sum(pulse)

        res = np.convolve(f, pulse, 'same')
        # ans = np.array([bins - 2 * altitude / const.LIGHT_SPEED * 1e9 + 0.5, res]).T
        ans = np.array([times, res]).T

        [dirname, name] = os.path.split(out)

        # np.savetxt(dirname + '/accum/accum-' + name, np.array([bins - 2 * altitude / const.LIGHT_SPEED * 1e9 + 0.5, f]).T)
        np.savetxt(dirname + '/accum/accum-' + name, np.array([times, f]).T)
        np.savetxt(out, ans)
        return ans

    def lower_bound(self, nums, target):
        low, high = 0, len(nums) - 1
        pos = len(nums)
        while low < high:
            mid = (low + high) / 2
            if nums[mid] < target:
                low = mid + 1
            else:  # >=
                high = mid
                # pos = high
        if nums[low] >= target:
            pos = low
        return pos
