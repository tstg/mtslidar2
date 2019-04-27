import os

import numpy as np

import const

INDEX_OF_TIME = 3
INDEX_OF_ENERGY = 4
INDEX_OF_DEPTH = 5
C = const.LIGHT_SPEED

def convolve(filename, out, altitude=10000, dt_mes=1e-9, height=6):
    """ get the convolved waveform, return the result in an array, and save it in a file.

    :param filename: File created by Tracer
    :param out: the name of the output file
    :param altitude: lidar's altitude [m]
    :param dt_mes: Delta time of measurement, according the acquiring rate of the lidar [s]
    :param height: result above and under the ground [m]
    :return:
    """

    b = const.LIGHT_SPEED * dt_mes  # width of bin in terms of length [m]
    num_bin = int(np.ceil(height / b) * 2)
    bins = np.linspace(altitude * 2 - b * (num_bin // 2), altitude * 2 + b * (num_bin // 2), num_bin, endpoint=False)

    f = np.zeros(num_bin)  # contain the accumulated energy input

    # read file
    data = np.loadtxt(filename)
    if len(data.shape) == 1:
        data = np.array([data])

    length = data[:, INDEX_OF_TIME]
    energy = data[:, INDEX_OF_ENERGY]

    d = np.array([length * const.LIGHT_SPEED, energy]).T

    # accumulate the energy, assign to variable `f`
    for i in d:
        if not altitude * 2 - height <= i[0] <= altitude * 2 + height:
            continue
        idx = np.searchsorted(bins, i[0]) - 1  # lower bound better
        f[idx] += i[1]

    # convolve
    n = 5  # hard code
    x = np.linspace(-3, 3, 2 * n + 1)
    pulse = np.exp(-x * x / 2)
    pulse = pulse / np.sum(pulse)

    res = np.convolve(f, pulse, 'same')
    ans = np.array([(bins - altitude * 2) / const.LIGHT_SPEED, res]).T

    [dirname, name] = os.path.split(out)

    np.savetxt(dirname + '/accum/accum-' + name, np.array([(bins - altitude * 2) / const.LIGHT_SPEED, f]).T)
    np.savetxt(out, ans)
    return ans

def convolveUsingDistance(filename, out, altitude=10000, dt_mes=1e-9, height=6):
    """ get the convolved waveform, return the result in an array, and save it in a file.

    :param filename: File created by Tracer
    :param out: the name of the output file
    :param altitude: lidar's altitude [m]
    :param dt_mes: Delta time of measurement, according the acquiring rate of the lidar [s]
    :param height: result above and under the ground [m]
    :return:
    """

    bins = [0]
    time = dt_mes * 1e9
    while time * const.LIGHT_SPEED / 1e9 <= height:
        bins.append(time)
        bins.append(-time)
        time += dt_mes * 1e9
    bins.sort()
    num_bin = len(bins)
    bins = np.array(bins)
    times = np.array(bins)
    bins = bins * const.LIGHT_SPEED / 1e9

    f = np.zeros(num_bin)  # contain the accumulated energy input

    # read file
    data = np.loadtxt(filename)
    if len(data.shape) == 1:
        data = np.array([data])

    length = data[:, INDEX_OF_TIME] - 2 * altitude
    energy = data[:, INDEX_OF_ENERGY]

    # d = np.array([length * const.LIGHT_SPEED, energy]).T
    d = np.array([length, energy]).T

    # accumulate the energy, assign to variable `f`
    for i in d:
        if not - height <= i[0] <= height:
            continue
        idx = np.searchsorted(bins, i[0]) - 1  # lower bound better
        # idx = lower_bound(bins, i[0]) - 1
        f[idx] += i[1]

    # convolve
    # n = 5  # hard code
    # x = np.linspace(-3, 3, 2 * n + 1)
    # pulse = np.exp(-x * x / 2)
    numberOfSigma = 3
    relativePower = 0.5
    durationAtRelativePower = 2.0  # half pulse duration at relative power [ns]
    sigmaPulse = durationAtRelativePower / np.sqrt(2.0) / np.sqrt(-np.log(relativePower))
    print('sigmaPulse = %f' % sigmaPulse)
    n = int(np.round((2 * numberOfSigma * sigmaPulse / (dt_mes * 1e9) - 1) / 2))
    print('n = %d' % n)
    x = np.linspace(-numberOfSigma * sigmaPulse, numberOfSigma * sigmaPulse, 2 * n + 1)
    pulse = np.exp(-0.5 * x * x / (sigmaPulse * sigmaPulse))
    pulse = pulse / np.sum(pulse)

    # pulse = np.array([0.029411760000000, 0.235294100000000, 0.470588200000000, 0.235294100000000, 0.029411760000000])

    pulse = np.array([0.003088973, 0.01469372, 0.04942357, 0.1175497, 0.1976943, 0.2350995, 0.1976943, 0.1175497, 0.04942357, 0.01469372, 0.003088973])

    res = np.convolve(f, pulse, 'same')
    # res = np.correlate(f, pulse, 'same')
    ans = np.array([times, res]).T

    [dirname, name] = os.path.split(out)

    np.savetxt(dirname + '/accum/accum-' + name, np.array([times, f]).T)
    np.savetxt(out, ans)
    return ans


def convolveUsingTime(filename, out, altitude=10000, dt_mes=1e-9, height=8, firstOrder=False):
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
    while time * C / 1e9 <= height:
        bins.append(time)
        bins.append(-time)
        time += dt_mes * 1e9
    bins.sort()
    num_bin = len(bins)
    bins = np.array(bins)
    times = np.array(bins)
    bins = bins + 2 * altitude / C * 1e9  # - altitude * 1e9 / const.LIGHT_SPEED % (dt_mes * 1e9 / 2)
    print('convolveUsingTime')
    # print(bins)

    f = np.zeros(num_bin)  # contain the accumulated energy input

    # read file
    data = np.loadtxt(filename)
    if len(data.shape) == 1:
        data = np.array([data])

    length = data[:, INDEX_OF_TIME] / C * 1e9
    energy = data[:, INDEX_OF_ENERGY]
    if firstOrder:
        length = length[data[:, INDEX_OF_DEPTH] == 0]
        energy = energy[data[:, INDEX_OF_DEPTH] == 0]
        print('convolveUsingTime 1st energy', energy.sum())
    # print('convolveUsingTime')
    # print(length)

    d = np.array([length, energy]).T

    # accumulate the energy, assign to variable `f`
    l = 2 * (altitude - height) / C * 1e9
    r = 2 * (altitude + height) / C * 1e9
    for i in d:
        if not l <= i[0] <= r:
            continue
        # idx = np.searchsorted(bins, i[0]) - 1  # lower bound better
        # idx, ansr = searchRange(bins, i[0])
        idx = lower_bound(bins, i[0]) - 1
        f[idx] += i[1]

    # convolve
    n = 5  # hard code
    x = np.linspace(-3, 3, 2 * n + 1)
    pulse = np.exp(-x * x / 2)
    pulse = pulse / np.sum(pulse)

    pulse = np.array([0.003088973, 0.01469372, 0.04942357, 0.1175497, 0.1976943, 0.2350995, 0.1976943, 0.1175497, 0.04942357, 0.01469372, 0.003088973])

    res = np.convolve(f, pulse, 'same')
    # ans = np.array([bins - 2 * altitude / const.LIGHT_SPEED * 1e9 + 0.5, res]).T
    ans = np.array([times, res]).T

    [dirname, name] = os.path.split(out)

    np.savetxt(dirname + '/accum/accum-' + name, np.array([bins - 2 * altitude / const.LIGHT_SPEED * 1e9 + 0.5, f]).T)
    np.savetxt(dirname + '/accum/accum-' + name, np.array([times, f]).T)
    np.savetxt(out, ans)
    return ans

def searchRange(nums, target):
        """
        :type nums: List[int]
        :type target: int
        :rtype: List[int]
        """
        Len = len(nums)
        if Len == 0: return [-1, -1]
        l = 0
        r = Len - 1
        while l < r:
            m = (l + r) / 2
            if nums[m] < target:
                l = m + 1
            else:
                r = m
        if nums[r] != target:
            return [-1, -1]
        ansl = r
        ansr = r
        while r < Len:
            if nums[r] == target:
                ansr = r
                r += 1
            else:
                break
        return [ansl, ansr]

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