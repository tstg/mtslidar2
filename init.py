MITSUBA_PATH = '../mtslidar1/Mitsuba 0.5.0/'

import os, sys
sys.path.append(MITSUBA_PATH + 'python/2.7')
os.environ['PATH'] = MITSUBA_PATH + os.pathsep + os.environ['PATH']
