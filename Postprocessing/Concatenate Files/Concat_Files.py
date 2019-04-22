import os
import glob
import numpy as np
import fnmatch

callFolder = 'Raw_Data/'

nrFiles = len(fnmatch.filter(os.listdir(callFolder), '*.BIN'))

for i in range(0, nrFiles):
    for f in glob.glob(os.path.join(callFolder, 'ROV' + str(i+1) + '.BIN')):
        os.system('cat '+f+' >> Rover.bin')
