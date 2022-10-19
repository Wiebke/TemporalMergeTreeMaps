#Inviwo Python script 
import inviwopy
from inviwopy.glm import size2_t,vec3
import os
import time

app = inviwopy.app
network = app.network

outputfolder = r'E:/Wiebke/DataSets/Duct/'
if not os.path.exists(outputfolder):
    os.mkdir(outputfolder)

filename = 'DuctPercolationScalar_'

for i in range(2):
    network.RawPercolationLoader.timeSlice = i+1
    outputfilename = os.path.join(outputfolder, filename + '%i'.zfill(3) + '.ivf')
    network.VolumeExport.file.value = outputfilename
    network.VolumeExport.export.press()