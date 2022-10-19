#Inviwo Python script 
import inviwopy
from inviwopy.glm import size2_t,vec3
import os
import time

app = inviwopy.app
network = app.network

inputfolder = r'E:/Data/Tangaroa/'
outputfolder = r'E:/Data/Tangaroa/Magnitude'
if not os.path.exists(outputfolder):
    os.mkdir(outputfolder)

outputfolderOkuboWeiss = r'E:/Data/Tangaroa/OkuboWeiss'
if not os.path.exists(outputfolderOkuboWeiss):
    os.mkdir(outputfolderOkuboWeiss)

app.resizePool(0)   

filename = 'tangaroa-'

for i in range(0,201):
    timestep = i*0.01
    inputfile = os.path.join(inputfolder, filename + '%.2f.am'%timestep)
    print(inputfile)
    network.VolumeSource.filename.value = inputfile

    app.waitForPool()   
    #outputfilename = os.path.join(outputfolder, filename + '%.2f'%timestep + '_magnitude.dat')
    #network.VolumeExport.file.value = outputfilename
    #network.VolumeExport.export.press()
    outputfilename = os.path.join(outputfolderOkuboWeiss, filename + '%.2f'%timestep + '_okuboweiss.dat')
    network.VolumeExport2.file.value = outputfilename
    network.VolumeExport2.export.press()



app.resizePool(18)