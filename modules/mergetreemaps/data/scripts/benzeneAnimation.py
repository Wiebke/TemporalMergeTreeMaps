#Inviwo Python script 
import inviwopy
from inviwopy.glm import size2_t,vec3
import os
import time

app = inviwopy.app
network = app.network

outputfolder = r"E:\Wiebke\MTMResults\Benzene\animation_video"

if not os.path.exists(outputfolder):
    os.mkdir(outputfolder)

app.resizePool(0)

for i in range(0,21,1): 
    network.VolumeSliceViewer2.sliceZ.value=(i+1)
    app.waitForPool()    
    outputimg = os.path.join(outputfolder,"benzene_t"+ ("%i"%i).zfill(2) + "_noOverlay.png")
    network.Canvas2.snapshot(outputimg) 

app.resizePool(18)

#help(inviwopy)