#Inviwo Python script 
import inviwopy
from inviwopy.glm import size2_t,vec3
import os
import time

app = inviwopy.app
network = app.network

outputfolder = r"E:\Wiebke\MTMResults\Tangaroa\volRendering"

if not os.path.exists(outputfolder):
    os.mkdir(outputfolder)

app.resizePool(0)

for i in range(0,5,1): 
    network.VolumeSequenceElementSelector.timeStep.selectedSequenceIndex.value=(i+1) 
    app.waitForPool()    
    outputimg = os.path.join(outputfolder,"tangaroa_t"+ ("%i"%i).zfill(3) + ".png")
    network.Canvas2.snapshot(outputimg) 

app.resizePool(18)

#help(inviwopy)