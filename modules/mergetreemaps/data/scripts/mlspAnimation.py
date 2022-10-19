#Inviwo Python script 
import inviwopy
from inviwopy.glm import size2_t,vec3
import os
import time

app = inviwopy.app
network = app.network

outputfolder = r"E:/Wiebke/MTMResults/Animation/"

app.resizePool(0)

for i in range(1,40,1):
    network.ContourTreeSequenceElementSelector.timeStep.selectedSequenceIndex.value=(i+1)
    app.waitForPool()    
    outputimg = os.path.join(outputfolder,"mlsp_landscape_"+ ("%i"%i).zfill(3) + ".png")
    network.Canvas3.snapshot(outputimg) 

app.resizePool(16)

#help(inviwopy)