#Inviwo Python script 
import inviwopy
from inviwopy.glm import size2_t,vec3
import os
import time

app = inviwopy.app
network = app.network

outputfolder = r"E:\Wiebke\MTMResults\Cylinder\volRendering_new2"

if not os.path.exists(outputfolder):
    os.mkdir(outputfolder)

app.resizePool(0)

for i in range(0,508,1): 
    network.VolumeSeries2.fileIndex.value=i
    app.waitForPool()    
    outputimg = os.path.join(outputfolder,"cylinder_t"+ ("%i"%i).zfill(3) + ".png")
    network.Canvas.snapshot(outputimg) 

app.resizePool(18)

#help(inviwopy)