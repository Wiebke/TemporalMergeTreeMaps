#Inviwo Python script 
import inviwopy
from inviwopy.glm import size2_t,vec3
import os
import time

app = inviwopy.app
network = app.network

outputfolder = r"E:\Wiebke\MTMResults\EuropeanWinterstormsDec1999\anomaly_animation_all"

if not os.path.exists(outputfolder):
    os.mkdir(outputfolder)

app.resizePool(0)

for i in range(0,744,1): 
    network.IndexToTimeStampText.timeStep.value=(i+1)
    app.waitForPool()    
    timestamp = network.IndexToTimeStampText.timeStepText.value
    timestamp = timestamp.replace(':','-')
    outputimg = os.path.join(outputfolder,"mslp_anomaly_"+ ("%i"%i).zfill(3) + "_" + timestamp + ".png")
    network.Canvas4.snapshot(outputimg)

app.resizePool(18)

#help(inviwopy)