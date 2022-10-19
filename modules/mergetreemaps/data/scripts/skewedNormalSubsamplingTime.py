#Inviwo Python script 
import inviwopy
import os
import time

app = inviwopy.app
network = app.network

outputfolder = r"E:/Wiebke/MTMResults/SubsamplingTime/"

app.resizePool(0)

timesteps=network.GenerateMovingSkewedNormals.timesteps.value

for i in range(16,timesteps,16):
    network.GenerateMovingSkewedNormals.features.skewedNormal2.timestepEnd = i/2-1
    network.GenerateMovingSkewedNormals.features.skewedNormal3.timestepBegin = i/2
    network.GenerateMovingSkewedNormals.features.skewedNormal3.timestepEnd = i-1
    network.GenerateMovingSkewedNormals.timesteps.value = i
    app.waitForPool()
    
    outputimg = os.path.join(outputfolder,"appear_disappear_"+ ("%i"%i).zfill(3) + ".png")
    network.Canvas8.snapshot(outputimg) 

#help(inviwopy)