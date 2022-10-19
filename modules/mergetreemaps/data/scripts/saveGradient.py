#Inviwo Python script 
import inviwopy
import os
import time
from pathlib import Path

app = inviwopy.app
network = app.network

outputfolder = r"E:/Wiebke/DataSets/MeanSeaLevelPressure/gradient"

timesteps = 744

volumefile = network.VolumeSource2.filename.value
path = Path(volumefile)

if not os.path.exists(outputfolder):
    os.mkdir(outputfolder)

for i in range(0,timesteps,1):
    network.VolumeSequenceElementSelector.timeStep.selectedSequenceIndex.value=(i+1)
    app.waitForPool()    
    outputfile = os.path.join(outputfolder, path.stem+"_gradient_magnitude"+ ("%i"%i).zfill(3) + ".dat")
    network.VolumeExport.file.value = outputfile
    network.VolumeExport.export.press()

#help(inviwopy)