#Inviwo Python script 
import inviwopy
from inviwopy.glm import size2_t,vec3
import os
import time
import pandas as pd

app = inviwopy.app
network = app.network

outputfolder = r"E:\Wiebke\MTMResults\Cylinder\cylinder_subset_x_57-191_tmtm"

if not os.path.exists(outputfolder):
    os.mkdir(outputfolder)

app.resizePool(0)

timesteps = network.OptimizeMergeTreeMapGreedy.timesteps.value
outputcsv = os.path.join(outputfolder, "cylinder_objectives.csv")

for i in range(0,timesteps,50):
    for c in [True, False]:
        network.OptimizeMergeTreeMapGreedy.useCounts.value=c
        network.OptimizeMergeTreeMapGreedy.focustimestep.value=i
        app.waitForPool()
        filePrefix = ("cylinder__CountsOn_%i")%(1 if c else 0) + "_focustime_"+ ("%i"%i).zfill(3)
        outputimg = os.path.join(outputfolder, filePrefix+".png")
        network.ImageExport.file.value = outputimg
        network.ImageExport.export.press()
        outputimgObj = os.path.join(outputfolder, filePrefix+"_objective.png")
        network.ImageExport2.file.value = outputimgObj
        network.ImageExport2.export.press()
        objectiveValue = network.OptimizeMergeTreeMapGreedy.objective.value
        df = pd.DataFrame(columns=["Objective", "Focus Timestep", "Counts On"])
        df.loc[0] = [objectiveValue, i, c]
        if not os.path.exists(outputcsv):
            print("File does not exist. Recording first sample.")
            df.to_csv(outputcsv, index=False)
        # load the existing file and concatenate
        else:
            dfOld = pd.read_csv(outputcsv)
            concatDf = pd.concat([dfOld, df])
            concatDf.to_csv(outputcsv, index=False)

app.resizePool(18)

#help(inviwopy)