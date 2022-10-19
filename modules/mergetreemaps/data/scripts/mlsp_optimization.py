#Inviwo Python script 
import inviwopy
from inviwopy.glm import size2_t,vec3
import os
import time
import pandas as pd

app = inviwopy.app
network = app.network

outputfolder = r"E:\Wiebke\MTMResults\EuropeanWinterstormsDec1999\optimization"

if not os.path.exists(outputfolder):
    os.mkdir(outputfolder)

timesteps = network.OptimizeMergeTreeMapGreedy2.timesteps.value
outputcsv = os.path.join(outputfolder, "mlsp_anomaly_objectives_10.csv")
numseeds = 2

network.OptimizeMergeTreeMapGreedy2.useCounts.value=1
network.OptimizeMergeTreeMapGreedy2.randomizeOrders.value=1

for i in range(0,2):
    network.OptimizeMergeTreeMapGreedy2.focustimestep.value=i
    for seed in range(0,numseeds):
        network.OptimizeMergeTreeMapGreedy2.colorSeed.value=seed
        app.waitForPool()
        # Record result
        objectiveValue = network.OptimizeMergeTreeMapGreedy2.objective.value
        df = pd.DataFrame(columns=["Objective", "Focus Timestep", "Seed"])
        df.loc[0] = [objectiveValue, i, seed]
        if not os.path.exists(outputcsv):
            print("File does not exist. Recording first sample.")
            df.to_csv(outputcsv, index=False)
        # load the existing file and concatenate
        else:
            dfOld = pd.read_csv(outputcsv)
            concatDf = pd.concat([dfOld, df])
            concatDf.to_csv(outputcsv, index=False)

#help(inviwopy)