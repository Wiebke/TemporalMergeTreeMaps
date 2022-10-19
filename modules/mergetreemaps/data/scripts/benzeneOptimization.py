#Inviwo Python script 
import inviwopy
from inviwopy.glm import size2_t,vec3
import os
import time
import pandas as pd

app = inviwopy.app
network = app.network

outputfolder = r"E:\Wiebke\MTMResults\Benzene\benzene_optimization"

if not os.path.exists(outputfolder):
    os.mkdir(outputfolder)

app.resizePool(0)

timesteps = network.OptimizeMergeTreeMapGreedy.timesteps.value
outputcsv = os.path.join(outputfolder, "benzene_objectives_random.csv")
numseeds = 100

network.OptimizeMergeTreeMapGreedy.useCounts.value=1
network.OptimizeMergeTreeMapGreedy.randomizeOrders.value=1
network.OptimizeMergeTreeMapGreedy.normalizeObj.value=0

randomizeFully = True
network.OptimizeMergeTreeMapGreedy.showcaseBadChoices.randomizeFully.value=randomizeFully
chooseWorse = True
network.OptimizeMergeTreeMapGreedy.showcaseBadChoices.chooseWorse.value=chooseWorse

for i in range(0,timesteps):
    network.OptimizeMergeTreeMapGreedy.focustimestep.value=i
    for seed in range(0,numseeds):
        network.OptimizeMergeTreeMapGreedy.colorSeed.value=seed
        app.waitForPool()
        # Record result
        objectiveValue = network.OptimizeMergeTreeMapGreedy.objective.value
        if (randomizeFully):
            objectiveValue = network.EvaluateMergeTreeMap.evalValue.value
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

app.resizePool(18)

#help(inviwopy)