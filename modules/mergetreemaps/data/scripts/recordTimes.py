#Inviwo Python script 
import inviwopy
import pandas as pd
import os

app = inviwopy.app
network = app.network

processorsTopology = [network.PersistenceDiagramsForSequence, \
    network.TopologicalSimplificationForSequence, \
    network.ContourTreesForSequence]

processorsUs = [network.OptimizeMergeTreeMapGreedy ,\
    network.ContourTreeSequenceToLandscape2 ,\
    network.MergeTreeMap2]

columns = []   
values = []

totalT = 0.0
totalTopology = 0.0
for p in processorsTopology:
    t = p.timer.value
    name = p.displayName
    print(t, name)
    columns.append(name)
    values.append(t)
    totalT = totalT + t
    totalTopology = totalTopology + t
columns.append('Topology Total')
values.append(totalTopology)

totalUs = 0.0
for p in processorsUs:
    t = p.timer.value
    name = p.displayName
    print(t, name)
    columns.append(name)
    values.append(t)
    totalT = totalT + t
    totalUs = totalUs + t
columns.append('Ours Total')
values.append(totalUs)

columns.append('Overall Total')
values.append(totalT)

columns.append('Persistence')
persistence = network.TopologicalSimplificationForSequence.threshold.value
values.append(persistence)

df = pd.DataFrame(columns=columns)
df.loc[0] = values
#print(df)

timings_file = r'E:\Wiebke\MTMResults\Tangaroa\tangaroa_persistence_timings18procs.csv'

# if file does not exist create it and save new DataFrame
if not os.path.exists(timings_file):
    print("File does not exist. Recording first sample.")
    df.to_csv(timings_file, index=False)
# load the existing file and concatenate
else:
    dfOld = pd.read_csv(timings_file)
    concatDf = pd.concat([df, dfOld])
    concatDf.to_csv(timings_file, index=False)
    numSamples = concatDf.shape[0]
    print("Recorded %i samples so far." %numSamples)
    # If we have 10 sampled compute mean
    if (numSamples >= 10):
        mean_timings_file = timings_file.replace('.csv', '_mean.csv')
        df_mean = concatDf.mean()
        print(df_mean)
        df_mean.to_csv(mean_timings_file, index=True)
    