#Inviwo Python script 
import inviwopy
import os
import time

app = inviwopy.app
network = app.network

outputfolder = r"E:/Wiebke/MTMResults/1DExamples/"

app.resizePool(0)

lines = network.VolumeCombiner.description.value.split('\n')

for line in lines:
    idAndDecsription = line.split(': Generate')
    volumeId = (idAndDecsription[0])
    if (len(volumeId) > 1):
        name = idAndDecsription[1]
        #print(volumeId)
        name = name.replace(" ", "")
        name = name[0].lower() + name[1:]
        #print(name)
        network.VolumeCombiner.eqn.value = volumeId
        app.waitForPool()
        outputimg = os.path.join(outputfolder,name+ "_data.png")
        network.Canvas.snapshot(outputimg) 
        outputimg2 = os.path.join(outputfolder,name+ "_lvlsets.png")
        network.Canvas3.snapshot(outputimg2)
        outputimg3 = os.path.join(outputfolder,name+ "_1d_mtm.png")
        network.Canvas6.snapshot(outputimg3)
        outputimg4 = os.path.join(outputfolder,name+ "_1d_ntg.png")
        network.Canvas7.snapshot(outputimg4)
        outputcsv = os.path.join(outputfolder,name+ "_1d_mtm.csv")
        network.DataFrameExporter.exportFile.value = outputcsv
        network.DataFrameExporter.snapshot.press()
        outputcsv2 = os.path.join(outputfolder,name+ "_mergetree.csv")
        network.DataFrameExporter2.exportFile.value = outputcsv2
        network.DataFrameExporter2.snapshot.press()
    


#help(inviwopy)