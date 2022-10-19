#Inviwo Python script 
import inviwopy
import os
from pathlib import Path

app = inviwopy.app
network = app.network


filenameCurrent = network.VolumeSource.filename.value
p = Path(filenameCurrent)
print(filenameCurrent)

directory = p.parent.as_posix()
newDirectory = directory + '_subsampled_x_57-191/'

if not os.path.exists(newDirectory):
    os.mkdir(newDirectory)

for filename in os.listdir(directory):
    path = Path(filename)
    if (path.suffix == '.am' and 'OkuboWeiss' in filename):
        network.VolumeSource.filename.value = directory + '/' + filename
        network.VolumeSource.reload.press()
        network.VolumeExport.file.value = newDirectory + '/' + path.stem + '.dat'
        network.VolumeSource.Information.valueRange.value[1] = -10.7
        network.VolumeSource.Information.valueRange.value[1] = 40.93
        network.VolumeExport.export.press()
        print(filename)
