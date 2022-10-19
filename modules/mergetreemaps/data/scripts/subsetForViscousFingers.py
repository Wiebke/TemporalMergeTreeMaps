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
newDirectory = directory + '_subsampled_z_0-50/'

if not os.path.exists(newDirectory):
    os.mkdir(newDirectory)

for filename in os.listdir(directory):
    path = Path(filename)
    if (path.suffix == '.dat' and 'density' in filename):
        network.VolumeSource.filename.value = directory + '/' + filename
        network.VolumeSource.reload.press()
        network.VolumeExport.file.value = newDirectory + '/' + path.name
        network.VolumeSource.Information.valueRange.value[1] = 135.613
        network.VolumeExport.export.press()
        print(filename)
