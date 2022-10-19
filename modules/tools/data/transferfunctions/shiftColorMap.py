#Inviwo Python script 
import inviwopy


app = inviwopy.app
network = app.network

tf = inviwopy.app.network.VolumeSliceViewer.tfGroup.transferFunction

colors = tf.getValues()
tf.clear()

shiftBy = -1

# Values stay the same but colors get shifted
for i in range(len(colors)):
    shiftedI = (i + shiftBy)%len(colors)
    pos = colors[i].pos
    color = colors[shiftedI].color
    tf.add(pos, color)
