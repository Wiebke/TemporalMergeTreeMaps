# Name: MergeTreeExample 

import inviwopy as ivw

class MergeTreeExample(ivw.Processor):
    def __init__(self, id, name):
        ivw.Processor.__init__(self, id, name)
        self.outport = ivw.data.VolumeOutport("outport")
        self.addOutport(self.outport, owner=False)

    @staticmethod
    def processorInfo():
        return ivw.ProcessorInfo(
    		classIdentifier = "org.inviwo.MergeTreeExample", 
    		displayName = "MergeTreeExample",
    		category = "Python",
    		codeState = ivw.CodeState.Stable,
    		tags = ivw.Tags.PY
        )

    def getProcessorInfo(self):
        return MergeTreeExample.processorInfo()

    def initializeResources(self):
        pass

    def process(self):
        size_x = 5
        size_y = 5

        #difference in rows -> difference in y

        data = np.zeros((size_x, size_y), dtype='float32')
        # First row from bottom
        data[0][0] = 7
        data[0][1] = 24
        data[0][2] = 4
        data[0][3] = 0
        data[0][4] = 1

        # Second row from bottom
        data[1][0] = 10
        data[1][1] = 23
        data[1][2] = 22
        data[1][3] = 2
        data[1][4] = 17

        # Third row from bottom
        data[2][0] = 11
        data[2][1] = 16
        data[2][2] = 20
        data[2][3] = 12
        data[2][4] = 14

        # Fourth row from bottom
        data[3][0] = 15
        data[3][1] = 19
        data[3][2] = 6
        data[3][3] = 8
        data[3][4] = 9

        # Fifth row from bottom
        data[4][0] = 21
        data[4][1] = 13
        data[4][2] = 3
        data[4][3] = 5
        data[4][4] = 18

        print(data)

        minValue = np.min(data)
        maxValue = np.max(data)
        # Inviwo seems to understand the data as column-major??
        data = data.flatten(order='F').reshape((size_x,size_y, 1)).astype('float32')
        volume = ivw.data.Volume(data)
        volume.dataMap.dataRange = dvec2(minValue, maxValue)
        volume.dataMap.valueRange = dvec2(minValue, maxValue)
        volume.interpolation = ivw.data.InterpolationType.Nearest

        self.outport.setData(volume)
