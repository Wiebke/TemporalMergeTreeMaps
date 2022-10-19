# Name: CarrDataSetExample 

import inviwopy as ivw

class CarrDataSetExample(ivw.Processor):
    def __init__(self, id, name):
        ivw.Processor.__init__(self, id, name)
        self.outport = ivw.data.VolumeOutport("outport")
        self.addOutport(self.outport, owner=False)

    @staticmethod
    def processorInfo():
        return ivw.ProcessorInfo(
    		classIdentifier = "org.inviwo.CarrDataSetExample", 
    		displayName = "Carr Data Set Example",
    		category = "Python",
    		codeState = ivw.CodeState.Stable,
    		tags = ivw.Tags.PY
        )

    def getProcessorInfo(self):
        return CarrDataSetExample.processorInfo()

    def initializeResources(self):
        pass

    def process(self):
        size_x = 8
        size_y = 9

        #difference in rows -> difference in y

        data = np.zeros((size_x, size_y), dtype='float32')
        # First row from bottom
        data[0][0] = 12
        data[0][1] = 11
        data[0][2] = 10
        data[0][3] = 9
        data[0][4] = 8
        data[0][5] = 7
        data[0][6] = 6
        data[0][7] = 5
        data[0][8] = 0

        # Second row from bottom
        data[1][0] = 13
        data[1][1] = 42
        data[1][2] = 43
        data[1][3] = 44
        data[1][4] = 34
        data[1][5] = 33
        data[1][6] = 31
        data[1][7] = 30
        data[1][8] = 14

        # Third row from bottom
        data[2][0] = 17
        data[2][1] = 41
        data[2][2] = 77
        data[2][3] = 45
        data[2][4] = 35
        data[2][5] = 20
        data[2][6] = 21
        data[2][7] = 32
        data[2][8] = 15

        # Fourth row from bottom
        data[3][0] = 19
        data[3][1] = 76
        data[3][2] = 80
        data[3][3] = 78
        data[3][4] = 46
        data[3][5] = 68
        data[3][6] = 67
        data[3][7] = 40
        data[3][8] = 16

        # Fifth row from bottom
        data[4][0] = 23
        data[4][1] = 75
        data[4][2] = 79
        data[4][3] = 48
        data[4][4] = 69
        data[4][5] = 87
        data[4][6] = 88
        data[4][7] = 81
        data[4][8] = 18

        # Sixth row from bottom
        data[5][0] = 25
        data[5][1] = 47
        data[5][2] = 50
        data[5][3] = 73
        data[5][4] = 86
        data[5][5] = 90
        data[5][6] = 71
        data[5][7] = 82
        data[5][8] = 22

        # Seventh row from bottom
        data[6][0] = 27
        data[6][1] = 100
        data[6][2] = 49
        data[6][3] = 72
        data[6][4] = 85
        data[6][5] = 89
        data[6][6] = 83
        data[6][7] = 28
        data[6][8] = 24

        # Eigth row from bottom
        data[7][0] = 29
        data[7][1] = 37
        data[7][2] = 39
        data[7][3] = 70
        data[7][4] = 74
        data[7][5] = 84
        data[7][6] = 38
        data[7][7] = 36
        data[7][8] = 26

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
