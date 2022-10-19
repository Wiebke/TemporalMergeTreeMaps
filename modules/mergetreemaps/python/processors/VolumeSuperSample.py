# Name: VolumeSuperSample 

import inviwopy as ivw
import numpy as np
from scipy.interpolate import interpn

class VolumeSuperSample(ivw.Processor):
    def __init__(self, id, name):
        ivw.Processor.__init__(self, id, name)
        self.inport = ivw.data.VolumeInport("inport")
        self.addInport(self.inport, owner=False)
        self.outport = ivw.data.VolumeOutport("outport")
        self.addOutport(self.outport, owner=False)

        self.volSize = ivw.properties.IntSize3Property("dims", "Output Dimensions", \
            size3_t(128,128,128), size3_t(1,1,1), size3_t(1024,1024,1024))
        self.addProperty(self.volSize, owner=False)

    @staticmethod
    def processorInfo():
        return ivw.ProcessorInfo(
    		classIdentifier = "org.inviwo.VolumeSuperSample", 
    		displayName = "Volume Super Sample",
    		category = "Python",
    		codeState = ivw.CodeState.Stable,
    		tags = ivw.Tags.PY
        )

    def getProcessorInfo(self):
        return VolumeSuperSample.processorInfo()

    def initializeResources(self):
        pass

    def process(self):
        if (not self.inport.hasData()): 
            return

        V = self.inport.getData()
        originalDims = V.dimensions
        x = np.linspace(0,1,originalDims[0])
        y = np.linspace(0,1,originalDims[1])
        z = np.linspace(0,1,originalDims[2])

        dims = self.volSize.value
        xi_ = np.linspace(0, 1, dims[0])
        yi_ = np.linspace(0, 1, dims[1])
        zi_ = np.linspace(0, 1, dims[2])

        inV = V.data
        points = np.stack(np.meshgrid(xi_, yi_, zi_, indexing='ij'), -1).reshape(-1, 3)
        data = interpn((x, y, z), inV, points).astype('float32')
        data = data.reshape((dims[0],dims[1],dims[2]))
        points = points.reshape((dims[0],dims[1],dims[2],3))
        
        # Inviwo seems to understand the data as column-major??
        data = data.flatten(order='F').reshape((dims[0],dims[1],dims[2]))
        volume = ivw.data.Volume(data)
        volume.dataMap.dataRange = V.dataMap.dataRange
        volume.dataMap.valueRange = V.dataMap.valueRange
        volume.worldMatrix = V.worldMatrix
        volume.modelMatrix = V.modelMatrix

        self.outport.setData(volume)
        print("???")
