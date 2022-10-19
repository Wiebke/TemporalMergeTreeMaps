# Name: SpreadingRingGeneration 

import inviwopy as ivw

class spreading_ring:
    
    def __init__(self, epicenter, peak_0, peak_a, mu_0, mu_a, omega_0, omega_a):
        self.epicenter = epicenter
        self.peak_0 = peak_0
        self.peak_a = peak_a
        self.mu_0 = mu_0
        self.mu_a = mu_a
        self.omega_0 = omega_0
        self.omega_a = omega_a

    def evaluate(self, x, t):
        p_t = self.peak_0 + t*self.peak_a
        mu_t = self.mu_0 + t*self.mu_a
        omega_t = self.omega_0 + t*self.omega_a
        # distance to epicenter
        dist = np.sqrt(np.sum((x-self.epicenter) ** 2, axis=2))
        # use dist as x sort of for a Gaussian
        inner = (dist - mu_t)/omega_t
        ring = p_t * np.exp(-0.5 * inner**2)
        return ring


class SpreadingRingGeneration(ivw.Processor):
    def __init__(self, id, name):
        ivw.Processor.__init__(self, id, name)
        self.outport = ivw.data.VolumeOutport("outport")
        self.addOutport(self.outport, owner=False)
        # Dimensions and size (original 14x14 with each cell 15x15)
        self.volSize = ivw.properties.IntSize2Property("dims", "Dimensions", size2_t(14,14), size2_t(1,1), size2_t(512,512))
        self.addProperty(self.volSize, owner=False)
        self.rangeX = ivw.properties.FloatMinMaxProperty("rangeX", "X Range", 0, 210, -256, 256)
        self.addProperty(self.rangeX, owner=False)
        self.rangeY = ivw.properties.FloatMinMaxProperty("rangeY", "Y Range", 0, 210, -256, 256)        
        self.addProperty(self.rangeY, owner=False)
        self.timesteps = IntProperty("steps", "Time Steps", 1, 1, 1000)
        self.addProperty(self.timesteps, owner=False)
        # Static parameter (epicenter)
        self.epicenter = ivw.properties.FloatVec2Property("epicenter", "Epicenter", vec2(30.4,40.6), vec2(-256,-256), vec2(256,256))
        self.addProperty(self.epicenter, owner=False)
        # Initial and changed parameter values (peak value, median radius, standard deviation)
        self.peak0 = ivw.properties.FloatProperty("peak0", "Inital Peak", 1, -10, 10)
        self.addProperty(self.peak0, owner=False)
        self.peakChange = ivw.properties.FloatProperty("peakChange", "Peak Change", 0.3, -10, 10)
        self.addProperty(self.peakChange, owner=False)
        self.omega0 = ivw.properties.FloatProperty("omega0", "Initial Std", 12.16, -20, 20)
        self.addProperty(self.omega0, owner=False)
        self.omegaChange = ivw.properties.FloatProperty("omegaChange", "Std Change", 0.05, -10, 10)
        self.addProperty(self.omegaChange, owner=False)
        self.mu0 = ivw.properties.FloatProperty("mu0", "Initial Median", 0, -10, 10)
        self.addProperty(self.mu0, owner=False)
        self.muChange = ivw.properties.FloatProperty("muChange", "Median Change", 1.94, -10, 10)
        self.addProperty(self.muChange, owner=False)
        # Output data range
        self.useCustomDataRange = ivw.properties.BoolProperty("useCustomRange", "Use Custom Range", False)
        self.addProperty(self.useCustomDataRange, owner=False)
        self.customDataRange = ivw.properties.DoubleMinMaxProperty("customDataRange", "Custom Data Range", \
            0.0, 1.0, -100000, 100000, 0.01, 0.01, \
            inviwopy.properties.InvalidationLevel.InvalidOutput, ivw.properties.PropertySemantics.Text)
        self.addProperty(self.customDataRange, owner=False)
        self.dataRange = ivw.properties.DoubleMinMaxProperty("dataRange", "Output Range", \
            0.0, 1.0, -100000, 100000, 0.01, 0.01, \
            inviwopy.properties.InvalidationLevel.Valid, ivw.properties.PropertySemantics.Text)
        self.addProperty(self.dataRange, owner=False)
        self.dataRange.readOnly = True

    @staticmethod
    def processorInfo():
        return ivw.ProcessorInfo(
    		classIdentifier = "org.inviwo.SpreadingRingGeneration", 
    		displayName = "Spreading Ring Generation",
    		category = "Python",
    		codeState = ivw.CodeState.Stable,
    		tags = ivw.Tags.PY
        )

    def getProcessorInfo(self):
        return SpreadingRingGeneration.processorInfo()

    def initializeResources(self):
        pass

    def process(self):
        # Extract property values
        dims = self.volSize.value
        size_x = dims[0]
        size_y = dims[1]
        range_x = self.rangeX.value
        range_y = self.rangeY.value

        # Create input
        x, y = np.mgrid[range_x[0]:range_x[1]:size_x*1j, range_y[0]:range_y[1]:size_y*1j]
        pos = np.dstack((x, y))

        c = self.epicenter.value
        peak_0 = self.peak0.value
        peak_a = self.peakChange.value
        mu_0 = self.mu0.value
        mu_a = self.muChange.value
        omega_0 = self.omega0.value
        omega_a = self.omegaChange.value
        timesteps = self.timesteps.value

        ring = spreading_ring(c, peak_0, peak_a, mu_0, mu_a, omega_0, omega_a)

        currValueMin = 1.0e20
        currValueMax = -1.0e20

        data = np.zeros([size_x, size_y, timesteps]).astype('float32')

        for t in range(timesteps):
            ring_data = ring.evaluate(pos, t)
            # Set data and value range
            data[:,:,t] = ring_data
            currValueMin = min(currValueMin, np.min(ring_data))
            currValueMax = max(currValueMax, np.max(ring_data))

        # Inviwo seems to understand the data as column-major??
        data = data.flatten(order='F').reshape((size_x,size_y, timesteps)).astype('float32')
        volume = ivw.data.Volume(data)
        if self.useCustomDataRange.value:
            customDataRange = self.customDataRange.value
            volume.dataMap.dataRange = customDataRange
            volume.dataMap.valueRange = customDataRange
        else:
            volume.dataMap.dataRange = dvec2(currValueMin, currValueMax)
            volume.dataMap.valueRange = dvec2(currValueMin, currValueMax)
        self.dataRange.value = dvec2(currValueMin, currValueMax)
        volume.interpolation = ivw.data.InterpolationType.Nearest

        self.outport.setData(volume)