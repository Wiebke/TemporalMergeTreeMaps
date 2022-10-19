# Name: SkewedNormal2D 

import inviwopy as ivw
from inviwopy.glm import size2_t, vec2, mat2

# Adapted from http://gregorygundersen.com/blog/2020/12/29/multivariate-skew-normal/
import numpy as np
from scipy.stats import (multivariate_normal as mvn,norm)
from scipy.stats._multivariate import _squeeze_output
import matplotlib.pyplot as plt

class multivariate_skewnorm:
    
    def __init__(self, shape, cov=None, loc=None, scale=1):
        self.dim = len(shape)
        self.shape = np.asarray(shape)
        self.mean = np.zeros(self.dim)
        self.cov = np.eye(self.dim) if cov is None else np.asarray(cov)
        self.loc = np.zeros(self.dim) if loc is None else np.asarray(loc)
        self.scale = scale

    def pdf(self, x):
        return np.exp(self.logpdf(x))
        
    def logpdf(self, x):
        # skewnorm.pdf(x, a) = 2 * norm.pdf(x) * norm.cdf(a*x)
        # skewnorm.pdf(x, a, loc, scale) = skewnorm.pdf(y, a) / scale with y = (x - loc) / scale. 
        x    = mvn._process_quantiles(x, self.dim)
        pdf  = mvn(self.mean, self.cov).logpdf((x-self.loc)/self.scale)
        cdf  = norm(0, 1).logcdf(np.dot(x-self.loc, self.shape)/self.scale)
        return _squeeze_output(np.log(2/self.scale) + pdf + cdf)

class SkewedNormal2D(ivw.Processor):
    def __init__(self, id, name):
        ivw.Processor.__init__(self, id, name)
        self.outport = ivw.data.VolumeOutport("outport")
        self.addOutport(self.outport, owner=False)

        self.volSize = ivw.properties.IntSize2Property("dims", "Dimensions", size2_t(128,128), size2_t(1,1), size2_t(512,512))
        self.addProperty(self.volSize, owner=False)
        self.rangeX = ivw.properties.FloatMinMaxProperty("rangeX", "X Range", -1, 1, -10, 10)
        self.addProperty(self.rangeX, owner=False)
        self.rangeY = ivw.properties.FloatMinMaxProperty("rangeY", "Y Range", -1, 1, -10, 10)        
        self.addProperty(self.rangeY, owner=False)
        self.cov = ivw.properties.FloatMat2Property("cov", "Covariance", mat2(1,0,0,1), mat2(0,0,0,0), mat2(10,10,10,10), 
            semantics=ivw.properties.PropertySemantics.Text)
        self.addProperty(self.cov, owner=False)
        self.shape = ivw.properties.FloatVec2Property("shape", "Shape (alpha)", vec2(0,0), vec2(-10,-10), vec2(10,10))
        self.addProperty(self.shape, owner=False)
        self.scale = ivw.properties.FloatProperty("scale", "Scale (omega)", 1, 0.00001, 10)
        self.addProperty(self.scale, owner=False)
        self.loc = ivw.properties.FloatVec2Property("loc", "Location (xi)", vec2(0,0), vec2(-10,-10), vec2(10,10))
        self.addProperty(self.loc, owner=False)
        self.valueMin = ivw.properties.FloatProperty("valueMin", "Mapped to Min", 0, -10, 10)
        self.addProperty(self.valueMin, owner=False)
        self.valueMax = ivw.properties.FloatProperty("valueMax", "Mapped to Max", 1, -10, 10)
        self.addProperty(self.valueMax, owner=False)

    @staticmethod
    def processorInfo():
        return ivw.ProcessorInfo(
    		classIdentifier = "org.inviwo.SkewedNormal2D", 
    		displayName = "Skewed Normal 2D",
    		category = "Python",
    		codeState = ivw.CodeState.Stable,
    		tags = ivw.Tags.PY
        )

    def getProcessorInfo(self):
        return SkewedNormal2D.processorInfo()

    def initializeResources(self):
        pass

    def process(self):
        # Extract property values
        dims = self.volSize.value
        size_x = dims[0]
        size_y = dims[1]
        range_x = self.rangeX.value
        range_y = self.rangeY.value
        shape = self.shape.value.array
        cov = self.cov.value.array
        loc = self.loc.value.array
        scale = self.scale.values
        valueMin = self.valueMin.value
        valueMax = self.valueMax.value

        # Create Input for pdf function
        x, y = np.mgrid[range_x[0]:range_x[1]:size_x*1j, range_y[0]:range_y[1]:size_y*1j]
        pos = np.dstack((x, y))

        mvsn = multivariate_skewnorm(shape=shape, cov=cov, loc=loc, scale=scale)
        pdf = mvsn.pdf(pos)
        data = pdf.reshape((size_x, size_y, 1)).astype('float32')

        # Set data and value range
        currValueMin = np.min(data)
        currValueMax = np.max(data)

        # Inverse Mapping, take times -1, switch range
        if valueMax < valueMin:
            data = -data
            valueMaxInterm = valueMax
            valueMax = valueMin
            valueMin = valueMaxInterm

        data = (data - currValueMin)*(valueMax-valueMin)/(currValueMax-currValueMin) + valueMin

        minValue = np.min(data)
        maxValue = np.max(data)
        # Inviwo seems to understand the data as column-major??
        data = data.flatten(order='F').reshape((size_x,size_y, 1)).astype('float32')
        volume = ivw.data.Volume(data)
        volume.dataMap.dataRange = dvec2(minValue, maxValue)
        volume.dataMap.valueRange = dvec2(minValue, maxValue)

        self.outport.setData(volume)

