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
        x    = mvn._process_quantiles(x, self.dim)
        pdf  = mvn(self.mean, self.cov).logpdf((x-self.loc)/self.scale)
        cdf  = norm(0, 1).logcdf(np.dot(x-self.loc, self.shape)/self.scale)
        return _squeeze_output(np.log(2/self.scale) + pdf + cdf)

#Given in Inviwo: size_x, size_y, range_x, range_y, shape, cov, loc, share, cov, loc, scale, valueMin, valueMax

# Create Input for pdf function
x, y = np.mgrid[range_x[0]:range_x[1]:size_x*1j, range_y[0]:range_y[1]:size_y*1j]
pos = np.dstack((x, y))

# Compute skewed normal function
mvsn = multivariate_skewnorm(shape=shape.array, cov=cov.array, loc=loc.array, scale=scale)
data = mvsn.pdf(pos)

currValueMin = np.min(data)
currValueMax = np.max(data)

# Inverse Mapping, take times -1, switch range
if valueMax < valueMin:
    data = -data
    valueMaxInterm = valueMax
    valueMax = valueMin
    valueMin = valueMaxInterm
    currValueMaxInterm = currValueMax
    currValueMin = currValueMax
    currValueMax = currValueMin

# Scale to desired value range
data = (data - currValueMin)*(valueMax-valueMin)/(currValueMax-currValueMin) + valueMin
# Inviwo seems to understand the data as column-major??
data = data.flatten(order='F').reshape((size_x,size_y, 1)).astype('float32')
volume = ivw.data.Volume(data)