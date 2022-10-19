import numpy as np
from scipy.stats import (multivariate_normal as mvn,norm)
from scipy.stats._multivariate import _squeeze_output
import matplotlib.pyplot as plt

# Adapted from http://gregorygundersen.com/blog/2020/12/29/multivariate-skew-normal/
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

class delta_sequence:
    
    def __init__(self, epsilon):
        self.epsilon = epsilon

    def evaluate(self, x, y):
        res = 1/math.pi * self.epsilon /(x**2 + y**2 + self.epsilon**2)
        return res

def ackley_function(x,y):
    return -20.0 * np.exp(-0.2 * np.sqrt(0.5 * (x**2 + y**2)))-np.exp(0.5 * (np.cos(2 * math.pi * x)+np.cos(2 * math.pi * y))) + math.e + 20

def single_ridge(x,y):
    return 0.5*np.exp(-(10*np.power(x - np.sin(2*y)/2, 2) + 1 * np.power(y, 2)))

def himmelblau(x,y):
    x = x*5
    y = y*5
    return (x**2 + y - 11)**2 + (x + y**2 - 7)**2

class gaussian_ring:
    
    def __init__(self, epicenter, peak, mu, omega):
        self.epicenter = epicenter
        self.peak = peak
        self.mu = mu
        self.omega = omega

    def evaluate(self, x):
        # distance to epicenter
        dist = np.sqrt(np.sum((x-self.epicenter) ** 2, axis=2))
        # use dist as x sort of for a Gaussian
        inner = (dist - self.mu)/self.omega
        return self.peak * np.exp(-0.5 * inner**2)

# Given in Inviwo: 
# size_x, size_y, range_x, range_y, 
# offset, scaleFactor
# function = 0 (gaussian), 1 (skewed normal), 2 (delta)
# comparison = 0 (<), 1 (<=), 2 (>=), 3(>)
# threshold, levelSetSize

# Common create input
x, y = np.mgrid[range_x[0]:range_x[1]:size_x*1j, range_y[0]:range_y[1]:size_y*1j]
pos = np.dstack((x, y))

data = np.zeros((size_x, size_y))

if (function == 0):
    # cov (sigma), loc (mu)
    data = mvn(mean=loc.array, cov=cov.array).pdf(pos)
elif (function == 1):
    # Given additionally in Inviwo: 
    # shape (alpha), cov (sigma), loc (xi), scale (omega)
    mvsn = multivariate_skewnorm(shape=shape.array, cov=cov.array, loc=loc.array, scale=scale)
    data = mvsn.pdf(pos)
elif (function == 2):
    deltaSeq = delta_sequence(epsilon)
    data = deltaSeq.evaluate(x,y)
elif (function == 3):
    data = ackley_function(x,y)
elif (function == 4):
    data = single_ridge(x,y)
elif (function == 5):
    data = himmelblau(x,y)
elif (function == 6):
    ring = gaussian_ring(epicenter=loc.array, peak=peak, mu=muRadius, omega=stdRadius)
    data = ring.evaluate(pos)
elif (function == 7):
    # Prototype
    deltaSeq = delta_sequence(0.4)
    data = deltaSeq.evaluate(x,y)
    cov1 = np.identity(2)*0.05
    mean1 = np.array([-0.7,-0.7])
    data1 = mvn(mean=mean1, cov=cov1).pdf(pos)
    print("Range 1", np.max(data1)-np.min(data1))
    cov2 = np.identity(2)*0.025
    mean2 = np.array([0.8,0.8])
    data2 = mvn(mean=mean2, cov=cov2).pdf(pos)
    print("Range 1", np.max(data2)-np.min(data2))
    data = 0.1*data2 + 0.2*data1 + 1.25*data

#print(data)

## Common again
# Scale to desired value range
data = offset + scaleFactor* data

# Compute segmentation according to comparison
if (comparison == 0):
    segmentation = data < threshold
    segmentation2 = data < threshold2
elif (comparison == 1):
    segmentation = data <= threshold
    segmentation2 = data <= threshold2
elif (comparison == 2):
    segmentation = data >= threshold
    segmentation2 = data >= threshold2
elif (comparison == 3):
    segmentation = data > threshold
    segmentation2 = data > threshold2

levelSetSize = np.count_nonzero(segmentation)
levelSetSize2 = np.count_nonzero(segmentation2)

# Determine nestedNess
combinedSegmentation = segmentation.astype('float32') + segmentation2.astype('float32')

# Inviwo seems to understand the data as column-major??
data = data.flatten(order='F').reshape((size_x,size_y, 1)).astype('float32')
combinedSegmentation = combinedSegmentation.flatten(order='F').reshape((size_x,size_y, 1))
vol.data=data
volSeg.data=combinedSegmentation