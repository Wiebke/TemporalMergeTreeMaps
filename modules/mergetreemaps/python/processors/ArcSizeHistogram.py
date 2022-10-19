# Name: ArcSizeHistogram 

import inviwopy as ivw
from inviwopy.glm import ivec3, dvec2
from inviwopy.properties import FloatProperty, IntProperty, BoolProperty
from ivwdataframe import DataFrameInport
import ivwdataframe
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib import rc


class ArcSizeHistogram(ivw.Processor):
    def __init__(self, id, name):
        ivw.Processor.__init__(self, id, name)
        self.inport = DataFrameInport("inport")
        self.addInport(self.inport, owner=False)
        self.imageOutport = ivw.data.ImageOutport("plot")
        self.addOutport(self.imageOutport, owner=False)

        self.min = FloatProperty("min", "Min", 0, 0, 4096)
        self.addProperty(self.min, owner=False)
        self.max = FloatProperty("max", "Max", 500, 0, 4096)
        self.addProperty(self.max, owner=False)
        self.numBins = IntProperty("numBins", "Bins", 100, 10, 1000)
        self.addProperty(self.numBins, owner=False)
        
        self.fontSize = IntProperty("fontSize", "Font Size", 12, 6, 36)
        self.addProperty(self.fontSize, owner=False)
        self.figureWidth = FloatProperty("figureWidth", "Width (in)", 5, 1, 8.27)
        self.addProperty(self.figureWidth, owner=False)
        self.figureHeight = FloatProperty("figureHeight", "Height Scale", 0.8, 0.1, 1.5)
        self.addProperty(self.figureHeight, owner=False)

    @staticmethod
    def processorInfo():
        return ivw.ProcessorInfo(
    		classIdentifier = "org.inviwo.ArcSizeHistogram", 
    		displayName = "Arc Size Histogram",
    		category = "Python",
    		codeState = ivw.CodeState.Stable,
    		tags = ivw.Tags.PY
        )

    def getProcessorInfo(self):
        return ArcSizeHistogram.processorInfo()

    def initializeResources(self):
        #print("init")
        pass

    def process(self):
        df = self.inport.getData()
        if (df is None):
            print("No input for arc size histogram")
            return
        col = df.column(1)
        data = col.buffer.data
        print("The ", data.size ," arcs have sizes between ", np.min(data), "and ", np.max(data))

        # Plot settings
        fontSize = self.fontSize.value
        minValue = self.min.value
        maxValue = self.max.value
        numBins = self.numBins.value
        width = self.figureWidth.value
        goldenRevert = 2/(1+np.sqrt(5))
        height = self.figureHeight.value * width * goldenRevert

        #print(plt.rcParams.get('figure.figsize'))
        rc("font", size=fontSize)

        fig, ax = plt.subplots(figsize=(width, height))
        canvas = FigureCanvasAgg(fig)

        ax.hist(data, bins=numBins, range=(minValue, maxValue))
        ax.set_ylabel('Number of Elements')
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        #ax.ticklabel_format(scilimits=(-2,2))
        ax.set_xlim([minValue, maxValue])
        
        # Convert to Inviwo image, so that Qt Pyplot problems are avoided
        canvas.draw()
        buf = canvas.buffer_rgba()
        # convert to a NumPy array
        X = np.asarray(buf)
        print(buf.shape)
        X = np.flip(X, axis=0).copy()
        imageLayer = ivw.data.Layer(X.transpose((1, 0, 2)))
        image = ivw.data.Image(imageLayer)
        self.imageOutport.setData(image)
        plt.close(fig)