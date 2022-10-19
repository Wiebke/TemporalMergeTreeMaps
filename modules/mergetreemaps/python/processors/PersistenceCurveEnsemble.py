# Name: PersistenceCurveEnsemble 

import inviwopy as ivw
from ivwdataframe import DataFrameFlatMultiInport
from inviwopy.properties import IntProperty, BoolProperty
import ivwdataframe
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rc
from matplotlib.backends.backend_agg import FigureCanvasAgg

class PersistenceCurveEnsemble(ivw.Processor):
    def __init__(self, id, name):
        ivw.Processor.__init__(self, id, name)
        self.inport = DataFrameFlatMultiInport("inport")
        self.addInport(self.inport, owner=False)
        self.imageOutport = ivw.data.ImageOutport("plot")
        self.addOutport(self.imageOutport, owner=False)

        self.timestep = IntProperty("selectedStep", "Selected Step", 1, 1, 1000)
        self.addProperty(self.timestep, owner=False)

        self.fontSize = IntProperty("fontSize", "Font Size", 12, 6, 36)
        self.addProperty(self.fontSize, owner=False)
        self.figureWidth = FloatProperty("figureWidth", "Width (in)", 5, 1, 8.27)
        self.addProperty(self.figureWidth, owner=False)
        self.figureHeight = FloatProperty("figureHeight", "Height Scale", 0.8, 0.1, 1.5)
        self.addProperty(self.figureHeight, owner=False)

        self.rangeX = ivw.properties.FloatMinMaxProperty("rangeX", "X Range", 0, 210, 0, 256, 0.0000001)
        self.addProperty(self.rangeX, owner=False)
        self.rangeY = ivw.properties.FloatMinMaxProperty("rangeY", "Y Range", 0, 100000, 0, 100000, 1)        
        self.addProperty(self.rangeY, owner=False)

    @staticmethod
    def processorInfo():
        return ivw.ProcessorInfo(
    		classIdentifier = "org.inviwo.PersistenceCurveEnsemble", 
    		displayName = "Persistence Curve Ensemble Plot",
    		category = "Python",
    		codeState = ivw.CodeState.Stable,
    		tags = ivw.Tags.PY
        )

    def getProcessorInfo(self):
        return PersistenceCurveEnsemble.processorInfo()

    def initializeResources(self):
        #print("init")
        pass

    def process(self):
        if (not self.inport.hasData() or not self.inport.isReady()):
            return

        dfs = self.inport.getVectorData()
        numDfs = len(dfs)
        if (numDfs == 0):
            print("No input for persistence curve ensemple.")
            return

        #print(plt.rcParams.get('figure.figsize'))
        fontSize = self.fontSize.value
        rc("font", size=fontSize)

        width = self.figureWidth.value
        goldenRevert = 2/(1+np.sqrt(5))
        height = self.figureHeight.value * width * goldenRevert
        fig, ax = plt.subplots(figsize=(width, height))
        canvas = FigureCanvasAgg(fig)

        ax.set_ylabel("Number of CP Pairs")
        ax.set_xlabel("Persistence")
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        
        selected_step = self.timestep.value

        for i in range(numDfs):
            df = dfs[i]
            persistenceColumnId = 0
            numPairsId = 0
            for colId in range(df.cols):
                if (df.column(colId).header == "Persistence"):
                    persistenceColumnId = colId
                elif (df.column(colId).header == "Number of CP Pairs"):
                    numPairsId = colId
            persistenceColumn = df.column(persistenceColumnId)
            persistence = persistenceColumn.buffer.data
            numPairsColumn = df.column(numPairsId)
            numPairs = numPairsColumn.buffer.data
            ax.plot(persistence, numPairs)
            if (i == selected_step):
                print("Maximum persistence", np.max(persistence))
                print("Number of pairs at min", np.max(numPairs))

        range_x = self.rangeX.value
        range_y = self.rangeY.value

        ax.set_xlim([range_x[0], range_x[1]])
        ax.set_ylim([range_y[0], range_y[1]])

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

