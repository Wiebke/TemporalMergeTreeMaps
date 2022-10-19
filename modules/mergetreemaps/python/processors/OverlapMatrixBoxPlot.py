# Name: OverlapMatrixBoxPlot 

import inviwopy as ivw
from ivwdataframe import DataFrameFlatMultiInport
from matplotlib.backends.backend_agg import FigureCanvasAgg

class OverlapMatrixBoxPlot(ivw.Processor):
    def __init__(self, id, name):
        ivw.Processor.__init__(self, id, name)
        self.inportOverlap = DataFrameFlatMultiInport("inportOverlap")
        self.addInport(self.inportOverlap, owner=False)
        self.inportOverlapMask = DataFrameFlatMultiInport("inportOverlapMask")
        self.addInport(self.inportOverlapMask, owner=False)
        self.inportOverlapMask.optional = True
        self.imageOutport = ivw.data.ImageOutport("plot")
        self.addOutport(self.imageOutport, owner=False)

        self.filterByOverlapType = BoolProperty("filterByType", "Filter By Overlap Type")
        self.addProperty(self.filterByOverlapType, owner=False)

        self.fontSize = IntProperty("fontSize", "Font Size", 12, 6, 36)
        self.addProperty(self.fontSize, owner=False)
        self.figureWidth = FloatProperty("figureWidth", "Width (in)", 5, 1, 8.27)
        self.addProperty(self.figureWidth, owner=False)
        self.figureHeight = FloatProperty("figureHeight", "Height Scale", 0.8, 0.1, 1.5)
        self.addProperty(self.figureHeight, owner=False)


    @staticmethod
    def processorInfo():
        return ivw.ProcessorInfo(
    		classIdentifier = "org.inviwo.OverlapMatrixBoxPlot", 
    		displayName = "Overlap Matrix Box Plot",
    		category = "Python",
    		codeState = ivw.CodeState.Stable,
    		tags = ivw.Tags.PY
        )

    def getProcessorInfo(self):
        return OverlapMatrixBoxPlot.processorInfo()

    def initializeResources(self):
        pass

    def process(self):
        if (not self.inportOverlap.hasData() or not self.inportOverlap.isReady()):
            return

        if (self.filterByOverlapType.value and \
            (not self.inportOverlapMask.hasData() or not self.inportOverlapMask.isReady())):
            return

        dfs = self.inportOverlap.getVectorData()
        numDfs = len(dfs)
        if (numDfs == 0):
            return 

        filterByOverlapType = self.filterByOverlapType.value
        if (filterByOverlapType):
            filterDfs = self.inportOverlapMask.getVectorData()
            if (len(filterDfs) != numDfs):
                print("Number of filter matrices (%i) does not match with number of overlap matrices (%i)."% (len(filterDfs), numDfs))
                return

        fontSize = self.fontSize.value
        rc("font", size=fontSize)

        width = self.figureWidth.value
        goldenRevert = 2/(1+np.sqrt(5))
        height = self.figureHeight.value * width * goldenRevert
        fig, ax = plt.subplots(figsize=(width, height))
        canvas = FigureCanvasAgg(fig)

        ax.set_ylabel("")
        ax.set_xlabel("Time Step")

        data = [None] * numDfs

        nonZero = 0
        for i in range(numDfs):
            df = dfs[i]
            if (filterByOverlapType):
                filterDf = filterDfs[i]
            numRows = df.rows
            # Exclude index column
            numCols = df.cols - 1 
            matrix = np.zeros((numRows, numCols))
            filterMatrix = np.zeros((numRows, numCols))
            # One column is the index column, create separate counter
            colCounter = 0
            for colId in range(df.cols):
                column = df.column(colId)
                if (filterByOverlapType):
                    filterColumn = filterDf.column(colId)
                if (column.header != 'index'):
                    matrix[:, colCounter] = column.buffer.data
                    if (filterByOverlapType):
                        filterMatrix[:, colCounter] = filterColumn.buffer.data
                    colCounter=colCounter+1
            if (filterByOverlapType):
                matrix = matrix[filterMatrix > 0]
                nonZero = nonZero + np.count_nonzero(filterMatrix)
            data[i] = (matrix.flatten())
        
        if (filterByOverlapType):
            print("Totally filtered %i non-zero elements." % nonZero)

        ax.boxplot(data) 
        
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
