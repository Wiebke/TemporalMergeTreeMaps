# Name: ArcSizeBoxPlot

import inviwopy as ivw
from inviwopy.glm import size3_t
import ivwdataframe
from matplotlib.backends.backend_agg import FigureCanvasAgg
import matplotlib.cbook as cbook

class ArcSizeBoxPlot(ivw.Processor):
    def __init__(self, id, name):
        ivw.Processor.__init__(self, id, name)
        self.inportArcSizes = ivwdataframe.DataFrameFlatMultiInport("inportArcSizes")
        self.addInport(self.inportArcSizes, owner=False)
        self.imageOutport = ivw.data.ImageOutport("plot")
        self.addOutport(self.imageOutport, owner=False)
        self.numArcsOutport = ivwdataframe.DataFrameOutport("numArcs")
        self.addOutport(self.numArcsOutport, owner=False)
        self.dimSize = ivw.properties.IntSize3Property("dims", \
            "Dimensions", size3_t(64,64,64), size3_t(1,1,1), size3_t(4096,4096,4096), size3_t(1,1,1), \
            inviwopy.properties.InvalidationLevel.InvalidOutput, ivw.properties.PropertySemantics.Text)
        self.addProperty(self.dimSize, owner=False)
        self.dimProduct = ivw.properties.IntProperty("dimProduct", "Number of Voxels", 100, 1, 10000000, 1, \
            inviwopy.properties.InvalidationLevel.Valid, ivw.properties.PropertySemantics.Text)
        self.addProperty(self.dimProduct, owner=False)
        self.dimProduct.readOnly = True
        self.fontSize = IntProperty("fontSize", "Font Size", 12, 6, 36)
        self.addProperty(self.fontSize, owner=False)
        self.figureWidth = FloatProperty("figureWidth", "Width (in)", 16.54, 1, 16.54)
        self.addProperty(self.figureWidth, owner=False)
        self.figureHeight = FloatProperty("figureHeight", "Height Scale", 0.8, 0.1, 1.5)
        self.addProperty(self.figureHeight, owner=False)
        self.skipTicks = ivw.properties.IntProperty("skipTicks", "xTicks to skip", 100, 1, 1000, 1, \
            inviwopy.properties.InvalidationLevel.InvalidOutput, ivw.properties.PropertySemantics.Text)
        self.addProperty(self.skipTicks, owner=False)
        self.yMaxArcSize = ivw.properties.IntProperty("yMaxArcSize", "Arc Size Max", 100, 1, 10000000, 1, \
            inviwopy.properties.InvalidationLevel.InvalidOutput, ivw.properties.PropertySemantics.Text)
        self.addProperty(self.yMaxArcSize, owner=False)
        self.yMaxArcNumber = ivw.properties.IntProperty("yMaxArcNumber", "Arc Number Max", 700, 1, 1000, 1, \
            inviwopy.properties.InvalidationLevel.InvalidOutput, ivw.properties.PropertySemantics.Text)
        self.addProperty(self.yMaxArcNumber, owner=False)


    @staticmethod
    def processorInfo():
        return ivw.ProcessorInfo(
    		classIdentifier = "org.inviwo.ArcSizeBoxPlot", 
    		displayName = "Arc Size Box Plot",
    		category = "Python",
    		codeState = ivw.CodeState.Stable,
    		tags = ivw.Tags.PY
        )

    def getProcessorInfo(self):
        return ArcSizeBoxPlot.processorInfo()

    def initializeResources(self):
        pass

    def process(self):
        if (not self.inportArcSizes.hasData() or not self.inportArcSizes.isReady()):
            return

        dfs = self.inportArcSizes.getVectorData()
        numDfs = len(dfs)
        if (numDfs == 0):
            return 

        fontSize = self.fontSize.value
        rc("font", size=fontSize)

        width = self.figureWidth.value
        goldenRevert = 2/(1+np.sqrt(5))
        height = self.figureHeight.value * width * goldenRevert
        #fig, ax = plt.subplots()
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(width, height))
        canvas = FigureCanvasAgg(fig)

        dims = self.dimSize.value
        numVoxels = dims[0]*dims[1]*dims[2]
        self.dimProduct.value = numVoxels

        ax1.set_ylabel("")
        ax1.set_ylim([0, self.yMaxArcSize.value])
        ax1.get_xaxis().set_visible(False)
        ax1.set_xlim([-1, numDfs])
        ax1.set_ylabel("Size of super arcs")
        ax2.set_xlabel("Time Step")
        ax2.set_ylabel("Number of super arcs")
        ax2.set_xlim([-1, numDfs])
        ax2.set_ylim([0, self.yMaxArcNumber.value])
        labels = np.arange(0, numDfs, self.skipTicks.value)
        ax2.set_xticklabels(labels)
        ax2.set_xticks(labels)

        # print(numDfs)
        data = [None] * numDfs
        sizes = np.zeros((numDfs,1))
        # print(sizes)

        nonZero = 0
        for i in range(numDfs):
            df = dfs[i]
            numRows = df.rows
            # Exclude index column
            numCols = df.cols - 1 
            matrix = np.zeros((numRows, numCols))
            # One column is the index column, create separate counter
            colCounter = 0
            for colId in range(df.cols):
                column = df.column(colId)
                if (column.header != 'index'):
                    matrix[:, colCounter] = column.buffer.data
            data[i] = (matrix.flatten())
            sizes[i] = data[i].size
        
        stats = cbook.boxplot_stats(data, whis=(0, 100), bootstrap=10000)
        ax1.bxp(stats, positions=np.arange(0, numDfs))
        #print(stats)
        ax2.plot(sizes)
        #print(sizes)

        # Convert to Inviwo image, so that Qt Pyplot problems are avoided
        canvas.draw()
        buf = canvas.buffer_rgba()
        # convert to a NumPy array
        X = np.asarray(buf)
        # print(buf.shape)
        X = np.flip(X, axis=0).copy()
        imageLayer = ivw.data.Layer(X.transpose((1, 0, 2)))
        image = ivw.data.Image(imageLayer)
        self.imageOutport.setData(image)
        plt.close(fig)

        buffer = inviwopy.data.Buffer(sizes)
        d = ivwdataframe.DataFrame()
        d.addColumnFromBuffer('Number of Arcs', buffer)
        d.updateIndex()
        self.numArcsOutport.setData(d)
