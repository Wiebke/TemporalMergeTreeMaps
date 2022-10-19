# Name: IndexToTimeStampText 

import inviwopy as ivw
from inviwopy.glm import vec3, vec4
from inviwopy.data import DrawType, ConnectivityType, BasicMesh
import numpy as np
import datetime

class IndexToTimeStampText(ivw.Processor):
    def __init__(self, id, name):
        ivw.Processor.__init__(self, id, name)
        self.outport = ivw.data.MeshOutport("outport")
        self.addOutport(self.outport, owner=False)

        self.startTime = ivw.properties.CompositeProperty("startTime", "Starting Time")
        self.addProperty(self.startTime, owner=False)
        self.day = ivw.properties.IntProperty("day", "Day", 1, 1, 31, 1, semantics=ivw.properties.PropertySemantics.Text)
        self.startTime.addProperty(self.day, owner=False)
        self.month = ivw.properties.IntProperty("month", "Month", 12, 1, 12, 1, semantics=ivw.properties.PropertySemantics.Text)
        self.startTime.addProperty(self.month, owner=False)
        self.year = ivw.properties.IntProperty("year", "Year", 1999, 1, 2100, 1, semantics=ivw.properties.PropertySemantics.Text)
        self.startTime.addProperty(self.year, owner=False)
        self.hour = ivw.properties.IntProperty("hour", "Hour", 0, 0, 23, 1, semantics=ivw.properties.PropertySemantics.Text)
        self.startTime.addProperty(self.hour, owner=False)
        self.minute = ivw.properties.IntProperty("minute", "Minute", 0, 0, 23, 1, semantics=ivw.properties.PropertySemantics.Text)
        self.startTime.addProperty(self.minute, owner=False)
        
        self.interval = ivw.properties.CompositeProperty("interval", "Time Step Interval")
        self.addProperty(self.interval, owner=False)
        self.dayInterval = ivw.properties.IntProperty("daysInterval", "Days", 0, 0, 1000, 1, \
            semantics=ivw.properties.PropertySemantics.Text)
        self.interval.addProperty(self.dayInterval, owner=False)
        self.hourInterval = ivw.properties.IntProperty("hoursInterval", "Hours", 1, 0, 1000, 1, \
            semantics=ivw.properties.PropertySemantics.Text)
        self.interval.addProperty(self.hourInterval, owner=False)
        self.minuteInterval = ivw.properties.IntProperty("minutesInterval", "Minutes", 0, 0, 1000, 1, \
            semantics=ivw.properties.PropertySemantics.Text)
        self.interval.addProperty(self.minuteInterval, owner=False)
        self.secondInterval = ivw.properties.IntProperty("secondsInterval", "Seconds", 0, 0, 1000, 1, \
            semantics=ivw.properties.PropertySemantics.Text)
        self.interval.addProperty(self.secondInterval, owner=False)
        self.timeStep = ivw.properties.IntProperty("timeStep", "TimeStep", 1, 0)
        self.addProperty(self.timeStep, owner=False)
        self.zeroBased = ivw.properties.BoolProperty("zeroBasedIndex", "Zero-Based Index", False)
        self.addProperty(self.zeroBased, owner=False)
        self.timeStepFraction = ivw.properties.FloatVec3Property("timeStepFraction", "TimeStep Fraction", \
            vec3(0.5,0.5,0), vec3(0.5,0.5,0), vec3(0.5,0.5,1), \
            invalidationLevel=ivw.properties.InvalidationLevel.Valid,
            semantics=ivw.properties.PropertySemantics.Text)
        self.addProperty(self.timeStepFraction, owner=False)
        self.numTimeSteps = ivw.properties.IntProperty("numTimeSteps", "Number of Time Steps", 124, 2, 4096, \
            semantics=ivw.properties.PropertySemantics.Text)
        self.addProperty(self.numTimeSteps, owner=False)
        self.timeStepText = ivw.properties.StringProperty("timeStepText", "Time Step Text", "")
        self.addProperty(self.timeStepText, owner=False)
        self.timeStepText.readOnly = True

        self.meshColor = ivw.properties.FloatVec4Property("color", "Index Indicater Color", vec4(0,0,0,1), \
            semantics=ivw.properties.PropertySemantics.Color)
        self.addProperty(self.meshColor, owner=False)

    @staticmethod
    def processorInfo():
        return ivw.ProcessorInfo(
    		classIdentifier = "org.inviwo.IndexToTimeStampText", 
    		displayName = "Index To TimeStamp Text",
    		category = "Python",
    		codeState = ivw.CodeState.Stable,
    		tags = ivw.Tags.PY
        )

    def getProcessorInfo(self):
        return IndexToTimeStampText.processorInfo()

    def initializeResources(self):
        pass

    def process(self):
        ##Update properties
        t = self.timeStep.value
        if not self.zeroBased:
            t = t-1
        numTimeSteps = self.numTimeSteps.value
        if (self.zeroBased.value):
            self.timeStep.minValue = 0
            self.timeStep.maxValue = numTimeSteps-1
        else:
            self.timeStep.minValue = 1
            self.timeStep.maxValue = numTimeSteps
        outputMesh = BasicMesh()
        startDate =  datetime.datetime(year=self.year.value, month=self.month.value, day=self.day.value, hour=self.hour.value, minute=self.minute.value)

        posX = float(t) / (numTimeSteps-1)
        # Normalized timestep
        self.timeStepFraction.value = vec3(0.5, 0.5, posX)
        dx = 1.0 / numTimeSteps
        posX = 0.5 *dx + float(t) * dx
        outputMesh.addVertex(vec3(posX, 0.0, 0.0), vec3(0.0, 0.0, 1.0), vec3(0.0, 0.0, 0.0), vec4(self.color.value))
        outputMesh.addVertex(vec3(posX, 1.0, 0.0), vec3(0.0, 0.0, 1.0), vec3(0.0, 0.0, 0.0), vec4(self.color.value))
        indexBuffer = outputMesh.addIndexBuffer(DrawType.Lines, ConnectivityType.None_)
        indexBuffer.size = 2
        indexBuffer.data[0] = 0
        indexBuffer.data[1] = 1
        delta = datetime.timedelta(\
            days=self.dayInterval.value, \
            hours=self.hourInterval.value, \
            minutes=self.minuteInterval.value, \
            seconds=self.secondInterval.value)
        timeStepDate = startDate + t * delta
        self.timeStepText.value = timeStepDate.strftime("%Y-%m-%d %H:%M")
        #print(self.timeStepText)
        self.outport.setData(outputMesh)