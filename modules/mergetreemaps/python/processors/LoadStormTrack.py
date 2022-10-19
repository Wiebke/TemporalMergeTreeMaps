# Name: LoadStormTrack 

import inviwopy as ivw
import numpy as np
import pandas as pd
import math
from inviwopy.glm import vec3, vec4
from inviwopy.data import DrawType, ConnectivityType, BasicMesh
import datetime
from scipy.stats import (multivariate_normal as mvn,norm)

class LoadStormTrack(ivw.Processor):
    def __init__(self, id, name):
        ivw.Processor.__init__(self, id, name)
        self.outportTracks = ivw.data.MeshOutport("outportTracks")
        self.addOutport(self.outportTracks, owner=False)
        self.outportPoints = ivw.data.MeshOutport("outportPoints")
        self.addOutport(self.outportPoints, owner=False)
        self.outportField = ivw.data.VolumeOutport("outportGaussianTrackField")
        self.addOutport(self.outportField, owner=False)

        self.trackFile = ivw.properties.FileProperty("file", "Track File")
        self.addProperty(self.trackFile, owner=False)
        self.timeStepInTrack = ivw.properties.BoolProperty("timeStepInTrack", "Time Step in Track", False)
        self.addProperty(self.timeStepInTrack, owner=False)
        self.currentTimeStepAsSpiral = ivw.properties.BoolProperty("currentTimeStepAsSpiral", "Line Mesh For Current TimeStep")
        self.addProperty(self.currentTimeStepAsSpiral, owner=False)
        self.trackColor = ivw.properties.FloatVec4Property("trackColor", "Track Color", vec4(0,0,0,1), semantics=ivw.properties.PropertySemantics.Color)
        self.addProperty(self.trackColor, owner=False)
        self.timeStepColor = ivw.properties.FloatVec4Property("TimeStepColor", "Time Step Color", vec4(0,0,0,1), semantics=ivw.properties.PropertySemantics.Color)
        self.addProperty(self.timeStepColor, owner=False)
        self.gaussianField = ivw.properties.CompositeProperty("gaussField", "Gaussian Blob for Track")
        self.addProperty(self.gaussianField, owner=False)
        self.createVolOutput = ivw.properties.BoolProperty("createVolOutput", "Create Volume")
        self.gaussianField.addProperty(self.createVolOutput, owner=False)
        self.volSize = ivw.properties.IntSize2Property("dims", "Dimensions", size2_t(281,181), size2_t(1,1), size2_t(512,512))
        self.gaussianField.addProperty(self.volSize, owner=False)
        self.rangeX = ivw.properties.FloatMinMaxProperty("rangeX", "Lon Range", -30, 40, -180, 180)
        self.gaussianField.addProperty(self.rangeX, owner=False)
        self.rangeY = ivw.properties.FloatMinMaxProperty("rangeY", "Lat Range", 35, 80, 0, 360)        
        self.gaussianField.addProperty(self.rangeY, owner=False)
        self.cov = ivw.properties.FloatMat2Property("cov", "Covariance", mat2(1,0,0,1), mat2(0,0,0,0), mat2(10,10,10,10), 
            semantics=ivw.properties.PropertySemantics.Text)
        self.gaussianField.addProperty(self.cov, owner=False)
        self.scale = ivw.properties.FloatProperty("scale", "Scale", 1, 0.00001, 10)
        self.gaussianField.addProperty(self.scale, owner=False)

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
        self.dayInterval = ivw.properties.IntProperty("daysInterval", "Days", 0, 0, 1000, 1, semantics=ivw.properties.PropertySemantics.Text)
        self.interval.addProperty(self.dayInterval, owner=False)
        self.hourInterval = ivw.properties.IntProperty("hoursInterval", "Hours", 1, 0, 1000, 1, semantics=ivw.properties.PropertySemantics.Text)
        self.interval.addProperty(self.hourInterval, owner=False)
        self.minuteInterval = ivw.properties.IntProperty("minutesInterval", "Minutes", 0, 0, 1000, 1, semantics=ivw.properties.PropertySemantics.Text)
        self.interval.addProperty(self.minuteInterval, owner=False)
        self.secondInterval = ivw.properties.IntProperty("secondsInterval", "Seconds", 0, 0, 1000, 1, semantics=ivw.properties.PropertySemantics.Text)
        self.interval.addProperty(self.secondInterval, owner=False)
        self.timeStep = ivw.properties.IntProperty("timeStep", "TimeStep", 1, 0)
        self.addProperty(self.timeStep, owner=False)
        self.zeroBased = ivw.properties.BoolProperty("zeroBasedIndex", "Zero-Based Index", False)
        self.addProperty(self.zeroBased, owner=False)
        self.numTimeSteps = ivw.properties.IntProperty("numTimeSteps", "Number of Time Steps", 124, 2, 4096, semantics=ivw.properties.PropertySemantics.Text)
        self.addProperty(self.numTimeSteps, owner=False)


    @staticmethod
    def processorInfo():
        return ivw.ProcessorInfo(
    		classIdentifier = "org.inviwo.LoadStormTrack", 
    		displayName = "Load Storm Track from CSV",
    		category = "Python",
    		codeState = ivw.CodeState.Stable,
    		tags = ivw.Tags.PY
        )

    def getProcessorInfo(self):
        return LoadStormTrack.processorInfo()

    def initializeResources(self):
        pass

    def drawSpiral(self, centerX, centerY, z=0.0, coils=10, radius=0.5, chord=0.05):
        """Creates a spiral starting at centerX, centerY
        Adapted from: https://stackoverflow.com/questions/13894715/draw-equidistant-points-on-a-spiral
        chord -- distance between points to plot
        """
        counter = 0
        normal = vec3(0.0, 0.0, 1.0)
        texture = vec3(0.0, 0.0, 0.0)
        color = self.timeStepColor.value
        outputMeshTimeStep = BasicMesh()
        outputMeshTimeStep.addVertex(vec3(centerX, centerY, z), normal, texture, color)
        if not self.currentTimeStepAsSpiral.value:
            indexBufferTimeStep = outputMeshTimeStep.addIndexBuffer(DrawType.Points, ConnectivityType.None_)
            indexBufferTimeStep.size=1
            indexBufferTimeStep.data[counter] = counter
            return outputMeshTimeStep, indexBufferTimeStep
        counter = counter+1

        #value of theta corresponding to end of last coil
        thetaMax = coils * 2 * math.pi;

        #How far to step away from center for each side.
        awayStep = radius / thetaMax;

        #For every side, step around and away from center.
        #start at the angle corresponding to a distance of chord
        #away from centre.
        theta = chord / awayStep
        while (theta <= thetaMax):
            #How far away from center
            #fraction of 0 to thetaMax
            away = awayStep * theta 
            x = centerX + math.cos ( theta ) * away
            y = centerY + math.sin ( theta ) * away
            #to a first approximation, the points are on a circle
            #so the angle between them is chord/radius
            theta += chord / away
            outputMeshTimeStep.addVertex(vec3(x, y, z), normal, texture, color)
            counter = counter+1

        indexBufferTimeStep = outputMeshTimeStep.addIndexBuffer(DrawType.Lines, ConnectivityType.StripAdjacency)
        indexBufferTimeStep.size=counter
        indexBufferTimeStep.data = np.arange(0, counter, 1, dtype=np.uint32)
        return outputMeshTimeStep, indexBufferTimeStep


    def process(self):
        fileName = self.trackFile.value
        #print(fileName)
        df = pd.read_csv(fileName)
        if 'Date and time (YYYYMMDDHH)' not in df.columns:
            print("Date and time (YYYYMMDDHH) column in track file. Aborting")
            return 
        df['Date and time (YYYYMMDDHH)'] =  pd.to_datetime(df['Date and time (YYYYMMDDHH)'],\
            format='%Y%m%d %H:%M:%S')
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
        startDate =  datetime.datetime(year=self.year.value, month=self.month.value, day=self.day.value, hour=self.hour.value, minute=self.minute.value)
        delta = datetime.timedelta(days=self.dayInterval.value, hours=self.hourInterval.value,\
            minutes=self.minuteInterval.value, seconds=self.secondInterval.value)
        df['TimeStep'] = (df['Date and time (YYYYMMDDHH)'] - startDate)/delta

        df = df.filter(['TimeStep', \
                        'Longitude of associated MSLP minimum (degrees)', \
                        'Latitude of associated MSLP minimum (degrees)'])
        # Drop out of time range 
        df = df[df['TimeStep']<numTimeSteps]
        df = df[df['TimeStep']>0]
        df['TimeStep'] = df['TimeStep'].astype('int')
        # Drop out of range values
        df = df[df['Longitude of associated MSLP minimum (degrees)']< 1e+10]
        # Convert lontitude range from 0 to 360 to -180 to 180
        df.loc[:, 'Longitude of associated MSLP minimum (degrees)'] = \
            df['Longitude of associated MSLP minimum (degrees)'].apply(lambda x: x if x<180 else x - 360)
        # Interpolate missing time steps
        maxT = df['TimeStep'].max()
        minT = df['TimeStep'].min()
        add_indices = pd.Index(range(minT, maxT)).difference(df['TimeStep'])
        df.set_index('TimeStep',drop=True,inplace=True)
        add_df = pd.DataFrame(index=add_indices, columns=df.columns)
        df = pd.concat([df, add_df])
        df['TimeStep'] = df.index
        df = df.sort_values(by=['TimeStep'],ignore_index=True)
        df = df.interpolate(columns=['TimeStep'])

        # Prepare output
        outputMeshTrack = BasicMesh()
        indexBuffer = outputMeshTrack.addIndexBuffer(DrawType.Lines, ConnectivityType.StripAdjacency)
        indexBuffer.size = df.shape[0]

        dims = self.volSize.value
        size_x = dims[0]
        size_y = dims[1]
        range_x = self.rangeX.value
        range_y = self.rangeY.value
        cov = self.cov.value.array
        scale = self.scale.value
        x, y = np.mgrid[range_x[0]:range_x[1]:size_x*1j, range_y[0]:range_y[1]:size_y*1j]
        pos = np.dstack((x, y))
        data = np.zeros((size_x, size_y, numTimeSteps))

        normal = vec3(0.0, 0.0, 1.0)
        texture = vec3(0.0, 0.0, 0.0)
        color = self.trackColor.value
        counter = 0
        drewTimeStep = False

        for index, row in df.iterrows():
            lon = row['Longitude of associated MSLP minimum (degrees)']
            if lon > 1e+10:
                continue
            if (lon >= 180):
                lon = lon - 360
            lat = row['Latitude of associated MSLP minimum (degrees)']
            currentTimeStep = row['TimeStep']
            # Sip the future
            if (currentTimeStep > t):
                continue
            if (not self.timeStepInTrack.value):
                currentTimeStep = 0.0
            outputMeshTrack.addVertex(vec3(lon, lat, currentTimeStep), normal, texture, color)
            #print(lon, lat)
            indexBuffer.data[counter] = counter
            counter = counter+1
            if (row['TimeStep'] == t):
                outputMeshTimeStep, indexBufferTimeStep = self.drawSpiral(lon, lat, z=currentTimeStep, coils=5, radius=0.1, chord=0.01)
                drewTimeStep = True
            currentTimeStep = int(row['TimeStep'])
            if (self.createVolOutput.value):
                data[:,:,currentTimeStep] = mvn(mean=[lon,lat], cov=cov).pdf(pos)*scale
        indexBuffer.size = counter+1

        self.outportTracks.setData(outputMeshTrack)
        if not drewTimeStep:
            outputMeshTimeStep = BasicMesh()
        self.outportPoints.setData(outputMeshTimeStep)

        minValue = np.min(data)
        maxValue = np.max(data)
        if (self.createVolOutput.value):
            # Inviwo seems to understand the data as column-major??
            data = data.flatten(order='F').reshape((size_x,size_y,numTimeSteps)).astype('float32')
        else:
            data = data.astype('float32')
        
        volume = ivw.data.Volume(data)
        volume.dataMap.dataRange = dvec2(minValue, maxValue)
        volume.dataMap.valueRange = dvec2(minValue, maxValue)
        self.outportField.setData(volume)