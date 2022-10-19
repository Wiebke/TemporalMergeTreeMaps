from paraview import servermanager
presets = paraview.servermanager.vtkSMTransferFunctionPresets()

lut = GetColorTransferFunction('dummy')
lut.ApplyPreset(lutName)

import vtk
helper = vtk.vtkUnsignedCharArray()

dctf = lut.GetClientSideObject()
dataRange = dctf.GetRange()

for i in range(0, 256):
x = (i/255.0) * (dataRange[1]-dataRange[0]) + dataRange[0]
helper.SetVoidArray(dctf.MapValue(x), 3, 1)
r = helper.GetValue(0)
g = helper.GetValue(1)
b = helper.GetValue(2)
print("%d %d %d" % (r, g, b))