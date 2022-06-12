from __future__ import print_function
import sys
# in vtk .exe


# if(len(sys.argv) != 3):
#   print('Usage: ', sys.argv[0], 'D:/ma/models/vessel_detection/vessel_1k7.obj D:/ma/models/vessel_detection/vessel_1k7.vtk')
#   sys.exit()

import vtk
reader = vtk.vtkOBJReader()
reader.SetFileName('D:/ma/models/vessel_detection/vessel_1k7.obj')
reader.Update()
obj = reader.GetOutput()
writer = vtk.vtkPolyDataWriter()
writer.SetFileName('D:/ma/models/vessel_detection/vessel_1k7.vtk')
if vtk.VTK_MAJOR_VERSION <= 5:
    writer.SetInput(obj)
else:
    writer.SetInputData(obj)
writer.Write()
