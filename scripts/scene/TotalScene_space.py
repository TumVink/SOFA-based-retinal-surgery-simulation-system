import Sofa
import os
#from stlib.physics.rigid import Floor
import numpy as np
import math as m
from stlib.physics.rigid import Cube, Sphere, Floor
import time
import pickle
from multiprocessing.connection import Client

address = ('localhost', 6000)
conn = Client(address, authkey='secret password')

def Rx(theta):
    theta = m.radians(theta)
    return np.matrix([[1, 0, 0],
                      [0, m.cos(theta), -m.sin(theta)],
                      [0, m.sin(theta), m.cos(theta)]])
def Ry(theta):
    theta = m.radians(theta)
    return np.matrix([[m.cos(theta), 0, m.sin(theta)],
                      [0, 1, 0],
                      [-m.sin(theta), 0, m.cos(theta)]])
def Rz(theta):
    theta = m.radians(theta)
    return np.matrix([[m.cos(theta), -m.sin(theta), 0],
                      [m.sin(theta), m.cos(theta), 0],
                      [0, 0, 1]])

def Force_transf(dx,dy,dz,force):
    # input: dx,dy,dz in degrees
    #       force in list[x,y,z] or [0,0.1,0]
    # output: transformed force in list [x,y,z,0,0,0]

    R = np.round(Rx(dx)*Ry(dy)*Rz(dz),decimals=3)

    R = -R.transpose()
    R[:,2] = -R[:,2]
    print R
    input_force = np.array(force)
    print input_force
    output_force = np.matmul(R,input_force)
    print output_force
    output_force = [output_force[0],output_force[1],output_force[2],0,0,0]#str(output_force[])

    return output_force

def ConstraintForce_tranf(dx,dy,dz,ConstraintForce):
    # input: dx,dy,dz in degrees
    #       force in list[x,y,z] or [0,0.1,0]
    # output: transformed force in list [x,y,z,0,0,0]

    R = np.round(Rx(dx)*Ry(dy)*Rz(dz),decimals=3)

    R = -R.transpose()
    R[:,2] = -R[:,2]
    print R
    input_force = np.array(force)
    print input_force
    output_force = np.matmul(R,ConstraintForce)
    print output_force
    output_force = [output_force[0],output_force[1],output_force[2]]#str(output_force[])

    return output_force

def write_txt(fs):
    pass

def preprocess(arr):

    arr = arr[:-1]
    normal = [0.5,0.5,0.5,0.5,0.5,0.5]
    out = [0,0,0,0,0,0]

    for i in range(len(arr)):
        out[i] = abs(arr[i]) / normal[i]
    max_value = max(out)
    max_index = out.index(max_value)
    max_value = arr[max_index]

    # for j in len(out):
    #     if j != max_index:
    #         out[j] = 0
    #print arr
    return [max_value,max_index]


class MainScene(Sofa.PythonScriptController):

    def createGraph(self, node):
        dt = 0.01*5  # In second
        displayFlags = []

        self.rootNode = node.getRoot()
        self.rootNode.dt = dt
        self.rootNode.gravity = [0, 0, 0]
        self.step = 0

        self.rootNode.createObject('RequiredPlugin', name='SofaMiscCollision')
        self.rootNode.createObject('RequiredPlugin', name='SofaPython')
        self.rootNode.createObject('RequiredPlugin', name='CImgPlugin')
        #self.rootNode.createObject('RequiredPlugin', name='SofaOpenglVisual')

        self.rootNode.createObject('VisualStyle', displayFlags=displayFlags)
        self.rootNode.createObject('FreeMotionAnimationLoop')

        self.rootNode.createObject('GenericConstraintSolver', maxIterations=1000, tolerance=1e-07, computeConstraintForces=False)
        self.rootNode.createObject('DefaultPipeline', verbose='0', depth="6", draw='1')
        self.rootNode.createObject('BruteForceDetection', name='N2')
        # self.rootNode.createObject('DiscreteIntersection', name='Intersection')
        self.rootNode.createObject("LocalMinDistance", name="Intersection", alarmDistance="0.2", contactDistance="0.012", useLMDFilters="0")
        #0.1/0.0122   --- young moodle 200   basic
        #0.2/0.012    --- young module 100 more deformation

        self.rootNode.createObject('DefaultContactManager', name="Response", response="FrictionContactConstraint",
                                   responseParams='mu=0.8')

        self.rootNode.createObject('OglViewport', screenPosition="0 0", screenSize="500 500", cameraPosition="-6.75 4.99 1",  #dx=6.25, dy=2.75, dz=-9.5
                                  cameraOrientation="-0 -0 -0 1", drawCamera="true")

        # self.rootNode.createObject('interactiveCamera', name='baseCamera')
        # self.rootNode.createObject('interactiveCamera', name='insideCamera')
        self.trans = self.rootNode.createObject('TransformEngine', name="transform", template="Rigid3d", translation="0 0 0",
                                   rotation=[0,0,0], scale="1 1 1",listening="True")
        self.needle_pose = [0,0,0]
        self.rotation_matrix = self.rootNode.createObject('RotateTransformMatrixEngine',rotation=[0,0,0],listening="True")
        self.angleX = 0
        self.angleY = 0
        self.angleZ = 0
        self.magnification = self.rootNode.getObject('OglViewport').cameraPosition[0][2]

        # tasks
        # True if task is finished
        self.count_sec = [0,0]
        self.ball_size = 0.3
        self.task = [False,False]
        self.points = 0
        self.time_start = time.time()
        self.average_time = 100      # the average time
        self.times_wound = 0
        self.eyeball_center = [-6.75,2.75,-9.5]
        self.eyeball_radius = 12
        self.gang = 5
        self.sec = 0
        self.wounds_locked = False
        self.finish = False
        #print self.file_name

# # ==========================
# # Eyeball NODE
# # ==========================
#         meshFile_col = "../../models/eyeball/eyeball_origin_4k.obj"
#         meshFile_viz = "../../models/eyeball/eyeball_origin_7k.obj"
#
#     #overall def
#         eyeball_node= self.rootNode.createChild("eyeball")
#         scale=1
#         #fatNode.gravity = [0, -1, 0]
#
#         eyeball_node.createObject('EulerImplicitSolver', rayleighStiffness=0.1, rayleighMass=0.1)
#         eyeball_node.createObject('CGLinearSolver', iterations=25, tolerance=1e-11, threshold=1e-18, printLog=False)
#
#         eyeball_node.createObject('MeshObjLoader',name="meshLoader0",filename=meshFile_col)
#         #eyeball_node.createObject('MeshTopology',name="mesh",src="@meshLoader0")
#         eyeball_node.createObject('MechanicalObject', name='mechObjectEyeball', template="Rigid3d",dx=-6.75, dy=2.75, dz=-9.5, rx=0,ry=0, rz=0,scale=scale,showObject="1")
#         eyeball_node.createObject('UniformMass', template="Vec3d", name='mass', totalMass="100")
#
#         # eyeball_node.createObject('RegularGridTopology', nx=2, ny=20, nz=10, xmin=-0.08, xmax=0.08, ymin=-0.08, ymax=0.08,
#         #                      zmin=0, zmax=2)
#         #
#         # eyeball_node.createObject('TetrahedronFEMForceField', template='Vec3d', name='FEM_tissue',
#         #                      method='polar', poissonRatio='0.495', youngModulus='0.0006', computeVonMisesStress='1',
#         #                      showVonMisesStressPerNode='false', listening='1', updateStiffness='true',
#         #                      showStressColorMap='blue 1 0 0 1  1 0.5 0.5 1', isToPrint='1')
#
#         #
#         #eyeball_node.createObject('UncoupledConstraintCorrection')
#         #self.eyeMecha = eyeball_node.getObject('mechObjectEyeball')
#
#         #fatNode.createObject('UniformMass', totalMass="0.5")
#         # eyeball_node.createObject('BoxROI', name='BoxROI', box=[-0.5,-0.5,0,0.5,0.5,2.5], drawBoxes=True,
#         #                doUpdate=False)
#         # eyeball_node.createObject('RestShapeSpringsForceField', points='@BoxROI.indices', stiffness='1e12')
#
#         # Visual node
#         eyeVisNode = eyeball_node.createChild('Visual')
#         eyeVisNode.createObject('MeshObjLoader', name='meshLoader', filename=meshFile_viz)
#         eyeVisNode.createObject('OglModel', src='@meshLoader', color="0.7 0.7 0.7 1", name="Eye",scale3d=[1,1,1], texturename="../../models/texture/texture_2.jpg")
#         eyeVisNode.createObject('RigidMapping', name="Visual Mapping", output="@Eye")

        # Collision

# # ==========================
# # Eyeball Upper part
# # ==========================
#
        meshFile_col = "../../models/eyeball/eyeball_sclera.obj"
        meshFile_viz = "../../models/eyeball/eyeball_sclera.obj"

        # overall def
        eyeball_node = self.rootNode.createChild("eyeball_top")
        scale = 1
        # fatNode.gravity = [0, -1, 0]

        eyeball_node.createObject('EulerImplicitSolver', rayleighStiffness=0.1, rayleighMass=0.1)
        eyeball_node.createObject('CGLinearSolver', iterations=25, tolerance=1e-11, threshold=1e-18, printLog=False)

        eyeball_node.createObject('MeshObjLoader', name="meshLoader0", filename=meshFile_col)
        # eyeball_node.createObject('MeshTopology',name="mesh",src="@meshLoader0")
        eyeball_node.createObject('MechanicalObject', name='mechObjectEyeball', template="Rigid3d", dx=-6.75, dy=2.75,
                                  dz=-9.5, rx=0, ry=0, rz=0, scale=scale, showObject="1")
        eyeball_node.createObject('UniformMass', template="Vec3d", name='mass', totalMass="100")


        # Visual node
        eyeVisNode = eyeball_node.createChild('Visual')
        eyeVisNode.createObject('MeshObjLoader', name='meshLoader', filename=meshFile_viz)
        eyeVisNode.createObject('OglModel', src='@meshLoader', color="0.4 0.4 0.4 0.4", name="Eye", scale3d=[1, 1, 1])#,
                                #texturename="../../models/texture/eye_1.jpg")
        eyeVisNode.createObject('RigidMapping', name="Visual Mapping", output="@Eye")



# ==========================
# Eyeball Upper part
# ==========================

        meshFile_col = "../../models/eyeball/eye_white.obj"
        meshFile_viz = "../../models/eyeball/eye_white.obj"

        # overall def
        eyeball_node = self.rootNode.createChild("eyeball_bot")
        scale = 1
        # fatNode.gravity = [0, -1, 0]

        eyeball_node.createObject('EulerImplicitSolver', rayleighStiffness=0.1, rayleighMass=0.1)
        eyeball_node.createObject('CGLinearSolver', iterations=25, tolerance=1e-11, threshold=1e-18, printLog=False)

        eyeball_node.createObject('MeshObjLoader', name="meshLoader0", filename=meshFile_col)
        # eyeball_node.createObject('MeshTopology',name="mesh",src="@meshLoader0")
        eyeball_node.createObject('MechanicalObject', name='mechObjectEyeball', template="Rigid3d", dx=-6.75, dy=2.75,
                                  dz=-9.5, rx=0, ry=0, rz=0, scale=scale, showObject="1")
        eyeball_node.createObject('UniformMass', template="Vec3d", name='mass', totalMass="100")

        # Visual node
        eyeVisNode = eyeball_node.createChild('Visual')
        eyeVisNode.createObject('MeshObjLoader', name='meshLoader', filename=meshFile_viz)
        eyeVisNode.createObject('OglModel', src='@meshLoader', color="0.7 0.7 0.7 1", name="Eye", scale3d=[1, 1, 1],
                    texturename="../../models/texture/texture_2.jpg")
        eyeVisNode.createObject('RigidMapping', name="Visual Mapping", output="@Eye")


# ==========================
# Eyeball Iris part
# ==========================

        meshFile_col = "../../models/eyeball/iris.obj"
        meshFile_viz = "../../models/eyeball/iris.obj"

        # overall def
        eyeball_node = self.rootNode.createChild("eyeball_iris")
        scale = 11.0
        # fatNode.gravity = [0, -1, 0]

        eyeball_node.createObject('EulerImplicitSolver', rayleighStiffness=0.1, rayleighMass=0.1)
        eyeball_node.createObject('CGLinearSolver', iterations=25, tolerance=1e-11, threshold=1e-18, printLog=False)

        eyeball_node.createObject('MeshObjLoader', name="meshLoader0", filename=meshFile_col)
        # eyeball_node.createObject('MeshTopology',name="mesh",src="@meshLoader0")
        eyeball_node.createObject('MechanicalObject', name='mechObjectEyeball', template="Rigid3d", dx=-6.75, dy=2.75,
                                  dz=-8.5, rx=-90, ry=0, rz=0, scale=scale, showObject="10")#,rotation=[90,0,0])
        eyeball_node.createObject('UniformMass', template="Vec3d", name='mass', totalMass="1")

        # Visual node
        eyeVisNode = eyeball_node.createChild('Visual')
        eyeVisNode.createObject('MeshObjLoader', name='meshLoader', filename=meshFile_viz)
        eyeVisNode.createObject('OglModel', src='@meshLoader', color="0.3 0.3 0.3 1", name="Eye", scale3d=[scale, scale, scale],
                                texturename="../../models/texture/iris_background.png")#0.54 0.27 0.07
        eyeVisNode.createObject('RigidMapping', name="Visual Mapping", output="@Eye")


# ==========================
# vessel NODE
# ==========================
        meshFile_col = "../../models/vessel/vessel_3d/vessel_4k_complete.obj"
        meshFile_viz = "../../models/vessel/vessel_3d/vessel_8k_complete.obj"

        # overall def
        vessel_node = self.rootNode.createChild("vessel")
        scale = 1
        # fatNode.gravity = [0, -1, 0]

        vessel_node.createObject('EulerImplicitSolver', rayleighStiffness=0.1, rayleighMass=0.1)
        vessel_node.createObject('CGLinearSolver', iterations=25, tolerance=1e-11, threshold=1e-18, printLog=False)

        vessel_node.createObject('MeshObjLoader', name="meshLoader0", filename=meshFile_col)
        # eyeball_node.createObject('MeshTopology',name="mesh",src="@meshLoader0")
        vessel_node.createObject('MechanicalObject', name='mechObject', template="Rigid3d", dx=-6.75, dy=2.75, dz=-9.5, rx=0,
                                  ry=0, rz=0, scale=scale, showObject="1")
        vessel_node.createObject('UniformMass', template="Vec3d", name='mass', totalMass="100")

        # eyeball_node.createObject('RegularGridTopology', nx=2, ny=20, nz=10, xmin=-0.08, xmax=0.08, ymin=-0.08, ymax=0.08,
        #                      zmin=0, zmax=2)
        #
        # eyeball_node.createObject('TetrahedronFEMForceField', template='Vec3d', name='FEM_tissue',
        #                      method='polar', poissonRatio='0.495', youngModulus='0.0006', computeVonMisesStress='1',
        #                      showVonMisesStressPerNode='false', listening='1', updateStiffness='true',
        #                      showStressColorMap='blue 1 0 0 1  1 0.5 0.5 1', isToPrint='1')

        #
        # eyeball_node.createObject('UncoupledConstraintCorrection')

        # fatNode.createObject('UniformMass', totalMass="0.5")
        # eyeball_node.createObject('BoxROI', name='BoxROI', box=[-0.5,-0.5,0,0.5,0.5,2.5], drawBoxes=True,
        #                doUpdate=False)
        # eyeball_node.createObject('RestShapeSpringsForceField', points='@BoxROI.indices', stiffness='1e12')

        # Visual node
        vesselVisNode = vessel_node.createChild('Visual')
        vesselVisNode.createObject('MeshObjLoader', name='meshLoader', filename=meshFile_viz)
        vesselVisNode.createObject('OglModel', src='@meshLoader', color="0.7 0 0", name="Vessel", scale3d=[1, 1, 1])
        vesselVisNode.createObject('RigidMapping', name="Visual Mapping", output="@Vessel")
        # Collision

# ==========================
# Trocar NODE
# ==========================
        meshFile_col = "../../models/trocar/trocars_left&right.obj"
        meshFile_viz = "../../models/trocar/trocars_left&right.obj"

        # overall def
        trocar_node = self.rootNode.createChild("trocar")
        scale = 1
        # fatNode.gravity = [0, -1, 0]

        trocar_node.createObject('EulerImplicitSolver', rayleighStiffness=0.1, rayleighMass=0.1)
        trocar_node.createObject('CGLinearSolver', iterations=25, tolerance=1e-11, threshold=1e-18, printLog=False)

        trocar_node.createObject('MeshObjLoader', name="meshLoader0", filename=meshFile_col)
        #trocar_node.createObject('MeshTopology', name="mesh", src="@meshLoader0")
        trocar_node.createObject('MechanicalObject', name='mechObject', template="Rigid3d", dx=-6.75, dy=2.75, dz=-9.5, rx=0, ry=0,
                                  rz=0, scale=scale,showObject="1")
        trocar_node.createObject('UniformMass', template="Rigid3d", name='mass', totalMass="1")

        # eyeball_node.createObject('RegularGridTopology', nx=2, ny=20, nz=10, xmin=-0.08, xmax=0.08, ymin=-0.08, ymax=0.08,
        #                      zmin=0, zmax=2)
        #
        # eyeball_node.createObject('TetrahedronFEMForceField', template='Vec3d', name='FEM_tissue',
        #                      method='polar', poissonRatio='0.495', youngModulus='0.0006', computeVonMisesStress='1',
        #                      showVonMisesStressPerNode='false', listening='1', updateStiffness='true',
        #                      showStressColorMap='blue 1 0 0 1  1 0.5 0.5 1', isToPrint='1')

        #
        #trocar_node.createObject('UncoupledConstraintCorrection')

        # fatNode.createObject('UniformMass', totalMass="0.5")
        # trocar_node.createObject('BoxROI', name='BoxROI', box=[-0.5, -0.5, 0, 0.5, 0.5, 2.5], drawBoxes=True,
        #                           doUpdate=False)
        # trocar_node.createObject('RestShapeSpringsForceField', points='@BoxROI.indices', stiffness='1e12')

        # Visual node
        trocarVisNode = trocar_node.createChild('Visual')
        trocarVisNode.createObject('MeshObjLoader', name='meshLoader', filename=meshFile_viz)
        trocarVisNode.createObject('OglModel', src='@meshLoader', color="0.85 0.4 0.1 1", name="Trocar", scale3d=[1, 1, 1])# 0.39 0.25 0       trocarVisNode.createObject('OglModel', src='@meshLoader', color="0.39 0.25 0", name="Trocar", scale3d=[1, 1, 1])#
        trocarVisNode.createObject('RigidMapping', name="Visual Mapping", output="@Trocar")

        # Collision


# ==========================
# FAT NODE
# ==========================

        # meshFile_col = "../../models/vessel_detection/vessel-600.obj"
        # meshFile_viz = "../../models/vessel_detection/vessel_1k7.obj"
        # meshFile_8w = "../../models/vessel_detection/vessel-80000.obj"
        # meshFile_tetra = "../../models/vessel_detection/vessel_1k7.msh"
        #
        # fatNode = self.rootNode.createChild("Vessel")
        # #fatNode.gravity = [0, -1, 0]
        #
        # fatNode.createObject('EulerImplicitSolver', rayleighStiffness=0.1, rayleighMass=0.1)
        # fatNode.createObject('CGLinearSolver', iterations=25, tolerance=1e-11, threshold=1e-18, printLog=False)
        #
        #
        #
        # #Force Model1: Grid
        #
        # fatNode.createObject('MeshObjLoader',name="meshLoader0",filename=meshFile_viz)
        # fatNode.createObject('MeshTopology',name="mesh",src="@meshLoader0")
        # fatNode.createObject('MechanicalObject', name='mechObject', template="Vec3d",dx=0, dy=0, dz=0, rx=0,ry=0, rz=0,scale=scale)
        # fatNode.createObject('UniformMass', template="Vec3d", name='mass', totalMass="1")
        # fatNode.createObject('RegularGridTopology', nx=2, ny=20, nz=10, xmin=-0.08, xmax=0.08, ymin=-0.08, ymax=0.08,
        #                      zmin=0, zmax=2)
        #
        # fatNode.createObject('TetrahedronFEMForceField', template='Vec3d', name='FEM_tissue',
        #                      method='polar', poissonRatio='0.495', youngModulus='0.0006', computeVonMisesStress='1',
        #                      showVonMisesStressPerNode='false', listening='1', updateStiffness='true',
        #                      showStressColorMap='blue 1 0 0 1  1 0.5 0.5 1', isToPrint='1')
        #
        # #
        # fatNode.createObject('UncoupledConstraintCorrection')
        #
        #
        # #fatNode.createObject('UniformMass', totalMass="0.5")
        # fatNode.createObject('BoxROI', name='BoxROI', box=[-0.5,-0.5,0,0.5,0.5,2.5], drawBoxes=True,
        #                doUpdate=False)
        # fatNode.createObject('RestShapeSpringsForceField', points='@BoxROI.indices', stiffness='1e12')
        #
        # # Visual node
        # fatVisNode = fatNode.createChild('Visual')
        # fatVisNode.createObject('MeshObjLoader', name='meshLoader', filename=meshFile_viz)
        # fatVisNode.createObject('OglModel', src='@meshLoader', color="0.7 0 0", name="Fat",scale3d=[1000,1000,1000])
        # fatVisNode.createObject('BarycentricMapping', name="Visual Mapping", output="@Fat")
        #
        #
        # # Collision node
        # fatColNode = fatNode.createChild('Collision')
        # fatColNode.createObject('MeshObjLoader', name="meshLoader", filename=meshFile_viz)
        # # fatColNode.createObject('OglModel',name='Visual', src='@meshLoader', color="#ecc854")
        # fatColNode.createObject('MeshTopology', src="@meshLoader")
        # fatColNode.createObject('MechanicalObject', src="@meshLoader", name="CollisionObject", template="Vec3d",
        #                         scale="1000")
        # fatColNode.createObject('TTriangleModel', template="Vec3d",contactStiffness="1")
        # fatColNode.createObject('TLineModel',contactStiffness="1")
        # fatColNode.createObject('TPointModel',contactStiffness="1")
        # fatColNode.createObject('BarycentricMapping', name="Mechanical Mapping")


# ==========================
# NEEDLE NODE
# ==========================

        meshFile_viz = "../../models/needle/needle_tapped_long_2k.obj"
        meshFile_col = "../../models/needle/needle_tapped_long_200.obj"
        # meshFile_viz = "../../models/needle/needle_tapped_total.obj"
        # meshFile_col = "../../models/needle/needle_tapped_onlypart.obj"
        #meshFile_vtu = "../models/needle/needle.vtk"

        scale = 10

        needleNode = self.rootNode.createChild("Needle")
        #needleNode.gravity = [0, 0, 0]

        needleNode.createObject('EulerImplicitSolver', name="ODE solver", rayleighStiffness="0.1", rayleighMass="0.1")
        needleNode.createObject('CGLinearSolver', name="linear solver", iterations="25", tolerance="1e-9",
                                threshold="1e-18")

        ### obj and triangular

        needleNode.createObject('MechanicalObject',template="Rigid3d", name="mechObject", translation="0 0 0",#"-4.51 -5.77 12.16","270 -120 90"
                                rotation="0 0 0",position="8.00797 0.474738 12.321 0.0225288 -0.476696 0.0373461 0.877985", showObject="1",showObjectScale="1")  #sphere1: 8.00797 0.474738 12.321 0.0225288 -0.476696 0.0373461 0.877985
                                #default: 9.56495 -0.117 16.5925 0 -0.5 0 0.866023
        needleNode.createObject('UniformMass', template="Rigid3d", name="mass", totalMass="2")
        #needleNode.createObject('UncoupledConstraintCorrection')


        #force field
        self.MechanicalState = needleNode.getObject('mechObject')
        # pose = self.MechanicalState.position[0][3:]
        # output_force = Force_transf(270, -45, 90, [-1, 0, 0])
        # print output_force
        #needleNode.createObject('ConstantForceField', name="CFF",indices=[0,1,2], totalForce=[0,-70.7,-70.7,0,0,0])


        # Visual node
        needleVisNode = needleNode.createChild("VisualModel")
        needleVisNode.createObject('MeshObjLoader', name='instrumentMeshLoader', filename=meshFile_viz)
        needleVisNode.createObject('OglModel', name="InstrumentVisualModel", src='@instrumentMeshLoader')#,translation="3.75 -0.32 2.56",
                                #rotation="300 -45 90")
        #needleVisNode.createObject('BarycentricMapping', name="Visual mapping", output="@InstrumentVisualModel")
        needleVisNode.createObject('RigidMapping', name="MM-VM mapping", input="@../mechObject",
                                   output="@InstrumentVisualModel")

        # # Collision node
        needleColNode = needleNode.createChild("CollisionModel")
        needleColNode.createObject('MeshObjLoader', filename=meshFile_col, name="loader")
        needleColNode.createObject('MeshTopology', src="@loader", name="InstrumentCollisionModel")
        needleColNode.createObject('MechanicalObject',src="@InstrumentCollisionModel",name="instrumentCollisionState")#,
                                   #dy=-2 * scale, scale3d=[scale, scale, scale])
        friction = "0.1"
        needleColNode.createObject('TTriangleModel', name="instrumentTrinagle", contactStiffness="500",
                                   contactFriction=friction)
        needleColNode.createObject('TLineModel', name="instrumentLine", contactStiffness="500",
                                   contactFriction=friction)
        needleColNode.createObject('TPointModel', name="instrumentPoint",
                                   contactStiffness="500", contactFriction=friction)

        needleColNode.createObject('RigidMapping', name="MM-CM mapping", input="@../mechObject",
                                   output="@instrumentCollisionState")

        self.Col = needleColNode.getObject('instrumentCollisionState')

        #needleColNode.createObject('BarycentricMapping')#,template=Vec3d,Vec3d, input="@../mechObject", output="@instrumentCollisionState")





        ## control node
        needleNode.createObject('BoxROI', name='BoxROI_needle', box=[0,0.5,1.5,1,5,5], drawBoxes=True,
                        doUpdate=False)
        needleNode.createObject('LinearMovementConstraint', template="Vec3d",
                                #indices="0",#"@BoxROI_needle.indices",
                                keyTimes=[0, 1.0,  2.0],
                                movements=[[0, -0.05, -0.05, 0, 0, 0],
                                           [0, -0.22, -0.22, 0, 0, 0],
                                           [0, -0.05, -0.05, 0, 0, 0]])

        # needleNode.createObject('ConstantForceField', indices="0",#"0 1 2",
        #                         # times=[0, 1.2, 1.8*5, 2.5*5],   # increse the second para then speed goes up
        #                         # forces=[[0, 0, 0, 0, 0, 0],
        #                         #            [0, -0.40*10, -0.40*10, 0, 0, 0],
        #                         #            [0, -0.42*10, -0.42*10, 0, 0, 0],
        #                         #            [0, -0.2, -0.2, 0, 0, 0]],
        #                         force="0 -100 -100")

        # Instantiate floor
        # floor = Floor(self.rootNode, name="Floor", translation=[0.0, 7.8, 0], uniformScale=6,
        #               isAStaticObject=True, color=[0.3, 0.3, 0.4])


        ##### add a sphere ###
        self.sphere1 = Sphere(self.rootNode, name="sphere1", translation=[-5.8, 10.38, -18.19],color = [0,0.7,0],uniformScale=self.ball_size,isAStaticObject=True)     #dx=-6.75, dy=2.75, dz=-9.5
        self.sphere1_oglmodel = self.sphere1.VisualModel.getObject('model')
        self.sphere1_oglmodel.listening = "True"

        ##### add a sphere ###
        self.sphere2 = Sphere(self.rootNode, name="sphere2", translation=[-12.15, -1.1, -18.65],color = [0,0.7,0],uniformScale=self.ball_size,isAStaticObject=True)
        self.sphere2_oglmodel = self.sphere2.VisualModel.getObject('model')
        self.sphere2_oglmodel.listening = "True"



###############################
########### GUI ###############
###############################
        guiNode = self.rootNode.createChild("GUI")
        guiNode.createObject('Visual3DText',name="Gang", text="Gang: 5",position="20 0 -10",scale="3",listening="True")
        guiNode.createObject('Visual3DText', name="Wounds", text="# of wounds: 0", position="20 0 -15", scale="3",
                             listening="True")
        guiNode.createObject('Visual3DText', name="Tasks", text="Tasks finished: 0", position="20 0 -20", scale="3",
                             listening="True")
        guiNode.createObject('Visual3DText', name="Time", text="Time: ", position="20 0 20", scale="3",
                             listening="True")
        guiNode.createObject('Visual3DText', name="Points", text="Points: TBD", position="20 0 -25", scale="3",
                             listening="True")
        self.gang_gui = guiNode.getObject('Gang')
        self.wounds_gui = guiNode.getObject('Wounds')
        self.task_gui = guiNode.getObject('Tasks')
        self.time_gui = guiNode.getObject('Time')
        self.points_gui = guiNode.getObject('Points')



        return 0


#####################################
    def onKeyPressed(self, k):

        # arr = conn.recv()
        # #t = time.time() * 1000
        # out_arr = preprocess(arr)
        #self.out_spacemouse()

        #print ord(k)
        # free_position is a scalar array : [tx,ty,tz,rx,ry,rz,rw]
        #free_position = self.MechanicalState.position
        magitude = self.rootNode.getObject('OglViewport').cameraPosition


        #calculate the rotation matrix
        # ls is basically equal to self.needle_pose
        # ls = self.quaternion_to_euler(free_position[0][3],free_position[0][4],free_position[0][5],free_position[0][6])
        #print ls
        #print self.needle_pose
        # self.rotation_matrix.rotation = ls
        # #print self.rotation_matrix.outT
        # rotation_matrix = [self.rotation_matrix.outT[0][0],self.rotation_matrix.outT[0][4],self.rotation_matrix.outT[0][8]]

        #self.look_for_tip()

        # define translation speed and rotation angles
        # speed = 0.1 * self.gang
        # angle = 1 * self.gang
        # set_rotation = [0,0,0]
        # set_translation = [0,0,0]
        mag_speed = 0.1

        ### translation ###
        # # UP key : front
        # if ord(k) == 19:
        #     set_translation = [speed,0,0]
        # # DOWN key : rear
        # if ord(k) == 21:
        #     set_translation = [-1*speed,0,0]
        # # LEFT key : left
        # if ord(k) == 18:
        #     set_translation = [0,speed,0]
        # # RIGHT key : right
        # if ord(k) == 20:
        #     set_translation = [0,-1*speed,0]
        # PAGEUP key : up
        # if ord(k) == 22:
        #     set_translation = [0,0,speed]
        # # PAGEDN key : down
        # if ord(k) == 23:
        #     set_translation = [0,0,-1*speed]

        ### press PgUp and PgDn to adjust the microscope magnification ###

        # PgDn key : scale down the magnification
        if ord(k) == 23:
            if magitude[0][2] <= 4.4:
                magitude[0][2] += mag_speed
                self.rootNode.getObject('OglViewport').cameraPosition = magitude
                #print "ja"
                print self.rootNode.getObject('OglViewport').cameraPosition[0][2]

            else:
                print "microscope is already the lowest magnification"

        # PgUp down : scale up the magnification
        if ord(k) == 22:
            if magitude[0][2] >= -7.89:
                #self.magnification -= mag_speed
                magitude[0][2] -= mag_speed
                self.rootNode.getObject('OglViewport').cameraPosition = magitude
                print self.rootNode.getObject('OglViewport').cameraPosition[0][2]
            else:
                print "microscope is already the biggest magnification"


        ### adjust the slide movement speed and rotation speed
        if ord(k) == 75:  # k
            if self.gang <=4:
                self.gang +=1
                #print "gang: "+str(self.gang)
                self.gang_gui.text = "Gang: " + str(self.gang)
        if ord(k) == 77:  # m
            if self.gang >=2:
                self.gang -= 1
                self.gang_gui.text = "Gang: " + str(self.gang)
                #print "gang: " + str(self.gang)

        ### rotation around RCM###


        # if ord(k) == 49:   #num:1
        #     self.angleX = self.angleX - angle
        #
        #     quat = self.euler_to_quaternion()
        #     free_position[0][3] = quat[0]
        #     free_position[0][4] = quat[1]
        #     free_position[0][5] = quat[2]
        #     free_position[0][6] = quat[3]
        #     # print self.MechanicalState.position
        #     self.MechanicalState.position = free_position
        #     return 0
        #
        # if ord(k) == 51:  #num:3
        #     self.angleX = self.angleX + angle
        #
        #     quat = self.euler_to_quaternion()
        #     free_position[0][3] = quat[0]
        #     free_position[0][4] = quat[1]
        #     free_position[0][5] = quat[2]
        #     free_position[0][6] = quat[3]
        #     # print self.MechanicalState.position
        #     self.MechanicalState.position = free_position
        #     return 0
        #     #set_rotation = [angle,0,0]
        #
        # if ord(k) == 52:
        #     set_rotation = [0,-1*angle,0]
        # # PAGEDN key : down
        # if ord(k) == 54:
        #     set_rotation = [0,angle,0]
        # if ord(k) == 55:
        #     set_rotation = [0,0,-1*angle]
        # # PAGEDN key : down
        # if ord(k) == 57:
        #     set_rotation = [0,0,angle]
        #
        #
        #
        # #print ord(k)
        # ### slide along the x-axis of needle
        # if ord(k) == 56:
        #     set_translation = [x*speed*-1 for x in rotation_matrix]
        #
        # if ord(k) == 50:
        #     set_translation = [x*speed for x in rotation_matrix]
        #
        # ### set rotation & trans into self.trans
        # self.needle_pose[0] += set_rotation[0]
        # self.needle_pose[1] += set_rotation[1]
        # self.needle_pose[2] += set_rotation[2]
        #
        # self.trans.rotation = set_rotation
        # self.trans.input_position = free_position
        # self.trans.translation = set_translation
        #
        # free_position = self.trans.output_position
        # self.MechanicalState.position = free_position



        return 0


    def onBeginAnimationStep(self,dt):

        arr = conn.recv()
        #t = time.time() * 1000
        out_arr = preprocess(arr)
        if abs(out_arr[0])>=0.1:
            self.out_spacemouse(out_arr)

        if self.task == [True,True]:
            print "both tasks are finished!"
            if self.finish == False:
                self.finish=True
                self.points = 100
                self.time = time.time() - self.time_start
                self.evaluate_points()
        else:
            self.check_task()
            self.detect_wounds()
            #self.sec += 1
            current_time = time.time() - self.time_start

            # if self.sec * self.rootNode.dt % 20 ==0:
            #     #print "ja"
            if int(current_time)<60:
                self.time_gui.text = "Time: " + str(int(current_time))+"''"
            else:
                minute = int(current_time)/60
                sec = int(current_time)%60
                self.time_gui.text = "Time: " + str(minute) + "'"+str(sec)+"''"

        return 0


    def quaternion_to_euler(self,x, y, z, w):
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        X = m.degrees(m.atan2(t0, t1))

        t2 = 2.0 * (w * y - z * x)
        t2 = 1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = m.degrees(m.asin(t2))

        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        Z = m.degrees(m.atan2(t3, t4))

        self.angleX = X
        self.angleY = Y
        self.angleZ = Z

        return [X, Y, Z]

    def euler_to_quaternion(self):
        roll = self.angleX * m.pi /180
        pitch = self.angleY * m.pi /180
        yaw = self.angleZ * m.pi /180
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]


    def look_for_tip(self):

        pose_ls = self.Col.position
        pose_array = np.array(pose_ls)
        indice = np.argmin(pose_array[:,2])
        #print indice

        return pose_array[indice]

    def check_tip_in_sphere(self):

        radius = self.ball_size
        insert_1 = False
        insert_2 = False
        # print radius
        center1 = np.array(self.sphere1.getObject('mstate').translation2[0])
        center2 = np.array(self.sphere2.getObject('mstate').translation2[0])
        # extract coor of needle tip
        location = self.look_for_tip()


        # print center
        dist1 = np.linalg.norm(center1-location)
        dist2 = np.linalg.norm(center2-location)

        # check if needle-tip inside the sphere
        if dist1<=radius:
            insert_1 = True
            print "the first vessel-inter is feeling hot now"
        else:
            insert_1 = False

        if dist2<=radius:
            insert_2 = True
            print "the second vessel-inter is feeling hot now"
        else:
            insert_2 = False

        return [insert_1,insert_2]


    def sphere_color(self,i):
        if i==0:
            self.sphere1.VisualModel.getObject(
                'model').material = "Default Diffuse 1 1 0 0 1 Ambient 1 0.2 0.2 0 1 Specular 0 1 1 0 1 Emissive 0 1 1 0 1 Shininess 0 45"
        if i==1:
            self.sphere2.VisualModel.getObject(
                'model').material = "Default Diffuse 1 1 0 0 1 Ambient 1 0.2 0.2 0 1 Specular 0 1 1 0 1 Emissive 0 1 1 0 1 Shininess 0 45"


    def detect_wounds(self):
        location = self.look_for_tip()
        radius = self.eyeball_radius
        center = np.array(self.eyeball_center)
        dist1 = np.linalg.norm(center - location)

        if dist1>radius:
            wound = True
            if self.wounds_locked == False:

                self.wounds_locked = True
                self.times_wound += 1
                self.wounds_gui.text= "# of Wounds: " + str(self.times_wound)

            if self.wounds_locked == True:
                pass
            #print "That hurts! dude"
        else:
            if self.wounds_locked == True:
                self.wounds_locked = False
            wound = False

        return None

    def check_task(self):

        check_tip = self.check_tip_in_sphere()

        for i in range(2):
            if not self.task[i]:# only check if task not completed
                if check_tip[i]:
                    self.count_sec[i] += 1
                else:
                    self.count_sec[i] = 0
                if self.count_sec[i] >= 20:    ### complete if stay there for 50steps
                    self.sphere_color(i)
                    self.task[i] = True
                    self.task_gui.text= "Tasks finished: " + str(self.task.count(True))

    def evaluate_points(self):

        ### Performance1: time
        ### if time-cost is lower than average_time, then they would get 100% points at time-performance
        ###       Otherweise, minus points = time_diff * 0.5, at most minus-points is 50
        time_diff = self.time - self.average_time
        if time_diff > 0:
            self.points -= time_diff * 0.5
            self.points = max(self.points,50)

        ### Performance2: wound
        ### if needle insert the eyeball during the operation, then it will be penaltied 10 points for each time.
        self.points = self.points - self.times_wound*10
        self.points_gui.text = "Points: "+str(self.points)
        self.finish = True

        ### Performance3: TBD
        return self.time, self.times_wound, self.points

    def out_spacemouse(self,out_arr):
        # arr = conn.recv()
        # #t = time.time() * 1000
        # out_arr = preprocess(arr)

        print out_arr
        free_position = self.MechanicalState.position
        ls = self.quaternion_to_euler(free_position[0][3], free_position[0][4], free_position[0][5],
                                      free_position[0][6])
        # print ls
        # print self.needle_pose
        self.rotation_matrix.rotation = ls
        # print self.rotation_matrix.outT
        rotation_matrix = [self.rotation_matrix.outT[0][0], self.rotation_matrix.outT[0][4],
                           self.rotation_matrix.outT[0][8]]

        # define translation speed and rotation angles
        speed = 0.1 * self.gang
        angle = 1 * self.gang
        set_rotation = [0,0,0]
        set_translation = [0,0,0]
        #mag_speed = 0.1

        ### rotation around RCM###
        #if ord(k) == 49:   #num:1
        if out_arr[1]==5 and out_arr[0]<0:
            print "1"
            self.angleX = self.angleX - angle

            quat = self.euler_to_quaternion()
            free_position[0][3] = quat[0]
            free_position[0][4] = quat[1]
            free_position[0][5] = quat[2]
            free_position[0][6] = quat[3]
            # print self.MechanicalState.position
            self.MechanicalState.position = free_position
            return 0

        if out_arr[1]==5 and out_arr[0]>0:
            print "3"
            self.angleX = self.angleX + angle

            quat = self.euler_to_quaternion()
            free_position[0][3] = quat[0]
            free_position[0][4] = quat[1]
            free_position[0][5] = quat[2]
            free_position[0][6] = quat[3]
            # print self.MechanicalState.position
            self.MechanicalState.position = free_position
            return 0
            #set_rotation = [angle,0,0]

        if out_arr[1]==3 and out_arr[0]<0:
            set_rotation = [0,-1*angle,0]
        # PAGEDN key : down
        if out_arr[1]==3 and out_arr[0]>0:
            set_rotation = [0,angle,0]
        if out_arr[1]==4 and out_arr[0]<0:
            set_rotation = [0,0,-1*angle]
        # PAGEDN key : down
        if out_arr[1]==4 and out_arr[0]>0:
            set_rotation = [0,0,angle]

        #print ord(k)
        ### slide along the x-axis of needle
        if out_arr[1]==2 and out_arr[0]<0:
            set_translation = [x*speed*-1 for x in rotation_matrix]

        if out_arr[1]==2 and out_arr[0]>0:
            set_translation = [x*speed for x in rotation_matrix]

        ### set rotation & trans into self.trans
        self.needle_pose[0] += set_rotation[0]
        self.needle_pose[1] += set_rotation[1]
        self.needle_pose[2] += set_rotation[2]

        self.trans.rotation = set_rotation
        self.trans.input_position = free_position
        self.trans.translation = set_translation

        free_position = self.trans.output_position
        self.MechanicalState.position = free_position

        return 0



def createScene(rootNode):
    obj = MainScene(rootNode)
    obj.createGraph(rootNode)


