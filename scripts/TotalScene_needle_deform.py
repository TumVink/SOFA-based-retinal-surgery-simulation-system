import Sofa
import os
from stlib.physics.rigid import Floor
import numpy as np
import math as m

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

class MainScene(Sofa.PythonScriptController):

    def createGraph(self, node):
        dt = 0.02  # In second
        displayFlags = []

        self.rootNode = node.getRoot()
        self.rootNode.dt = dt
        self.rootNode.gravity = [0, 0, 0]

        self.rootNode.createObject('RequiredPlugin', name='SofaMiscCollision')
        self.rootNode.createObject('RequiredPlugin', name='SofaPython')
        self.rootNode.createObject('RequiredPlugin', name='CImgPlugin')
        #self.rootNode.createObject('RequiredPlugin', name='SofaOpenglVisual')

        self.rootNode.createObject('VisualStyle', displayFlags=displayFlags)
        self.rootNode.createObject('FreeMotionAnimationLoop')

        self.rootNode.createObject('GenericConstraintSolver', maxIterations=1000, tolerance=1e-07)
        self.rootNode.createObject('DefaultPipeline', verbose='0', depth="6", draw='1')
        self.rootNode.createObject('BruteForceDetection', name='N2')
        # self.rootNode.createObject('DiscreteIntersection', name='Intersection')
        self.rootNode.createObject("LocalMinDistance", name="Intersection", alarmDistance="0.2", contactDistance="0.012", useLMDFilters="0")
        #0.1/0.0122   --- young moodle 200   basic
        #0.2/0.012    --- young module 100 more deformation

        self.rootNode.createObject('DefaultContactManager', name="Response", response="FrictionContact",
                                   responseParams='mu=0.8')

        # ==========================
        # FAT NODE
        # ==========================

        meshFile_col = "../models/vessel/vessel_2d/vessel_300.obj"
        meshFile_viz = "../models/vessel/vessel_2d/vessel_8000.obj"

        fatNode = self.rootNode.createChild("Vessel")
        #fatNode.gravity = [0, -1, 0]

        fatNode.createObject('EulerImplicitSolver', rayleighStiffness=0.1, rayleighMass=0.1)
        fatNode.createObject('CGLinearSolver', iterations=25, tolerance=1e-11, threshold=1e-11, printLog=False)
        fatNode.createObject('MechanicalObject', name='mechObject', template="Vec3d",dx=0, dy=0, dz=0, rx=0,ry=0, rz=0)
        fatNode.createObject('UniformMass', template="Vec3d", name='mass', totalMass="0.1")

        fatNode.createObject('RegularGridTopology', nx=2, ny=2, nz=10, xmin=0, xmax=1, ymin=0, ymax=0.25,
                             zmin=0, zmax=2.5)

        fatNode.createObject('TetrahedronFEMForceField', template='Vec3d', name='FEM_tissue',
                             method='polar', poissonRatio='0.495', youngModulus='0.0006', computeVonMisesStress='1',
                             showVonMisesStressPerNode='false', listening='1', updateStiffness='true',
                             showStressColorMap='blue 1 0 0 1  1 0.5 0.5 1', isToPrint='1')
        fatNode.createObject('UncoupledConstraintCorrection')


        fatNode.createObject('BoxROI', name='BoxROI', box=[0,0,0,1,0.5,2.5], drawBoxes=True,
                       doUpdate=False)
        fatNode.createObject('RestShapeSpringsForceField', points='@BoxROI.indices', stiffness='1e5')  #stiffness less then deformation "looks like" to be smaller

        # Visual node
        fatVisNode = fatNode.createChild('Visual')
        fatVisNode.createObject('MeshObjLoader', name='meshLoader', filename=meshFile_viz)
        fatVisNode.createObject('OglModel', src='@meshLoader', color="0.7 0 0", name="Fat")
        fatVisNode.createObject('BarycentricMapping', name="Visual Mapping", output="@Fat")

        # Collision node
        fatColNode = fatNode.createChild('Collision')
        fatColNode.createObject('MeshObjLoader', name="meshLoader", filename=meshFile_col)
        # fatColNode.createObject('OglModel',name='Visual', src='@meshLoader', color="#ecc854")
        fatColNode.createObject('MeshTopology', src="@meshLoader")
        fatColNode.createObject('MechanicalObject', src="@meshLoader", name="CollisionObject", template="Vec3d",
                                scale="1.0")
        fatColNode.createObject('TTriangleModel', template="Vec3d",contactStiffness="10")
        fatColNode.createObject('TLineModel',contactStiffness="10")
        fatColNode.createObject('TPointModel',contactStiffness="10")
        fatColNode.createObject('BarycentricMapping', name="Mechanical Mapping")




        # ==========================
        # NEEDLE NODE
        # ==========================

        meshFile_viz = "../models/needle/needle_tapped_total.obj"
        meshFile_col = "../models/needle/needle_tapped_onlypart_400.obj"
        meshFile_vtu = "../models/needle/needle.vtk"

        #scale = 10

        needleNode = self.rootNode.createChild("Needle")
        #needleNode.gravity = [0, 0, 0]

        needleNode.createObject('EulerImplicitSolver', name="ODE solver", rayleighStiffness="0.1", rayleighMass="0.1")
        needleNode.createObject('CGLinearSolver', name="linear solver", iterations="25", tolerance="1e-9",
                                threshold="1e-50")

        ### obj and triangular
        needleNode.createObject('MeshObjLoader',name="meshLoader0",filename=meshFile_col)
        needleNode.createObject('MeshTopology',name="mesh",src="@meshLoader0")
        needleNode.createObject('MechanicalObject',template="Vec3d", name="mechObject", translation="3.75 -0.32 2.56",
                                rotation="300 -45 90", showObject="1")
        needleNode.createObject('TriangleFEMForceField', template="Vec3d", name='FEM_needle',
                                                         poissonRatio='0.265', youngModulus='190',method="large")
        needleNode.createObject('UniformMass', template="Vec3d", name="mass", totalMass="0.5")




        ## control node
        needleNode.createObject('BoxROI', name='BoxROI_needle', box=[0,0.2,1.5,1,5,5], drawBoxes=True,
                       doUpdate=False)
        # needleNode.createObject('LinearMovementConstraint', template="Vec3d",
        #                         indices="@BoxROI_needle.indices",#"0 1 2",
        #                         keyTimes=[0, 1.2, 1.8, 2.5],   # increse the second para then speed goes up
        #                         movements=[[0, 0, 0, 0, 0, 0],
        #                                    [0, -0.40, -0.40, 0, 0, 0],
        #                                    [0, -0.42, -0.42, 0, 0, 0],
        #                                    [0, -0.2, -0.2, 0, 0, 0]])


        needleNode.createObject('ConstantForceField', indices="@BoxROI_needle.indices",#"0 1 2",
                                # times=[0, 1.2, 1.8*5, 2.5*5],   # increse the second para then speed goes up
                                # forces=[[0, 0, 0, 0, 0, 0],
                                #            [0, -0.40*10, -0.40*10, 0, 0, 0],
                                #            [0, -0.42*10, -0.42*10, 0, 0, 0],
                                #            [0, -0.2, -0.2, 0, 0, 0]],
                                totalForce="0 -100 -100")


        #force field
        self.MechanicalState = needleNode.getObject('mechObject')
        # pose = self.MechanicalState.position[0][3:]
        # output_force = Force_transf(300, -45, 90, [-1, 0, 0])
        # print output_force
        #needleNode.createObject('ConstantForceField', name="CFF",indices=[0,1,2], totalForce=[0,-70.7,-70.7,0,0,0])


        # Visual node
        needleVisNode = needleNode.createChild("VisualModel")
        needleVisNode.createObject('MeshObjLoader', name='instrumentMeshLoader', filename=meshFile_col)
        needleVisNode.createObject('OglModel', name="InstrumentVisualModel", src='@instrumentMeshLoader',translation="3.75 -0.32 2.56",
                                rotation="300 -45 90")
        needleVisNode.createObject('BarycentricMapping', name="Visual mapping", output="@InstrumentVisualModel")
        #needleVisNode.createObject('RigidMapping', template="Rigid3d,Vec3d")

        # # Collision node
        needleColNode = needleNode.createChild("CollisionModel")
        #vtk and tetra
        # needleColNode.createObject('TriangleSetTopologyContainer', name="TriContainer")
        # needleColNode.createObject('TriangleSetTopologyModifier', name="Modifier")
        # needleColNode.createObject('TriangleSetTopologyAlgorithms', name="TopoAlgo")
        # needleColNode.createObject('TriangleSetGeometryAlgorithms', name="GeomAlgo")
        # needleColNode.createObject('Tetra2TriangleTopologicalMapping', name="mapping",input="@../volume",output="@TriContainer",flipNormals="true")


        needleColNode.createObject('MeshObjLoader', filename=meshFile_col, name="loader")
        needleColNode.createObject('MeshTopology', src="@loader", name="InstrumentCollisionModel")
        needleColNode.createObject('MechanicalObject',template="Vec3d",name="colNeedle",translation="3.75 -0.32 2.56",
                                rotation="300 -45 90")#,
                                   #dy=-2 * scale, scale3d=[scale, scale, scale])

        needleColNode.createObject('Triangle', name="instrumentTrinagle", contactStiffness="500",
                                   contactFriction="0.01")
        needleColNode.createObject('Line', name="instrumentLine", contactStiffness="500",
                                   contactFriction="0.01")
        needleColNode.createObject('Point', name="instrumentPoint",
                                   contactStiffness="500", contactFriction="0.01")

        #needleColNode.createObject('TriangleMeshModel', moving="1",simulated="1",contactStiffness="100000000")

        #needleColNode.createObject('RigidMapping', template= "Rigid3d,Vec3d")
        needleColNode.createObject('BarycentricMapping')#,template=Vec3d,Vec3d, input="@../mechObject", output="@instrumentCollisionState")




        # ## control node
        # needleNode.createObject('BoxROI', name='BoxROI_needle', box=[0,0.2,1.5,1,5,5], drawBoxes=True,
        #                doUpdate=True)
        # # needleNode.createObject('LinearMovementConstraint', template="Vec3d",
        # #                         indices="@BoxROI_needle.indices",#"0 1 2",
        # #                         keyTimes=[0, 1.2, 1.8, 2.5],   # increse the second para then speed goes up
        # #                         movements=[[0, 0, 0, 0, 0, 0],
        # #                                    [0, -0.40, -0.40, 0, 0, 0],
        # #                                    [0, -0.42, -0.42, 0, 0, 0],
        # #                                    [0, -0.2, -0.2, 0, 0, 0]])
        #
        #
        # needleNode.createObject('ConstantForceField', indices="@BoxROI_needle.indices",#"0 1 2",
        #                         # times=[0, 1.2, 1.8*5, 2.5*5],   # increse the second para then speed goes up
        #                         # forces=[[0, 0, 0, 0, 0, 0],
        #                         #            [0, -0.40*10, -0.40*10, 0, 0, 0],
        #                         #            [0, -0.42*10, -0.42*10, 0, 0, 0],
        #                         #            [0, -0.2, -0.2, 0, 0, 0]],
        #                         force="0 -100 -100")

        # Instantiate floor
        # floor = Floor(self.rootNode, name="Floor", translation=[0.0, 0.3, 0], uniformScale=0.1,
        #               isAStaticObject=True, color=[0.3, 0.3, 0.4])

        return 0


    def onKeyPressed(self, k):

        # free_position is a scalar array : [tx,ty,tz,rx,ry,rz,rw]
        free_position = self.MechanicalState.position

        # translation speed
        speed = 0.05

        # UP key : front
        if ord(k) == 19:
            free_position[0][2] += speed
        # DOWN key : rear
        if ord(k) == 21:
            free_position[0][2] -= speed
        # LEFT key : left
        if ord(k) == 18:
            free_position[0][0] -= speed
        # RIGHT key : right
        if ord(k) == 20:
            free_position[0][0] += speed
        # PAGEUP key : up
        if ord(k) == 22:
            free_position[0][1] -= speed
        # PAGEDN key : down
        if ord(k) == 23:
            free_position[0][1] += speed

        self.MechanicalState.position = free_position
        #print self.MechanicalState.position
        return 0


def createScene(rootNode):
    obj = MainScene(rootNode)
    obj.createGraph(rootNode)

    #todo: 1. 30deg ~ 45 deg   done
    #      2. force detection
    #      3. deform of needle
    #           do we really need topo for collision?
    #           stiffness of steel is 50~60
    #           young modulus: 190 gpa
    #           poisson ration: 0.265
