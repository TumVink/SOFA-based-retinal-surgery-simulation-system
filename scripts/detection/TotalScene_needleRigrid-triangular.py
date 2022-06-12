import Sofa
import os
from stlib.physics.rigid import Floor
import numpy as np
import math as m
import pickle

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


class MainScene(Sofa.PythonScriptController):

    def createGraph(self, node):
        dt = 0.001*5  # In second
        displayFlags = []

        self.rootNode = node.getRoot()
        self.rootNode.dt = dt
        self.rootNode.gravity = [0, 0, 0]
        self.step = 0
        self.force_field = "tetra"

        self.rootNode.createObject('RequiredPlugin', name='SofaMiscCollision')
        self.rootNode.createObject('RequiredPlugin', name='SofaPython')
        self.rootNode.createObject('RequiredPlugin', name='CImgPlugin')
        #self.rootNode.createObject('RequiredPlugin', name='SofaOpenglVisual')

        self.rootNode.createObject('VisualStyle', displayFlags=displayFlags)
        self.rootNode.createObject('FreeMotionAnimationLoop')

        self.rootNode.createObject('GenericConstraintSolver', maxIterations=1000, tolerance=1e-07, computeConstraintForces=True)
        self.rootNode.createObject('DefaultPipeline', verbose='0', depth="6", draw='1')
        self.rootNode.createObject('BruteForceDetection', name='N2')
        # self.rootNode.createObject('DiscreteIntersection', name='Intersection')
        self.rootNode.createObject("LocalMinDistance", name="Intersection", alarmDistance="0.2", contactDistance="0.012", useLMDFilters="0")
        #0.1/0.0122   --- young moodle 200   basic
        #0.2/0.012    --- young module 100 more deformation

        self.rootNode.createObject('DefaultContactManager', name="Response", response="FrictionContactConstraint",
                                   responseParams='mu=0.8')


        #for force detection
        self.angle = 45 # in degree
        self.file_name = os.getcwd()+'\\exps\\trian\\data_'+str(self.angle)+'.pickle'
        self.f_list=[]
        #print self.file_name



        # ==========================
        # FAT NODE
        # ==========================

        meshFile_col = "../../models/vessel_detection/vessel-600.obj"
        meshFile_viz = "../../models/vessel_detection/vessel_1k7.obj"
        meshFile_8w = "../../models/vessel_detection/vessel-80000.obj"
        meshFile_tetra = "../../models/vessel_detection/vessel_1k7.msh"

        if self.force_field == "tetra":
            scale = 1
        else:
            scale=1000
        fatNode = self.rootNode.createChild("Vessel")
        #fatNode.gravity = [0, -1, 0]

        fatNode.createObject('EulerImplicitSolver', rayleighStiffness=0.1, rayleighMass=0.1)
        fatNode.createObject('CGLinearSolver', iterations=25, tolerance=1e-11, threshold=1e-18, printLog=False)



        #Force Model1: Grid

        # fatNode.createObject('MeshObjLoader',name="meshLoader0",filename=meshFile_viz)
        # fatNode.createObject('MeshTopology',name="mesh",src="@meshLoader0")
        # fatNode.createObject('MechanicalObject', name='mechObject', template="Vec3d",dx=0, dy=0, dz=0, rx=0,ry=0, rz=0,scale=scale)
        # fatNode.createObject('UniformMass', template="Vec3d", name='mass', totalMass="1")
        # fatNode.createObject('RegularGridTopology', nx=3, ny=3, nz=15, xmin=-0.1, xmax=0.1, ymin=-0.1, ymax=0.1,
        #                      zmin=0, zmax=2)
        #
        # fatNode.createObject('TetrahedronFEMForceField', template='Vec3d', name='FEM_tissue',
        #                      method='polar', poissonRatio='0.495', youngModulus='0.0006', computeVonMisesStress='1',
        #                      showVonMisesStressPerNode='false', listening='1', updateStiffness='true',
        #                      showStressColorMap='blue 1 0 0 1  1 0.5 0.5 1', isToPrint='1')


        # Force Model2: TriangleFEM

        fatNode.createObject('MeshObjLoader',name="meshLoader0",filename=meshFile_viz)
        fatNode.createObject('MeshTopology',name="mesh",src="@meshLoader0")
        fatNode.createObject('MechanicalObject', name='mechObject', template="Vec3d",dx=0, dy=0, dz=0, rx=0,ry=0, rz=0,scale="1000")
        fatNode.createObject('UniformMass', template="Vec3d", name='mass', totalMass="1")
        fatNode.createObject('TriangleFEMForceField', template="Vec3d", name='FEM_tissue',
                                                         poissonRatio='0.495', youngModulus='0.006',method="large")

        fatNode.createObject('UncoupledConstraintCorrection')

        # Force Model3: vtk -> Tetrahedron
        # fatNode.createObject('MeshVTKLoader',name="meshLoader0",filename=meshFile_tetra)
        #
        # #fatNode.createObject('MeshTopology', name="mesh", src="@meshLoader0")
        # fatNode.createObject('TetrahedronSetTopologyContainer',name="volume",src="@meshLoader0")
        # fatNode.createObject('MechanicalObject', name='mechObject', template="Vec3d", dx=0, dy=0, dz=0, rx=0, ry=0,
        #                      rz=0, scale="1")
        # fatNode.createObject('TetrahedronSetGeometryAlgorithms')
        # #fatNode.createObject('TetrahedronSetTopologyModifier', name='modifier')
        #
        # fatNode.createObject('TetrahedronFEMForceField', template='Vec3d',name='FEM_tissue',method="large",  poissonRatio='0.495', youngModulus='0.0006')
        #
        # fatNode.createObject('UniformMass', totalMass="0.5")

        #fatNode.createObject('UniformMass', totalMass="0.5")
        fatNode.createObject('BoxROI', name='BoxROI', box=[-0.5,-0.5,0,0.5,0.5,2.5], drawBoxes=True,
                       doUpdate=False)
        fatNode.createObject('RestShapeSpringsForceField', points='@BoxROI.indices', stiffness='1')

        # Visual node
        fatVisNode = fatNode.createChild('Visual')
        fatVisNode.createObject('MeshObjLoader', name='meshLoader', filename=meshFile_viz)
        fatVisNode.createObject('OglModel', src='@meshLoader', color="0.7 0 0", name="Fat",scale3d=[1000,1000,1000])
        fatVisNode.createObject('BarycentricMapping', name="Visual Mapping", output="@Fat")

        # Visual node for tetra
        # fatVisNode = fatNode.createChild('Visual')
        # fatVisNode.createObject('OglModel', src='@../meshLoader0', color="0.7 0 0", name="Fat",scale3d="1000 1000 1000")
        # fatVisNode.createObject('IdentityMapping', name="Visual Mapping", mapForces="true", mapMasses="true")

        # Collision node
        fatColNode = fatNode.createChild('Collision')
        fatColNode.createObject('MeshObjLoader', name="meshLoader", filename=meshFile_viz)
        # fatColNode.createObject('OglModel',name='Visual', src='@meshLoader', color="#ecc854")
        fatColNode.createObject('MeshTopology', src="@meshLoader")
        fatColNode.createObject('MechanicalObject', src="@meshLoader", name="CollisionObject", template="Vec3d",
                                scale="1000")
        fatColNode.createObject('TTriangleModel', template="Vec3d",contactStiffness="1")
        fatColNode.createObject('TLineModel',contactStiffness="1")
        fatColNode.createObject('TPointModel',contactStiffness="1")
        fatColNode.createObject('BarycentricMapping', name="Mechanical Mapping")


        # Tetra collision
        # fatColNode = fatNode.createChild('Collision')
        # fatColNode.createObject('MeshTopology', src="@../meshLoader0")
        # # fatColNode.createObject('TriangleSetTopologyContainer', name="TriContainer")
        # # # fatColNode.createObject('OglModel',name='Visual', src='@meshLoader', color="#ecc854")
        # # fatColNode.createObject('TriangleSetTopologyModifier', name="Modifier")
        # # fatColNode.createObject('TriangleSetTopologyAlgorithms', name="TopoAlgo")
        # # fatColNode.createObject('TriangleSetGeometryAlgorithms', name="GeomAlgo")
        # #fatColNode.createObject('Tetra2TriangleTopologicalMapping', name="mapping",input="@../volume",output="@TriContainer",flipNormals="true")
        # fatColNode.createObject('MechanicalObject', scale="1000")
        #
        # fatColNode.createObject('TTriangleModel', contactStiffness="5")
        # fatColNode.createObject('TLineModel',contactStiffness="5")
        # fatColNode.createObject('TPointModel',contactStiffness="5")
        #
        # fatColNode.createObject('IdentityMapping', name="Mechanical Mapping", mapForces="true", mapMasses="true")




        # ==========================
        # NEEDLE NODE
        # ==========================

        meshFile_viz = "../../models/needle/needle_tapped_total.obj"
        meshFile_col = "../../models/needle/needle_tapped_onlypart_400.obj"
        #meshFile_vtu = "../models/needle/needle.vtk"

        scale = 10

        needleNode = self.rootNode.createChild("Needle")
        #needleNode.gravity = [0, 0, 0]

        needleNode.createObject('EulerImplicitSolver', name="ODE solver", rayleighStiffness="0.1", rayleighMass="0.1")
        needleNode.createObject('CGLinearSolver', name="linear solver", iterations="25", tolerance="1e-9",
                                threshold="1e-18")

        ### obj and triangular
        # needleNode.createObject('MeshObjLoader',name="meshLoader0",filename=meshFile_col)
        # needleNode.createObject('MeshTopology',name="mesh",src="@meshLoader0")
        needleNode.createObject('MechanicalObject',template="Rigid3d", name="mechObject", translation="2.49 -1.77 3.66",
                                rotation="270 -45 90", showObject="1",showObjectScale="1")
        needleNode.createObject('UniformMass', template="Rigid3d", name="mass", totalMass="2")
        # needleNode.createObject('TriangleFEMForceField', template="Vec3d", name='FEM_needle',
        #                                                  poissonRatio='0.265+0.23', youngModulus='190*0.000001')
        needleNode.createObject('UncoupledConstraintCorrection')


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
        #vtk and tetra
        # needleColNode.createObject('TriangleSetTopologyContainer', name="TriContainer")
        # needleColNode.createObject('TriangleSetTopologyModifier', name="Modifier")
        # needleColNode.createObject('TriangleSetTopologyAlgorithms', name="TopoAlgo")
        # needleColNode.createObject('TriangleSetGeometryAlgorithms', name="GeomAlgo")
        # needleColNode.createObject('Tetra2TriangleTopologicalMapping', name="mapping",input="@../volume",output="@TriContainer",flipNormals="true")


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

        #needleColNode.createObject('TriangleMeshModel', moving="1",simulated="1",contactStiffness="100000000")

        needleColNode.createObject('RigidMapping', name="MM-CM mapping", input="@../mechObject",
                                   output="@instrumentCollisionState")
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

    def onBeginAnimationStep(self,dt):
        csf = self.rootNode.GenericConstraintSolver.constraintForces #csf for constrained force
        #csf.length()
        l = len(csf)
        # if l != 0:
        #     print csf[0]
        n_force = l/3  #1 or 2
        #print n_force
        f_step = [0,0,0]
        #f_list = []
        #print csf
        for i in range(n_force): # x0 y1 z2 x3 y4 z5
            f_step[0] += csf[i*3][0]
            f_step[1] += csf[i*3+1][0]
            f_step[2] += csf[i*3+2][0]
        self.f_list.append(f_step)

        #print self.f_list
        #print dt
        #write_txt(f_ls)
        self.step +=1
        print self.step
        if self.step % 50 == 0:
            #print "yes"
            with open(self.file_name, 'wb') as handle:
                pickle.dump(self.f_list, handle, protocol=pickle.HIGHEST_PROTOCOL)
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
    #           stiffness: 0.265
