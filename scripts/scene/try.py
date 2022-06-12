import Sofa
import os
#from stlib.physics.rigid import Floor
import numpy as np
import math as m
from stlib.physics.rigid import Cube, Sphere, Floor
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

        self.rootNode.createObject('OglViewport', screenPosition="0 0", screenSize="250 250", cameraPosition="6.25 2.75 4.3",  #dx=6.25, dy=2.75, dz=-9.5
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
        self.ball_size = 0.3

        #print self.file_name

# ==========================
# Eyeball NODE
# ==========================
        meshFile_col = "../../models/eyeball/eyeball_bot_part.obj"
        meshFile_viz = "../../models/eyeball/eyeball_bot_part.obj"

    #overall def
        eyeball_node= self.rootNode.createChild("eyeball_bot")
        scale=1
        #fatNode.gravity = [0, -1, 0]

        eyeball_node.createObject('EulerImplicitSolver', rayleighStiffness=0.1, rayleighMass=0.1)
        eyeball_node.createObject('CGLinearSolver', iterations=25, tolerance=1e-11, threshold=1e-18, printLog=False)

        eyeball_node.createObject('MeshObjLoader',name="meshLoader0",filename=meshFile_col)
        #eyeball_node.createObject('MeshTopology',name="mesh",src="@meshLoader0")
        eyeball_node.createObject('MechanicalObject', name='mechObjectEyeball', template="Rigid3d",dx=-6.75, dy=2.75, dz=-9.5, rx=0,ry=0, rz=0,scale=scale,showObject="1")
        eyeball_node.createObject('UniformMass', template="Vec3d", name='mass', totalMass="100")

        # eyeball_node.createObject('RegularGridTopology', nx=2, ny=20, nz=10, xmin=-0.08, xmax=0.08, ymin=-0.08, ymax=0.08,
        #                      zmin=0, zmax=2)
        #
        # eyeball_node.createObject('TetrahedronFEMForceField', template='Vec3d', name='FEM_tissue',
        #                      method='polar', poissonRatio='0.495', youngModulus='0.0006', computeVonMisesStress='1',
        #                      showVonMisesStressPerNode='false', listening='1', updateStiffness='true',
        #                      showStressColorMap='blue 1 0 0 1  1 0.5 0.5 1', isToPrint='1')

        #
        #eyeball_node.createObject('UncoupledConstraintCorrection')
        #self.eyeMecha = eyeball_node.getObject('mechObjectEyeball')

        #fatNode.createObject('UniformMass', totalMass="0.5")
        # eyeball_node.createObject('BoxROI', name='BoxROI', box=[-0.5,-0.5,0,0.5,0.5,2.5], drawBoxes=True,
        #                doUpdate=False)
        # eyeball_node.createObject('RestShapeSpringsForceField', points='@BoxROI.indices', stiffness='1e12')

        # Visual node
        eyeVisNode = eyeball_node.createChild('Visual')
        eyeVisNode.createObject('MeshObjLoader', name='meshLoader', filename=meshFile_viz)
        eyeVisNode.createObject('OglModel', src='@meshLoader', color="0.7 0.7 0.7 1", name="Eye",scale3d=[1,1,1], texturename="../../models/texture/retinal_try.png",scaleTex = [-1,-1])
        eyeVisNode.createObject('RigidMapping', name="Visual Mapping", output="@Eye")

# # ==========================
# # Eyeball_top NODE
# # ==========================
#         meshFile_col = "../../models/eyeball/eyeball_top_part.obj"
#         meshFile_viz = "../../models/eyeball/eyeball_top_part.obj"
#
#         # overall def
#         eyeball_node = self.rootNode.createChild("eyeball_top")
#         scale = 1
#         # fatNode.gravity = [0, -1, 0]
#
#         eyeball_node.createObject('EulerImplicitSolver', rayleighStiffness=0.1, rayleighMass=0.1)
#         eyeball_node.createObject('CGLinearSolver', iterations=25, tolerance=1e-11, threshold=1e-18, printLog=False)
#
#         eyeball_node.createObject('MeshObjLoader', name="meshLoader0", filename=meshFile_col)
#         # eyeball_node.createObject('MeshTopology',name="mesh",src="@meshLoader0")
#         eyeball_node.createObject('MechanicalObject', name='mechObjectEyeball', template="Rigid3d", dx=6.25, dy=2.75,
#                                   dz=-9.5, rx=0, ry=0, rz=0, scale=scale, showObject="1")
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
#         # eyeball_node.createObject('UncoupledConstraintCorrection')
#         # self.eyeMecha = eyeball_node.getObject('mechObjectEyeball')
#
#         # fatNode.createObject('UniformMass', totalMass="0.5")
#         # eyeball_node.createObject('BoxROI', name='BoxROI', box=[-0.5,-0.5,0,0.5,0.5,2.5], drawBoxes=True,
#         #                doUpdate=False)
#         # eyeball_node.createObject('RestShapeSpringsForceField', points='@BoxROI.indices', stiffness='1e12')
#
#         # Visual node
#         eyeVisNode = eyeball_node.createChild('Visual')
#         eyeVisNode.createObject('MeshObjLoader', name='meshLoader', filename=meshFile_viz)
#         eyeVisNode.createObject('OglModel', src='@meshLoader', color="0.7 0.7 0.7 1", name="Eye", scale3d=[1, 1, 1],
#                                 texturename="../../models/texture/Eye_D.jpg", listening="True")
#         eyeVisNode.createObject('RigidMapping', name="Visual Mapping", output="@Eye")

        ### vessel  ###
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
        vessel_node.createObject('MechanicalObject', name='mechObject', template="Rigid3d", dx=-6.75, dy=2.75, dz=-9.5,
                                 rx=0,
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


        ##### add a sphere ###

        ##### add a sphere ###
        self.sphere1 = Sphere(self.rootNode, name="sphere1", translation=[-5.8, 10.38, -18.19],color = [0,0.7,0],uniformScale=self.ball_size)     #dx=-6.75, dy=2.75, dz=-9.5
        self.sphere1_oglmodel = self.sphere1.VisualModel.getObject('model')
        self.sphere1_oglmodel.listening = "True"

        ##### add a sphere ###
        self.sphere2 = Sphere(self.rootNode, name="sphere2", translation=[-12.15, -1.1, -18.65],color = [0,0.7,0],uniformScale=self.ball_size)
        self.sphere2_oglmodel = self.sphere2.VisualModel.getObject('model')
        self.sphere2_oglmodel.listening = "True"

    def onKeyPressed(self, k):

        # free_position is a scalar array : [tx,ty,tz,rx,ry,rz,rw]
        free_position = self.sphere2.getObject('mstate').position
        #free_position = self.eyeMecha.position

        #calculate the rotation matrix
        # ls is basically equal to self.needle_pose
        #self.look_for_tip()

        # define translation speed and rotation angles

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

        self.sphere2.getObject('mstate').position = free_position

def createScene(rootNode):
    obj = MainScene(rootNode)
    obj.createGraph(rootNode)