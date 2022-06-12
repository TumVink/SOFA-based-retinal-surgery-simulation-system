import Sofa
import os
from stlib.physics.rigid import Floor


class MainScene(Sofa.PythonScriptController):

    def createGraph(self, node):
        dt = 0.02  # In second
        displayFlags = ['showForceFields', 'showCollisionModels', 'showBehavior']

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
        self.rootNode.createObject("LocalMinDistance", name="Intersection", alarmDistance="2", contactDistance="1.5",
                                   useLMDFilters="0")
        self.rootNode.createObject('DefaultContactManager', name="Response", response="FrictionContact",
                                   responseParams='mu=0.8')

        # ==========================
        # FAT NODE
        # ==========================

        meshFile = "../models/out/Segmentation_Mar09_Fat.stl"

        fatNode = self.rootNode.createChild("Fat")

        fatNode.createObject('EulerImplicitSolver', rayleighStiffness=0.1, rayleighMass=0.1)
        fatNode.createObject('CGLinearSolver', iterations=25, tolerance=1e-9, threshold=1e-9, printLog=True)
        fatNode.createObject('MechanicalObject', name='mechObject', template="Vec3d", dx="0", dy="0", dz="0", rx="0",
                             ry="0", rz="0", scale="1.0")
        fatNode.createObject('UniformMass', template="Vec3d,double", name='mass', totalMass="1")

        fatNode.createObject('RegularGridTopology', nx=6, ny=6, nz=6, xmin=-155, xmax=125, ymin=-170, ymax=45,
                             zmin=-120, zmax=10)
        # fatNode.createObject('RegularGridSpringForceField', name="Springs", stiffness="50", damping="1")
        # fatNode.createObject('HexahedronFEMForceField', template="Vec3d", name="FEM", poissonRatio="0.45", youngModulus="2000")
        fatNode.createObject('TetrahedronFEMForceField', template='Vec3d', name='FEM_tissue',
                             method='polar', poissonRatio='0.495', youngModulus='400', computeVonMisesStress='1',
                             showVonMisesStressPerNode='false', listening='1', updateStiffness='true',
                             showStressColorMap='blue 1 0 0 1  1 0.5 0.5 1', isToPrint='1')
        fatNode.createObject('UncoupledConstraintCorrection')

        # Visual node
        fatVisNode = fatNode.createChild('Visual')
        fatVisNode.createObject('MeshSTLLoader', name='meshLoader', filename=meshFile)
        fatVisNode.createObject('OglModel', src='@meshLoader', color="#ecc854", name="Fat")
        fatVisNode.createObject('BarycentricMapping', name="Visual Mapping", output="@Fat")

        # Collision node
        fatColNode = fatNode.createChild('Collision')
        fatColNode.createObject('MeshSTLLoader', name="meshLoader", filename=meshFile)
        # fatColNode.createObject('OglModel',name='Visual', src='@meshLoader', color="#ecc854")
        fatColNode.createObject('MeshTopology', src="@meshLoader")
        fatColNode.createObject('MechanicalObject', src="@meshLoader", name="CollisionObject", template="Vec3d",
                                scale="1.0")
        fatColNode.createObject('TTriangleModel', template="Vec3d")
        fatColNode.createObject('TLineModel')
        fatColNode.createObject('TPointModel')
        fatColNode.createObject('BarycentricMapping', name="Mechanical Mapping")

        # ==========================
        # NEEDLE NODE
        # ==========================

        meshFile = "../models/out/syrette2.obj"
        scale = 10

        needleNode = self.rootNode.createChild("Needle")

        needleNode.createObject('EulerImplicitSolver', name="ODE solver", rayleighStiffness="0.01", rayleighMass="1.0")
        needleNode.createObject('CGLinearSolver', name="linear solver", iterations="25", tolerance="1e-10",
                                threshold="10e-10")
        needleNode.createObject('MechanicalObject', name="mechObject", template="Rigid3d", dx=0, dy=150, dz=-50, rx=0,
                                ry=0, rz=-90.0, scale3d=[scale, scale, scale])
        needleNode.createObject('UniformMass', name="mass", totalMass="5")
        needleNode.createObject('UncoupledConstraintCorrection')

        # Visual node
        needleVisNode = needleNode.createChild("VisualModel")
        needleVisNode.createObject('MeshObjLoader', name='instrumentMeshLoader', filename=meshFile)
        needleVisNode.createObject('OglModel', name="InstrumentVisualModel", src='@instrumentMeshLoader', dy=-2 * scale,
                                   scale3d=[scale, scale, scale])

        needleVisNode.createObject('RigidMapping', name="MM-VM mapping", input="@../mechObject",
                                   output="@InstrumentVisualModel")

        # Collision node
        needleColNode = needleNode.createChild("CollisionModel")
        needleColNode.createObject('MeshObjLoader', filename=meshFile, name="loader")
        needleColNode.createObject('MeshTopology', src="@loader", name="InstrumentCollisionModel")
        needleColNode.createObject('MechanicalObject', src="@InstrumentCollisionModel", name="instrumentCollisionState",
                                   dy=-2 * scale, scale3d=[scale, scale, scale])
        needleColNode.createObject('TTriangleModel', name="instrumentTrinagle", contactStiffness="500",
                                   contactFriction="0.01")
        needleColNode.createObject('TLineModel', name="instrumentLine", contactStiffness="500",
                                   contactFriction="0.01")
        needleColNode.createObject('TPointModel', name="instrumentPoint",
                                   contactStiffness="500", contactFriction="0.01")
        needleColNode.createObject('RigidMapping', name="MM-CM mapping", input="@../mechObject",
                                   output="@instrumentCollisionState")

        needleNode.createObject('LinearMovementConstraint', template="Rigid3d",
                                indices=0,
                                keyTimes=[0, 0.8, 1.6, 1.7],
                                movements=[[0, 0, 0, 0, 0, 0],
                                           [0, -80, 0, 0, 0, 0],
                                           [0, 0, 0, 0, 0, 0],
                                           [0, 0, 0, 0, 0, 0]])

        # Instantiate floor
        floor = Floor(self.rootNode, name="Floor", translation=[0.0, -180.0, -50.0], uniformScale=6,
                      isAStaticObject=True, color=[0.3, 0.3, 0.4])

        return 0


def createScene(rootNode):
    obj = MainScene(rootNode)
    obj.createGraph(rootNode)
