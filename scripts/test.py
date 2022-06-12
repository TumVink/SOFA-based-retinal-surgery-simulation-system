import sys
import Sofa
import spacenavigator
import time
from multiprocessing.connection import Client
from array import array

##                        +y
##                         .
##                         .
##                         .
##              -x ...  center  ... x (label:wireless)
##                         .
##                         .
##                         .
##                         -y

address = ('localhost', 6000)
conn = Client(address, authkey='secret password')


def createScene(rootNode):
    rootNode.findData('dt').value = '0.01'
    rootNode.findData('gravity').value = '0 0 0'
    rootNode.createObject('RequiredPlugin', name='SofaPython')

    myParticle = Particle(rootNode,[])
    return 0;


class Particle (Sofa.PythonScriptController):

    def createGraph(self,rootNode):
        return 0;


    def initGraph(self, node):
        #spacemouse

        self.rootNode = node
        self.Particle = node.createChild('Particle-0')
        self.Particle.createObject('EulerImplicitSolver')
        self.Particle.createObject('CGLinearSolver', threshold='1e-09', tolerance='1e-09', iterations='200')
        self.Pose = self.Particle.createObject('MechanicalObject', showObject='1', position='0 0 0    0 0 0 1', name='Particle-0', template='Rigid3d')
        self.Particle.createObject('UniformMass', totalMass='1')
        self.CFF=self.Particle.createObject('ConstantForceField', name="CFF", totalForce="0 0 0 0 0 0" )
        self.iteration = 1
        return 0;

    def onBeginAnimationStep(self,dt):
        arr = conn.recv()
        # force = self.CFF.findData('totalForce').value
        # force[0][0] = force[0][0] + arr[0]
        # force[0][1] = force[0][1] + arr[1]
        # force[0][2] = force[0][2] + arr[2]
        # force[0][3] = force[0][3] + arr[3]
        # force[0][4] = force[0][4] + arr[4]
        # force[0][5] = force[0][5] + arr[5]
        force = self.Pose.findData('velocity').value
        force[0][0] = force[0][0] + arr[0]
        force[0][1] = force[0][1] + arr[1]
        force[0][2] = force[0][2] + arr[2]
        force[0][3] = force[0][3] + arr[3]
        force[0][4] = force[0][4] + arr[4]
        force[0][5] = force[0][5] + arr[5]
        self.Pose.findData('velocity').value = force
        print arr
        return 0;


    # def onKeyPressed(self, c):
    #     if c=="B" :
    #         self.addParticle()
    #
    #     if c=="W" :
    #         force = self.CFF.findData('totalForce').value
    #         force[0][0] = force[0][0] +0.1
    #         self.CFF.findData('force').value = force
    #         print '+'
    #
    #     if c=="S" :
    #         force = self.CFF.findData('totalForce').value
    #         force[0][0] = force[0][0] -0.1
    #         self.CFF.findData('force').value = force
    #         print '-'
    #
    #     return 0;


    def addParticle(self):
        iteration_loc = self.iteration
        name = 'Particle'+str(iteration_loc)
        print name
        myParticleNode = self.rootNode.createChild(str(name))
        myParticleNode.createObject('EulerImplicitSolver')
        myParticleNode.createObject('CGLinearSolver', threshold='1e-09', tolerance='1e-09', iterations='200')
        myParticleNode.createObject('MechanicalObject', showObject='1', position='0 0 '+str(iteration_loc)+'    0 0 0 1', name='myParticle-'+str(iteration_loc), template='Rigid3d')

        myNewMass = myParticleNode.createObject('UniformMass', totalMass='1')
        myNewCFF  = myParticleNode.createObject('ConstantForceField', name="CFF", totalForce="0 0 0 0 0 0" )

        myNewMass.init()
        myNewCFF.init()

        self.iteration = iteration_loc +1

        return 0;


success = spacenavigator.open(DeviceNumber=5)
#list = spacenavigator.list_devices()
#print(list)
t=0
flag = True
if success:
  while 1:
    while (t%10==0):
      state = spacenavigator.read()
      print(state.x, state.y, state.z)
      flag = not flag
      success.set_led(flag)
      time.sleep(0.5)
    t = t+1
