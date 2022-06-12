import spacenavigator
import time
from multiprocessing.connection import Listener
from array import array

#setup listener
address = ('localhost', 6000)
listener = Listener(address, authkey='secret password')
conn = listener.accept()

#setup driver
success = spacenavigator.open(DeviceNumber=5)
#print success

if success:
  while 1:
      state = spacenavigator.read()
      t = time.time()*1000
      print state
      conn.send([state.x, state.y, state.z, state.roll,state.pitch,state.yaw,t])

      #success.set_led(flag)
      time.sleep(0.1)

# def createScene(rootNode):
#     rootNode.findData('dt').value = '0.01'
#     rootNode.findData('gravity').value = '0 0 0'
#     rootNode.createObject('RequiredPlugin', name='SofaPython')
#
#     #myParticle = Particle(rootNode,[])
#     return 0;

#Vendor ID:              0x256f
#Product ID:             0xc652
#Version number:         0x0106

###################
###         pitch+
#
#-roll       clock:+           +roll


###         pitch-


# slide:  z-  xia                z+ shang
