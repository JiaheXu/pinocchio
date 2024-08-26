import pinocchio
from sys import argv
from os.path import dirname, join, abspath
import numpy as np
# This path refers to Pinocchio source code but you can define your own directory here.

# __file__ = "/home/jiahe/pinocchio/examples/urdf/vx300s.urdf.xacro"
pinocchio_model_dir = join(dirname(dirname(str(abspath(__file__)))), "models")

# You should change here to set up your own URDF file or just pass it as an argument of this example.
urdf_filename = (
    "./vx300s.urdf"
)
print("urdf_filename: ", urdf_filename)
# Load the urdf model
model = pinocchio.buildModelFromUrdf(urdf_filename)
print("model name: " + model.name)

# Create data required by the algorithms
data = model.createData()

# Sample a random configuration
q = pinocchio.randomConfiguration(model)
pi = 3.14159265
# real robot follower_right/ee_arm_link

q = [0.4, 0.0, 0.0, 0., 0., 0., 0., 0.]
#0.439, 0.184, 0.419

# q = [0.0, 0.4, 0.0, 0., 0., 0., 0., 0.]
0.554, -0.002, 0.210

# q = [0.0, 0.0, 0.1, 0., 0., 0., 0., 0.]
#0.474, -0.001, 0.380

# q = [0.2, 0.0, 0.2, 0., 0., 0., 0., 0.]
#0.458, 0.091, 0.339

# q = [0.0, 0.0, 0., 0.4, 0., 0., 0., 0.]
#0.477, -0.001, 0.420

# q = [0.0, 0.0, 0., 0.4, 0.4, 0., 0., 0.] 
#0.467, 0.016, 0.381

# q = [0.0015, -1.8146, 1.5385, 0.0030, -1.5677, 0.0076, 1.6490, 0.0623]
# -0.047, -0.000, 0.303

# q = [-13.496197630345746, -0.75767905687227, 1.0483944958136877, 45.76435210098318, 7.244566785664069, 27.817489033016923, -0.10177495356304714, 0.9948074481160862]
# q = [ -0.679, -0.608, 0.967, -1.158, -0.751, 1.012, 1.707, 0.0]


q = np.array(q)
print("q: %s" % q.T)

# Perform the forward kinematics over the kinematic tree
pinocchio.forwardKinematics(model, data, q)

# Print out the placement of each joint of the kinematic tree
for name, oMi in zip(model.names, data.oMi):
    print(("{:<24} : {: .4f} {: .4f} {: .4f}".format(name, *oMi.translation.T.flat)))
#for name, oMi in zip(model.names, data.oMi):
#    print( name, ": ", *oMi.rotation)
