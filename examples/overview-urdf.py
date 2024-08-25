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
# real robot
q = [0.0, 0.0, 0., 0., 0., 0., 0., 0.]

# q = [-0.4996292031463921, -0.4095252174014675, -0.8399677478888944, -0.918397330315649, 0.6473635793137184, 0.4034789571181939, 0.9197006531611129, -0.6716216949699841]
# q = [0.7693837283790212, -0.7870014503863341, -0.855487651088164, 1.5006053995195083, 0.7718696539468464, -0.736395456551299, 0.740894256183462, -0.6716216949699841]

q = np.array(q)
print("q: %s" % q.T)

# Perform the forward kinematics over the kinematic tree
pinocchio.forwardKinematics(model, data, q)

# Print out the placement of each joint of the kinematic tree
for name, oMi in zip(model.names, data.oMi):
    print(("{:<24} : {: .2f} {: .2f} {: .2f}".format(name, *oMi.translation.T.flat)))
