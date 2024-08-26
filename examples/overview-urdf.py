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

# q = [0.4, 0.0, 0.0, 0., 0., 0., 0., 0.]
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

#q = [0.0015, -1.8146, 1.5385, 0.0030, -1.5677, 0.0076, 1.6490, 0.0623]


# -0.047, -0.000, 0.303

#check ik solver

# 0.3, -0.1, 0.3, real robot
# q = [ -0.679, -0.608, 0.967, -1.158, -0.751, 1.012, 1.707, 0.0]

# 0.3, 0.1, 0.3
#q = [0.482590558369953, -0.3978114139690695, 0.834289174737525, 0.8936957071088067, -0.6381557312987837, 0.11376602319707936, 0.6229923716947551, -0.7822279110400907]
# /upper_arm_link = shoulder
# /upper_forearm_link = elbow
# /lower_forearm_link = forearm_roll # only translation
# /wrist_link = wrist_angle
# /gripper_link close to wrist_rotate
# /ee_arm_link = gripper
#  q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
q = [
    -0.003067961661145091,
    0.013805827125906944,
    -0.003067961661145091,
    0.0015339808305725455,
    0.006135923322290182,
    0.006135923322290182,
    0.,
    0.
]
q[-2] = 0.
q[-1] = 0.
q = np.array(q)
print("q: %s" % q.T)

# Perform the forward kinematics over the kinematic tree
pinocchio.forwardKinematics(model, data, q)

# Print out the placement of each joint of the kinematic tree
for name, oMi in zip(model.names, data.oMi):
    print(("{:<24} : {: .4f} {: .4f} {: .4f}".format(name, *oMi.translation.T.flat)))
#for name, oMi in zip(model.names, data.oMi):
#    print( name, ": ", *oMi.rotation)
