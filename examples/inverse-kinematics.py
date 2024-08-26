from __future__ import print_function

import numpy as np
from numpy.linalg import norm, solve
from sys import argv
from os.path import dirname, join, abspath
import pinocchio
import time

pinocchio_model_dir = join(dirname(dirname(str(abspath(__file__)))), "models")

# You should change here to set up your own URDF file or just pass it as an argument of this example.
urdf_filename = (
    "./vx300s.urdf"
)
model = pinocchio.buildModelFromUrdf(urdf_filename)
# model = pinocchio.buildSampleModelManipulator()
data = model.createData()
###########
# joints
###########
# universe
# waist
# shoulder
# elbow
# forearm_roll
# wrist_angle
# wrist_rotate
# gripper

JOINT_ID = 7
oMdes = pinocchio.SE3(np.eye(3), np.array([0.3, 0.1, 0.3]))

q = pinocchio.neutral(model)
eps = 3e-3
IT_MAX = 1000
DT = 1e-1
damp = 1e-12
# q = [0.0, 0.0, 0., 0., 0., 0., 0., 0.]
q = [ -0.679, -0.608, 0.967, -1.158, -0.751, 1.012, 1.707, 0.0]
q = np.array(q)
print("initial q: ", q)
i = 0
start = time.time()
while True:
    pinocchio.forwardKinematics(model, data, q)
    iMd = data.oMi[JOINT_ID].actInv(oMdes)
    err = pinocchio.log(iMd).vector  # in joint frame
    if norm(err) < eps:
        success = True
        break
    if i >= IT_MAX:
        success = False
        break
    J = pinocchio.computeJointJacobian(model, data, q, JOINT_ID)  # in joint frame
    J = -np.dot(pinocchio.Jlog6(iMd.inverse()), J)
    v = -J.T.dot(solve(J.dot(J.T) + damp * np.eye(6), err))
    q = pinocchio.integrate(model, q, v * DT)
    if not i % 10:
        print("%d: error = %s" % (i, err.T))
    i += 1
end = time.time()

print("total time: ", end - start)
if success:
    print("Convergence achieved!")
else:
    print(
        "\nWarning: the iterative algorithm has not reached convergence to the desired precision"
    )

print("\nresult: %s" % q.flatten().tolist())
print("\nfinal error: %s" % err.T)


# q = [-0.49599570375936863, -0.4000007133364565, 0.8353610256704259, -0.9088585096450451, -0.6476674029486584, 0.9039197817991994, 0.9941650165682081, -0.10786992088591914]
# q = np.array(q)
pinocchio.forwardKinematics(model, data, q)
for name, oMi in zip(model.names, data.oMi):
    print(("{:<24} : {: .4f} {: .4f} {: .4f}".format(name, *oMi.translation.T.flat)))
# 0.3, -0.1, 0.3
# name:
# - waist
# - shoulder
# - elbow
# - forearm_roll
# - wrist_angle
# - wrist_rotate
# - gripper

# position:
# - -0.6795535087585449
# - -0.6089903712272644
# - 0.9679418802261353
# - -1.1581555604934692
# - -0.7516506314277649
# - 1.0124273300170898
# - 1.7073206901550293

# result: [-0.4996292031463921, -0.4095252174014675, -0.8399677478888944, -0.918397330315649, 0.6473635793137184, 0.4034789571181939, 0.9197006531611129]

# 0.3, 0.1, 0.3
# name:
# - waist
# - shoulder
# - elbow
# - forearm_roll
# - wrist_angle
# - wrist_rotate
# - gripper
# - left_finger
# - right_finger
# position:
# - 0.6795535087585449
# - -0.6089903712272644
# - 0.9679418802261353
# - 1.1566215753555298
# - -0.7516506314277649
# - -1.0108933448791504
# - 1.7073206901550293
# - 0.06204342097043991
# - -0.06204342097043991


# 0.3, 0.0, 0.4
# name:
# - waist
# - shoulder
# - elbow
# - forearm_roll
# - wrist_angle
# - wrist_rotate
# - gripper
# - left_finger
# - right_finger
# position:
# - 0.0015339808305725455
# - -0.8176117539405823
# - 0.7378447651863098
# - -0.0015339808305725455
# - 0.07669904083013535
# - 0.004601942375302315
# - 1.7103886604309082
# - 0.06202271953225136
# - -0.06202271953225136


# 0.3, 0.0, 0.3
# name:
# - waist
# - shoulder
# - elbow
# - forearm_roll
# - wrist_angle
# - wrist_rotate
# - gripper
# - left_finger
# - right_finger
# position:
# - 0.0
# - -0.7731263637542725
# - 1.0461748838424683
# - -0.0015339808305725455
# - -0.27458256483078003
# - 0.004601942375302315
# - 1.7103886604309082
# - 0.06202271953225136
# - -0.06202271953225136


# 0.3, 0.0, 0.2
# 0.0015339808305725455
# - -0.4586602747440338
# - 1.2824079990386963
# - -0.0015339808305725455
# - -0.8237476944923401
# - -0.00920388475060463
# - 0.6826214790344238
# - 0.04509120434522629
# - -0.04509120434522629

# result: [1.2891302122480344e-16, -0.07331030510734514, -0.9440306729587163, 1.5688163254680151e-16, 0.8707203678513389, -1.0143020879547478e-16, 1.0, 0.0, 0.0, 0.0]


# start
# name:
# - waist
# - shoulder
# - elbow
# - forearm_roll
# - wrist_angle
# - wrist_rotate
# - gripper
# - left_finger
# - right_finger
# position:
# - -0.003067961661145091
# - -1.8392430543899536
# - 1.5600584745407104
# - -0.04601942375302315
# - 0.3405437469482422
# - 0.12118448317050934
# - 0.6273981332778931
# - 0.043150387704372406
# - -0.043150387704372406