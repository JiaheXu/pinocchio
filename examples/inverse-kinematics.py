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

JOINT_ID = 6
oMdes = pinocchio.SE3(np.eye(3), np.array([0.3, 0.0, 0.2]))

q = pinocchio.neutral(model)
eps = 1e-3
IT_MAX = 1000
DT = 1e-1
damp = 1e-12

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
