import time
import numpy as np
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
import example_robot_data

robot = example_robot_data.load('panda')
model = robot.model
collision_model = robot.collision_model
visual_model = robot.visual_model

data = model.createData()

eef_frame_id = model.getFrameId("panda_hand")

RADIUS = 0.3
CENTER = np.array([0.5, 0.0, 0.3])
NUM_POINTS = 200
theta_vals = np.linspace(0, 2*np.pi, NUM_POINTS)

target_positions = []
for theta in theta_vals:
    target_positions.append(np.array([
        CENTER[0] + RADIUS * np.cos(theta),
        CENTER[1] + RADIUS * np.sin(theta),
        CENTER[2]
    ]))

eps = 1e-4
IT_MAX = 1000
damp = 1e-12
DT = 0.03

joint_trajectory = []
for idx, target_pos in enumerate(target_positions):
    q = pin.neutral(model)
    desired_orientation = np.array([[1., 0., 0.],
                                    [0., -1., 0.],
                                    [0., 0., 1.]])
    oMdes = pin.SE3(desired_orientation, target_pos)

    for k in range(IT_MAX):
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)

        oMi = data.oMf[eef_frame_id]

        dMi = oMi.actInv(oMdes)
        err = pin.log(dMi).vector

        if np.linalg.norm(err) < eps:
            break

        J = pin.computeFrameJacobian(model, data, q, eef_frame_id)
        J = -np.dot(pin.Jlog6(dMi.inverse()), J)

        v = -J.T.dot(np.linalg.solve(J.dot(J.T) + damp * np.eye(6), err))
        q = pin.integrate(model, q, v * DT)

    joint_trajectory.append(q.copy())

viz = MeshcatVisualizer(model, collision_model, visual_model)
viz.initViewer(open=True)
viz.loadViewerModel("panda")
viz.display(pin.neutral(model))

input("Press Enter to start.")
for q in joint_trajectory:
    viz.display(q)
    time.sleep(0.03)
