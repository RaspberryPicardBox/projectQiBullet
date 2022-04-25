import math
import pybullet
import pybullet as p
import qibullet
import threading


CONTROLLER_ID = 0
POSITION = 1
ORIENTATION = 2
NUM_MOVE_EVENTS = 5
BUTTONS = 6
ANALOG_AXIS = 3

GRIP = (0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)


def eular_angle(axis, quart):
    # From Shital Shah on https://stackoverflow.com/questions/5782658/extracting-yaw-from-a-quaternion
    if axis == 'roll':
        angle = math.atan2(2.0 * (quart[3] * quart[2] + quart[0] * quart[1]), 1.0 - 2.0 * (quart[1] * quart[1] + quart[2] * quart[2]))
    elif axis == 'pitch':
        angle = math.asin(2.0 * (quart[2] * quart[0] - quart[3] * quart[1]))
    elif axis == 'yaw':
        angle = math.atan2(2.0 * (quart[3] * quart[0] + quart[1] * quart[2]), - 1.0 + 2.0 * (quart[0] * quart[0] + quart[1] * quart[1]))
    else:
        angle = None

    return angle


def rotate_head(pepper):
    while True:
        HMDEvents = p.getVREvents(2)
        for e in HMDEvents:
            pepper.moveTo(pepper.getPosition()[0], pepper.getPosition()[1], eular_angle('roll', e[ORIENTATION]) + 1.5708,
                          speed=0.1, frame=1, _async=True)


def move_left_arm(pepper):
    while True:
        events = p.getVREvents()

        if events[CONTROLLER_ID] == 1:
            endEffectorLinkIndex = pepper.link_dict["l_hand"].getIndex()
            ik = p.calculateInverseKinematics(bodyUniqueId, endEffectorLinkIndex, e[POSITION])
            angles = list(ik)[:-3]
            angles.remove(angles[9])
            angles.remove(angles[28])
            angles[0:3] = [0, 0, 0]
            pepper.setAngles(joints, angles, 1)
            pepper.setAngles('LWristYaw', eular_angle('yaw', e[ORIENTATION]), 1)


def move_right_arm(pepper):
    while True:
        events = p.getVREvents()

        if events[CONTROLLER_ID] == 2:
            endEffectorLinkIndex = pepper.link_dict["r_hand"].getIndex()
            ik = p.calculateInverseKinematics(bodyUniqueId, endEffectorLinkIndex, e[POSITION])
            angles = list(ik)[:-3]
            angles.remove(angles[9])
            angles.remove(angles[28])
            angles[0:3] = [0, 0, 0]

            pepper.setAngles(joints, angles, 1)
            pepper.setAngles('RWristYaw', eular_angle('yaw', e[ORIENTATION]), 1)


def move_thread(pepper):
    while True:
        events = p.getVREvents()
        for e in events:
            if e[CONTROLLER_ID] == 1:
                if e[BUTTONS] == GRIP:
                    pepper.moveTo(1, 0, 0, _async=True)
            else:
                if e[BUTTONS] == GRIP:
                    pepper.moveTo(1, 0, 0, _async=True)


if __name__ == "__main__":
    simulation_manager = qibullet.SimulationManager()
    client_id = simulation_manager.launchSimulation(gui=False, use_shared_memory=True, auto_step=False)

    pybullet.setRealTimeSimulation(1)

    pepper = simulation_manager.spawnPepper(
        client_id,
        spawn_ground_plane=True)
    pepper.goToPosture('Stand', 1)

    pybullet.setVRCameraState(trackObject=pepper.getRobotModel(), trackObjectFlag=1)

    bodyUniqueId = pepper.getRobotModel()
    joints = list(pepper.joint_dict.keys())
    joints.remove('LWristYaw')
    joints.remove('RWristYaw')

    head_thread = threading.Thread(target=rotate_head, args=(pepper,), daemon=True)
    l_hand_thread = threading.Thread(target=move_left_arm, args=(pepper,), daemon=True)
    r_hand_thread = threading.Thread(target=move_right_arm, args=(pepper,), daemon=True)
    movement_thread = threading.Thread(target=move_thread, args=(pepper,), daemon=True)
    head_thread.start()
    l_hand_thread.start()
    r_hand_thread.start()
    movement_thread.start()

    try:
        while True:
            events = p.getVREvents()

            for e in events:
                if e[BUTTONS]:
                    print(e[CONTROLLER_ID], e[BUTTONS])

    except KeyboardInterrupt or SystemExit:
        pass

    simulation_manager.removePepper(pepper)
    simulation_manager.stopSimulation(client_id)
    p.disconnect(client_id)
