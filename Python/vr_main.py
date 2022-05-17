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
BUTTON_X = (0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)


def eular_angle(axis, quart):
    # From Shital Shah on https://stackoverflow.com/questions/5782658/extracting-yaw-from-a-quaternion
    if axis == 'roll':
        angle = math.atan2(2.0 * (quart[3] * quart[2] + quart[0] * quart[1]), 1.0 - 2.0 * (quart[1] * quart[1] + quart[2] * quart[2]))
        # angle = p.getEulerFromQuaternion(quart)[0]
    elif axis == 'pitch':
        angle = math.asin(2.0 * (quart[2] * quart[0] - quart[3] * quart[1]))
        # angle = p.getEulerFromQuaternion(quart)[1]
    elif axis == 'yaw':
        angle = math.atan2(2.0 * (quart[3] * quart[0] + quart[1] * quart[2]), - 1.0 + 2.0 * (quart[0] * quart[0] + quart[1] * quart[1]))
        # angle = p.getEulerFromQuaternion(quart)[2]
    else:
        angle = None

    return angle


def rotate_head(pepper):
    while True:
        HMDEvents = p.getVREvents(2)
        for e in HMDEvents:
            pepper.moveTo(pepper.getPosition()[0], pepper.getPosition()[1], eular_angle('roll', e[ORIENTATION]) + 1.5708, speed=0.1, frame=1, _async=True)


def move_left_arm(pepper):
    global LendEffectorLinkIndex
    while True:
        events = p.getVREvents()
        try:
            if events[0]:
                ik = p.calculateInverseKinematics(bodyUniqueId, LendEffectorLinkIndex, events[0][POSITION])
                angles = list(ik)[:-3]
                angles.remove(angles[9])
                angles.remove(angles[28])
                angles[0:3] = [0, 0, 0]
                pepper.setAngles(joints, angles, 1)
        except IndexError:
            pass


def move_right_arm(pepper):
    global RendEffectorLinkIndex
    while True:
        events = p.getVREvents()
        try:
            if events[1]:
                ik = p.calculateInverseKinematics(bodyUniqueId, RendEffectorLinkIndex, events[1][POSITION])
                angles = list(ik)[:-3]
                angles.remove(angles[9])
                angles.remove(angles[28])
                angles[0:3] = [0, 0, 0]
                pepper.setAngles(joints, angles, 1)
        except IndexError:
            pass


def move_thread(pepper):
    """global head_thread
    while True:
        events = p.getVREvents()
        for e in events:
            if e[CONTROLLER_ID] == 1:
                pepper.setAngles('LWristYaw', -eular_angle('yaw', e[ORIENTATION])-3.14, 1)
                if e[BUTTONS] == GRIP and p.VR_BUTTON_IS_DOWN:
                    pepper.moveTo(1, 0, 0, speed=5, _async=True)
            else:
                pepper.setAngles('RWristYaw', -eular_angle('yaw', e[ORIENTATION])+3.14, 1)
                if e[BUTTONS] == GRIP and p.VR_BUTTON_IS_DOWN:
                    pepper.moveTo(-1, 0, 0, speed=5, _async=True)"""

    while True:
        events = p.getVREvents()

        for e in events:
            if e[CONTROLLER_ID] == 1:
                if e[BUTTONS] == BUTTON_X and p.VR_BUTTON_IS_DOWN:
                    pepper.move(0.5, 0, eular_angle("roll", e[ORIENTATION]))
                    print("X button pressed")
                if e[BUTTONS][7] == 0:
                    pepper.stopMove()

            if e[CONTROLLER_ID] == 2:
                if e[BUTTONS] == BUTTON_X and p.VR_BUTTON_IS_DOWN:
                    pepper.move(-0.5, 0, eular_angle("roll", e[ORIENTATION]))
                    print("X button pressed")
                if e[BUTTONS][7] == 0:
                    pepper.stopMove()


if __name__ == "__main__":
    simulation_manager = qibullet.SimulationManager()
    client_id = simulation_manager.launchSimulation(gui=False, use_shared_memory=True, auto_step=False)

    pybullet.setRealTimeSimulation(1)

    pepper = simulation_manager.spawnPepper(
        client_id,
        spawn_ground_plane=True)
    pepper.goToPosture('Stand', 1)

    table = p.loadURDF("../../../Python/objects/TikkaNOVA.urdf", basePosition=(1.5, 0, 0.4), baseOrientation=(1, 0, 0, 1), useFixedBase=1)
    mug = p.loadURDF("../../../Python/objects/Cappuccino_cup.urdf", basePosition=(1.2, 0, 0.7), baseOrientation=(1, 1, 1, 1))

    pybullet.setVRCameraState(trackObject=pepper.getRobotModel(), trackObjectFlag=1)

    bodyUniqueId = pepper.getRobotModel()
    joints = list(pepper.joint_dict.keys())
    joints.remove('LWristYaw')
    joints.remove('RWristYaw')

    LendEffectorLinkIndex = pepper.link_dict["l_hand"].getIndex()
    RendEffectorLinkIndex = pepper.link_dict["r_hand"].getIndex()

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

    except KeyboardInterrupt or SystemExit:
        pass

    simulation_manager.removePepper(pepper)
    simulation_manager.stopSimulation(client_id)
    p.disconnect(client_id)
