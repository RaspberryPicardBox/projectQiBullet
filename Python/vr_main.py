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
        print("Head tick")
        HMDEvents = p.getVREvents(2)
        for e in HMDEvents:
            pepper.moveTo(pepper.getPosition()[0], pepper.getPosition()[1], eular_angle('roll', e[ORIENTATION]) + 1.5708, speed=0.1, frame=1, _async=True)


def move_left_arm(pepper):
    global LendEffectorLinkIndex
    while True:
        print("Left tick")
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
        print("Right tick")
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
    while True:
        print("Move tick")
        events = p.getVREvents()
        for e in events:
            if e[CONTROLLER_ID] == 1:
                pepper.setAngles('LWristYaw', eular_angle('yaw', e[ORIENTATION]), 1)
                if e[BUTTONS] == GRIP:
                    pepper.moveTo(1, 0, 0, _async=True)
            else:
                pepper.setAngles('RWristYaw', eular_angle('yaw', e[ORIENTATION]), 1)
                if e[BUTTONS] == GRIP:
                    pepper.moveTo(-1, 0, 0, _async=True)


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

    LendEffectorLinkIndex = pepper.link_dict["l_hand"].getIndex()
    RendEffectorLinkIndex = pepper.link_dict["r_hand"].getIndex()

    head_thread = threading.Thread(target=rotate_head, args=(pepper,), daemon=True)
    l_hand_thread = threading.Thread(target=move_left_arm, args=(pepper,), daemon=True)
    r_hand_thread = threading.Thread(target=move_right_arm, args=(pepper,), daemon=True)
    movement_thread = threading.Thread(target=move_thread, args=(pepper,), daemon=True)
    head_thread.start()
    l_hand_thread.start()
    r_hand_thread.start()
    #movement_thread.start()

    try:
        while True:
            print("Main tick")
            events = p.getVREvents()

            for e in events:

                if sum(e[BUTTONS]) > 0:
                    print(e[CONTROLLER_ID], e[BUTTONS])

                if e[CONTROLLER_ID] == 1:
                    pepper.setAngles('LWristYaw', eular_angle('yaw', e[ORIENTATION]) - 3.14159, 1)
                    if e[BUTTONS][2] > 0 and e[BUTTONS][34] > 0:
                        print("Moving")
                        pepper.move(5, 0, 0, _async=False)
                else:
                    pepper.setAngles('RWristYaw', eular_angle('yaw', e[ORIENTATION]) - 3.14159, 1)
                    if e[BUTTONS][2] > 0 and e[BUTTONS][34] > 0:
                        print("Moving")
                        pepper.move(-5, 0, 0, _async=True)

    except KeyboardInterrupt or SystemExit:
        pass

    simulation_manager.removePepper(pepper)
    simulation_manager.stopSimulation(client_id)
    p.disconnect(client_id)
