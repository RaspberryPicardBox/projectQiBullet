import math

import pybullet
import pybullet as p
import qibullet


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


if __name__ == "__main__":
    simulation_manager = qibullet.SimulationManager()
    client_id = simulation_manager.launchSimulation(gui=True, use_shared_memory=False, auto_step=False)

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

    try:
        while True:
            events = p.getVREvents()
            HMDEvents = p.getVREvents(2)

            for e in events:

                if e[CONTROLLER_ID] == 1:
                    endEffectorLinkIndex = pepper.link_dict["l_hand"].getIndex()
                    ik = p.calculateInverseKinematics(bodyUniqueId, endEffectorLinkIndex, e[POSITION])
                    angles = list(ik)[:-3]
                    angles.remove(angles[9])
                    angles.remove(angles[28])
                    angles[0:3] = [0, 0, 0]
                    pepper.setAngles(joints, angles, 1)
                    pepper.setAngles('LWristYaw', eular_angle('yaw', e[ORIENTATION]), 1)

                if e[CONTROLLER_ID] == 2:
                    endEffectorLinkIndex = pepper.link_dict["r_hand"].getIndex()
                    ik = p.calculateInverseKinematics(bodyUniqueId, endEffectorLinkIndex, e[POSITION])
                    angles = list(ik)[:-3]
                    angles.remove(angles[9])
                    angles.remove(angles[28])
                    angles[0:3] = [0, 0, 0]

                    pepper.setAngles(joints, angles, 1)
                    pepper.setAngles('RWristYaw', eular_angle('yaw', e[ORIENTATION]), 1)

                for button in e[BUTTONS]:
                    if button == 1:
                        print(e[CONTROLLER_ID], e[BUTTONS])
                if e[ANALOG_AXIS] != 0:
                    print(e[CONTROLLER_ID], e[ANALOG_AXIS])

            for e in HMDEvents:
                pepper.moveTo(pepper.getPosition()[0], pepper.getPosition()[1], e[ORIENTATION][3], frame=1, _async=True)

    except KeyboardInterrupt or SystemExit:
        pass

    simulation_manager.removePepper(pepper)
    simulation_manager.stopSimulation(client_id)
    p.disconnect(client_id)
