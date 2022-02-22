import pybullet
import pybullet as p
import qibullet


CONTROLLER_ID = 0
POSITION = 1
ORIENTATION = 2
NUM_MOVE_EVENTS = 5
BUTTONS = 6
ANALOG_AXIS = 3


def euc_dist(posA, posB):
    dist = 0.
    for i in range(len(posA)):
        dist += (posA[i] - posB[i]) ** 2
    return dist


if __name__ == "__main__":
    simulation_manager = qibullet.SimulationManager()
    client_id = simulation_manager.launchSimulation(gui=False, use_shared_memory=True, auto_step=False)

    pepper = simulation_manager.spawnPepper(
        client_id,
        spawn_ground_plane=True)

    cube = p.loadURDF("D:/Documents/~UNI_BACKUP~/Second_Year/Project CMP3753M/qiBullet Project/Python/objects/cube.urdf")

    joint = p.createConstraint(cube, -1, pepper.getRobotModel(), pepper.link_dict['r_hand'].index(), p.JOINT_POINT2POINT, [0,1,0], [0,0,0], [0,0,0])

    events = p.getVREvents()
    for e in events:
        if e[CONTROLLER_ID] == 2:
            p.resetBasePositionAndOrientation(cube, e[POSITION], e[ORIENTATION])



    try:
        while True:
            events = p.getVREvents()
            HMDEvents = p.getVREvents(2)

            """print("Pepper joints: {}".format((pepper.getAnglesPosition(["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "LHand"]))))"""
            """bodyUniqueId = pepper.getRobotModel()
            endEffectorLinkIndex = pepper.link_dict["r_hand"].getIndex()
            pos = [0.1, 0.2, 0.1]
            ik = p.calculateInverseKinematics2(bodyUniqueId, endEffectorLinkIndex, pos)
            print(len(ik))"""
            """for e in events:
                print("Controller ID: {}".format(e[CONTROLLER_ID]))
                print("Position: {}".format(e[POSITION]))
                print("Orientation: {}".format(e[ORIENTATION]))
                print("---")

                for button in e[BUTTONS]:
                    if button == 1:
                        print(e[CONTROLLER_ID], e[BUTTONS])
                if e[ANALOG_AXIS] != 0:
                    print(e[CONTROLLER_ID], e[ANALOG_AXIS])

            for e in HMDEvents:
                print(e[ORIENTATION])
                pepper.moveTo(e[POSITION][0], e[POSITION][1], e[ORIENTATION][3], frame=1, _async=True)"""

            simulation_manager.stepSimulation(client_id)

    except KeyboardInterrupt:
        pass

    simulation_manager.removePepper(pepper)
    simulation_manager.stopSimulation(client_id)
    p.disconnect(client_id)
