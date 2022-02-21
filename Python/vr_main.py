import pybullet as p
import qibullet

CONTROLLER_ID = 0
POSITION = 1
ORIENTATION = 2
NUM_MOVE_EVENTS = 5
BUTTONS = 6
ANALOG_AXIS = 3

if __name__ == "__main__":
    simulation_manager = qibullet.SimulationManager()
    client_id = simulation_manager.launchSimulation(gui=False, use_shared_memory=True, auto_step=False)
    
    pepper = simulation_manager.spawnPepper(
        client_id,
        spawn_ground_plane=True)

    try:
        while True:
            events = p.getVREvents()
            HMDEvents = p.getVREvents(2)
            for e in events:
                for button in e[BUTTONS]:
                    if button == 1:
                        print(e[CONTROLLER_ID], e[BUTTONS])
                if e[ANALOG_AXIS] != 0:
                    print(e[CONTROLLER_ID], e[ANALOG_AXIS])

            for e in HMDEvents:
                print(e[ORIENTATION])
                pepper.moveTo(e[POSITION][0], e[POSITION][1], 0, frame=1, _async=True)

            simulation_manager.stepSimulation(client_id)

    except KeyboardInterrupt:
        pass
            
    simulation_manager.removePepper(pepper)
    simulation_manager.stopSimulation(client_id)
    p.disconnect(client_id)
