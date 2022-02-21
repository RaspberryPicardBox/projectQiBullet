import threading
import cv2
import pybullet as p
import qibullet
from qibullet import PepperVirtual, Camera


def camera_view():
    while True:
        img = pepper.getCameraFrame(handle)
        cv2.imshow('Camera: ', img)
        cv2.waitKey(1)


if __name__ == "__main__":
    simulation_manager = qibullet.SimulationManager()
    client_id = simulation_manager.launchSimulation(gui=True)

    p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

    pepper = simulation_manager.spawnPepper(
        client_id,
        spawn_ground_plane=True)

    pepper.goToPosture("Stand", 1)

    handle = pepper.subscribeCamera(PepperVirtual.ID_CAMERA_TOP, resolution=Camera.K_Q720p, fps=60.0)
    cameraThread = threading.Thread(None, camera_view, name="Camera-Thread")
    cameraThread.daemon = True
    cameraThread.start()

    try:
        moveForward = 0
        moveRotate = 0
        while True:
            keys = p.getKeyboardEvents()
            for key in keys:
                if key == p.B3G_UP_ARROW and keys[key] == 1:
                    moveForward = 1
                elif key == p.B3G_DOWN_ARROW and keys[key] == 1:
                    moveForward = -1
                elif key == p.B3G_RIGHT_ARROW and keys[key] == 1:
                    moveRotate = -1
                elif key == p.B3G_LEFT_ARROW and keys[key] == 1:
                    moveRotate = 1
                else:
                    moveForward = 0
                    moveRotate = 0

            pepper.moveTo(moveForward, 0, moveRotate, _async=True)

    except KeyboardInterrupt:
        pass

    simulation_manager.removePepper(pepper)
    simulation_manager.stopSimulation(client_id)
