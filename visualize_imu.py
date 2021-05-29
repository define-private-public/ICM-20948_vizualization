#!/bin/env python3

import sys
from serial import Serial
from direct.showbase.ShowBase import ShowBase
from panda3d.core import AmbientLight


class VisualizeIMU(ShowBase):
    __slots__ = (
        '_imu_model',
        '_light',
        '_serial',
        '_heading',
        '_pitch',
        '_roll',
    )

    def __init__(self, port):
        ShowBase.__init__(self)

        # Note that our model is in the size of millimeters, so we need to use really tiny numbers for positioning

        # Set the camera
        self.camLens.setNearFar(0.0001, 100)
        self.disableMouse()
        self.camera.setPos(0, -0.05, 0.02)
        self.camera.lookAt(0, 0, 0)

        # load up the IMU model and set it to spin
        self._imu_model = self.loader.loadModel('imu_breakout.bam')
        self._imu_model.reparentTo(self.render)
        self.taskMgr.add(self._spin_imu_task, 'spin_imu')
        self._heading = 0.0
        self._pitch = 0.0
        self._roll = 0.0

        # Set an ambient light so things aren't so harsh
        self._light = AmbientLight('light')
        self._light.setColor((0.8, 0.8, 0.8, 1))
        light_node = self.render.attachNewNode(self._light)
        self.render.setLight(light_node)

        # Setup serial connection
        self._serial = Serial(port, 115200)
        self.exitFunc = self._cleanup_serial


    # Small code to rotate the model about the Z axis
    def _spin_imu_task(self, task):
        try:
            # Read data in from the serial port and have it control orientation
            line = self._serial.readline().decode('utf-8').strip()
            parts = line.split()

            if len(parts) == 4:
                if parts[0] == 'HPR':
                    # First make sure we got the correct data packet
                    # Then parse the degree strings
                    self._heading = float(parts[1])
                    self._pitch = float(parts[2])
                    self._roll = float(parts[3])

                    # Have the model orientation match our hardware
                    self._imu_model.setHpr(
                        self._heading,
                        self._pitch,
                        self._roll
                    )
        except:
            # Ignore all errors
            pass

        # Always continue on
        return task.cont


    # Make sure on window close we gracefully cleanup the serial connection
    def _cleanup_serial(self):
        self._serial.close()



# Run it
if __name__ == '__main__':
    # check there's enough arguments
    argc = len(sys.argv)
    if (argc < 2):
        print('Please supply a serial device to connect to; e.g. `/dev/ttyACM0`')
        sys.exit(0)

    port = sys.argv[1].strip()

    app = VisualizeIMU(port)
    app.run()
