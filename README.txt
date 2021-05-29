This is a small toy project for playing with the ICM-20948, a 9 DoF IMU that I'm
going to use to pursue a patent.  As for right now, this project's goal is to
simply visualize the orientation of the chip on the SparkFun breakout board,
using Panda3D (Python 3) and Arduino.

Two git submodules are included.  They are 3rd party code and are included mostly
a reference.  The Madgwick algorithm is used for sensor fusion.  As for the SparkFun
ICM-20948 code, it is required to be installed in the ArduinoIDE.  See below for
instructions.


Setup:
 1. Install Arduion IDE (v1.8.13 was used for development)
 2. In the Arduino IDE, install SparkFun's ICM 20948 library
    - Go to Tools >  Manage Libraries...
    - Search for "Sparkfun ICM 20948"
      - This: https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary
    - Install it (v1.2.6 was used for this project)
 3. Open the `imu_arduino_firmware` project in the Arduino IDE
 4. Compile the sketch and upload it to your Arduino UNO
 5. Connect your ICM-20948 breakout board to I2C on your arduino
    - You can also connect a tactile switch to pin 7, with a pulldown resistor to GND
      to use the "reset orientation" rfeatuer of the Sketch
 6. In the Arduino IDE, open up the serial monitor and set the buad rate to 115200
    -  If you start seeing data prefixed with "HPR" to each line, then the firmware
       is properly on the Arduino
 7. Setup a Python 3 virtual enviroment
    - e.g. `python -m venv env`
 8. Activate the virtual envirment
    - e.g. `source env/bin/activate`
 9. Install the required Python packages from `requirements.txt`:
    - `pip install -r requirements.txt`
10. Convert the .blend file to a .bam:
    - `blend2bam imu_breakout.blend imu_breakout.bam`

Now in the virtual environment, you can run:
  `python visualize_imu.py <serial_port>`

E.g. on Linux:
  `python visualize_imu.py /dev/ttyACM0`

And then should see a 3D model of the breakbout board, who's orientation should match
that of the connect Arduino/IMU.



TODO:
 - [ ] Change this to a .md (or .rst) with a photo showing this in action

