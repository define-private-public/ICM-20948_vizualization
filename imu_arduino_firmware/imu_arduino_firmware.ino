// This is meant to be nothing more than a sketch that dumps data on the serial port
// for the python application to show/visualize.
//
//  Wire up the SparkFun ICM-20948 to use I2C.  Pin 7 of the arduino is also used to reset the orientation
//  to point forward, when it is turned `HIGH`.  Adding a simple switch can be used here.
//
//
// Communication protocol:  This print strings on the Serial port, at the rate of 115200 baud; it never reads data
//   It might print out status messages like "waiting for data"
//   Actual orientation data will follow the format:
//
//     HPR <heading/yaw deg> <pitch deg> <roll deg>
//
//   For example:
//
//     HPR 5.5 -10.07773 0.22000023
//

#include <Arduino.h>
#include <Wire.h>
#include <ICM_20948.h>
#include "MadgwickAHRS.h"


#define SERIAL_BAUD_RATE 115200
#define I2C_RATE 400000
#define IMU_TIMEOUT_MS 500                                                  // When there was a failure in talking to the IMU (or no data ready), how many milliseconds should we wait?
#define FUSION_UPDATES_PER_SECOND 50.0f                                     // Rate to update the fusion algorithm
#define FUSION_UPDATE_DELTA_US static_cast<unsigned long>(1000000.0f / FUSION_UPDATES_PER_SECOND)        // How many milliseconds need to pass before the fusion algoirthm is sent an update
#define SERIAL_PRECISION 12                                                 // How many digits of the float to print to Serial
#define SERIAL_NUM_UPDATES_PER_SECOND 20                                    // How many times per second to send an update
#define SERIAL_UPDATE_DELTA_MS (1000 / SERIAL_NUM_UPDATES_PER_SECOND)       // How many milliseoncds to wait until to update the orientation on the serial connection
#define RESET_ORIENTATION_PIN 7                                             // Pin that is read, when `HIGH`, will set the `orientation_offset` HRP, so that the effective orientation points forward
bool print_raw_instead = false;                                             // Flag to print the raw IMU values

// TODO research this better
#define AD0_VAL   1     // The value of the last bit of the I2C address.
                        // On the SparkFun 9DoF IMU breakout the default is 1, and when
                        // the ADR jumper is closed the value becomes 0

// Using I2C to connect to the imu
ICM_20948_I2C imu;

// For orientation calculation
Madgwick filter;

// Data readings (TODO add units onto comments)
float ax = 0.0f, ay = 0.0f, az = 0.0f;      // Accelration
float gx = 0.0f, gy = 0.0f, gz = 0.0f;      // Gyroscope
float mx = 0.0f, my = 0.0f, mz = 0.0f;      // Magnometer

// The orientation (in degrees)
struct HPR {
    float heading = 0.0f;           // A.k.a. "yaw"
    float pitch = 0.0f;
    float roll = 0.0f;
};
HPR orientation;                    // The effective orientation (what's sent over serial)
HPR orientation_offset;             // The offset for the above so the user can configure what's the "front"

// For updating certain things
unsigned long prev_serial_update_ms = 0;                                    // Previous time (in milliseconds) and update was sent
unsigned long prev_fusion_update_us = 0;                                    // Previous time (in microseonds) the fusion algorithm was updated


// converts degrees to radians
//   x - an angle in degrees
//   returns: the same angle, but expressed in radians
inline float deg_to_rad(const float x) {
    return x * (3.1415926535f / 180.0f);
}

// Print three float values onto the serial line, prefexed by an `id` string; all will be separated by a single
// space.  There will be a newline at the end.  To see some example output, look at the "Communcation protocol" section
// at the top
void print_tripplet_to_serial(const char *id, const float x, const float y, const float z) {
    Serial.print(id);
    Serial.print(" ");
    Serial.print(x, SERIAL_PRECISION);
    Serial.print(" ");
    Serial.print(y, SERIAL_PRECISION);
    Serial.print(" ");
    Serial.print(z, SERIAL_PRECISION);
    Serial.println("");     // End the line reading
}

void setup() {
    // Setup pins
    pinMode(RESET_ORIENTATION_PIN, INPUT);

    // Setup communications
    Serial.begin(SERIAL_BAUD_RATE);     // To computer
    Wire.begin();                       // To IMU
    Wire.setClock(I2C_RATE);

//    imu.enableDebugging();

    // Try to enable the IMU
    bool connected = false;
    while (!connected) {
        imu.begin(Wire, AD0_VAL);

        // Print status to serial
        Serial.print("Init connection to ICM 20948: ");
        Serial.println(imu.statusString());

        // If it didn't work, try again shortly
        if (imu.status != ICM_20948_Stat_Ok) {
            Serial.println("Try again in 500 ms");
            delay(IMU_TIMEOUT_MS);
        } else
            connected = true;
    }

    Serial.println("Connected to ICM 20948");

    // Do a reset to make sure device is in a known state
    imu.swReset();
    if (imu.status != ICM_20948_Stat_Ok) {
        Serial.print("Error resetting the IMU: ");
        Serial.println(imu.statusString());
    }
    delay(250);     // Wait 1/4 second

    // Wake up the IMU
    imu.sleep(false);
    imu.lowPower(false);

    // Change the range of values collected to the higest possible
    // as people might be jerking this around quite a bit
    ICM_20948_fss_t fss;
    fss.a = gpm16;           // Accelerometer 16 g of gravity
    fss.g = dps2000;         // Gyroscope 2000 deg/sec

    imu.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), fss);
    if (imu.status != ICM_20948_Stat_Ok) {
        Serial.print("Error setting IMU collection range: ");
        Serial.println(imu.statusString());
    }

/*
    // Add digital low-pass filter
    ICM_20948_dlpcfg_t dlp_cfg;
    dlp_cfg.a = acc_d473bw_n499bw;
    dlp_cfg.g = gyr_d361bw4_n376bw5;
    imu.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), dlp_cfg);
    if (imu.status != ICM_20948_Stat_Ok) {
        Serial.print("Error setting digital-low-pass-filter: ");
        Serial.println(imu.statusString());
    }

    // Turn on the DLPLF (TODO error handling)
    imu.enableDLPF(ICM_20948_Internal_Acc, true);
    imu.enableDLPF(ICM_20948_Internal_Gyr, true);
*/

    // Startup magnometer
    imu.startupMagnetometer();
    if (imu.status != ICM_20948_Stat_Ok) {
        Serial.print("Error starting up magnometer: ");
        Serial.println(imu.statusString());
    }

    // Setup filter
    filter.begin(FUSION_UPDATES_PER_SECOND);
}


void loop() {
    // Is Some data ready from the IMU?
    if (imu.dataReady()) {
        imu.getAGMT();          // Retrieves data

        // Place into container variables
        ax = imu.accX();
        ay = imu.accY();
        az = imu.accZ();

        gx = imu.gyrX();
        gy = imu.gyrY();
        gz = imu.gyrZ();

        // Some of the magnometer's axis need to be flipped to match that of the gyro/accel for Madgwick to work
        mx =  imu.magX();
        my = -imu.magY();
        mz = -imu.magZ();
    }

    // Check if the "reset orientation" pin has been pressed
    if (digitalRead(RESET_ORIENTATION_PIN) == HIGH) {
        // Simply set the offset to be that of what the orientation is currently
        orientation_offset.heading = filter.getYaw();
        orientation_offset.pitch   = filter.getPitch();
        orientation_offset.roll    = filter.getRoll();
    }

    // If enough time has passed, update the fusion state
    const unsigned long current_us = micros();
    if (current_us >= (prev_fusion_update_us + FUSION_UPDATE_DELTA_US)) {
        // Mark the time we updated the fusion state
        prev_fusion_update_us = current_us;

        // Update the sensor fusion state
        filter.update(
            gx, gy, gz,
            ax, ay, az,
            mx, my, mz
        );

        // Read data
        orientation.heading = filter.getYaw()   - orientation_offset.heading;
        orientation.pitch   = filter.getPitch() - orientation_offset.pitch;
        orientation.roll    = filter.getRoll()  - orientation_offset.roll;
    }

    // If enough time has passed, send an update of the orientation over the serial connection
    const unsigned long current_millis = millis();
    if (current_millis >= (prev_serial_update_ms + SERIAL_UPDATE_DELTA_MS)) {
        // Mark the time we send the data over
        prev_serial_update_ms = current_millis;

        if (print_raw_instead) {
            // Print raw IMU data
            print_tripplet_to_serial("ACC", ax, ay, az);
            print_tripplet_to_serial("GYR", gx, gy, gz);
            print_tripplet_to_serial("MAG", mx, my, mz);
            Serial.println("");
        } else {
            // Send Orientation data to the PC (HPR = Heading, Pitch, Roll)
            print_tripplet_to_serial("HPR", orientation.heading, orientation.pitch, orientation.roll);
        }
    }
}
