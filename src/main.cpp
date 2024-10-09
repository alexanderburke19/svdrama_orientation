#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM9DS1.h>
#include <sstream>
#include <string>
#include "orientation_sensor.h"
#include "signalk_orientation.h"
#include "signalk_output.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp_app_builder.h"
#include "sensesp/sensors/sensor.h"

// Sensor hardware details: I2C addresses for LSM9DS1
#define LSM9DS1_ACCEL_MAG_ADDR 0x6B ///< I2C address for accelerometer and gyroscope
#define LSM9DS1_MAG_ADDR 0x1E       ///< I2C address for the magnetometer
#define PIN_I2C_SDA 21              // Adjust to your board
#define PIN_I2C_SCL 22              // Adjust to your board

// Create an instance of the LSM9DS1 sensor
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

// How often orientation parameters are published via Signal K
#define ORIENTATION_REPORTING_INTERVAL_MS 2000

using namespace sensesp;

reactesp::ReactESP app;

// SKOutput objects for basic data
SKOutputAttitude *sk_output_attitude;
SKOutputFloat *sk_output_yaw;
SKOutputFloat *sk_output_pitch;
SKOutputFloat *sk_output_roll;
SKOutputFloat *sk_output_accel_x;
SKOutputFloat *sk_output_accel_y;
SKOutputFloat *sk_output_accel_z;
SKOutputFloat *sk_output_mag_x;
SKOutputFloat *sk_output_mag_y;
SKOutputFloat *sk_output_mag_z;

// SKOutput objects for user-friendly values
SKOutputFloat *sk_output_total_acceleration;
SKOutputFloat *sk_output_magnetic_heading;
SKOutputFloat *sk_output_total_angular_velocity;
SKOutputFloat *sk_output_heel_angle;
SKOutputFloat *sk_output_rate_of_turn;
SKOutputFloat *sk_output_lateral_acceleration;
SKOutputFloat *sk_output_longitudinal_acceleration;
SKOutputFloat *sk_output_pitch_rate;
SKOutputFloat *sk_output_roll_rate;
SKOutputFloat *sk_output_true_heading;
SKOutputFloat *sk_output_g_force;
SKOutputFloat *sk_output_trim_angle;

void setup()
{
#ifndef SERIAL_DEBUG_DISABLED
    SetupSerialDebug(115200);
#endif

    // Initialize the I2C communication
    if (!lsm.begin())
    {
        debugE("Could not initialize the LSM9DS1 sensor! Check wiring.");
        while (1)
            ;
    }

    // Set the sensor range if necessary (optional)
    lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);   // 2G range
    lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);    // 4 Gauss range
    lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS); // 245 degrees per second range

    // SensESPApp initialization
    SensESPAppBuilder builder;
    sensesp_app = (&builder)
                      ->set_hostname("drama-sensesp_orientation")
                      ->set_wifi("drama_network", "sv_drama")
                      ->set_sk_server("10.10.10.1", 3000)
                      ->enable_ota("drama")
                      ->enable_uptime_sensor()
                      ->enable_ip_address_sensor()
                      ->enable_free_mem_sensor()
                      ->enable_system_hz_sensor()
                      ->enable_wifi_signal_sensor()
                      ->get_app();

    // Create SKOutputAttitude object for yaw, pitch, and roll, mapped to the Signal K path "navigation.attitude"
    sk_output_attitude = new SKOutputAttitude("navigation.attitude");

    // Create SKOutputFloat objects for yaw, pitch, and roll, mapped to the appropriate Signal K paths
    sk_output_yaw = new SKOutputFloat("navigation.attitude.yaw");
    sk_output_pitch = new SKOutputFloat("navigation.attitude.pitch");
    sk_output_roll = new SKOutputFloat("navigation.attitude.roll");

    // Create SKOutputFloat objects for accelerometer data
    sk_output_accel_x = new SKOutputFloat("sensors.accelerometer.accel_x");
    sk_output_accel_y = new SKOutputFloat("sensors.accelerometer.accel_y");
    sk_output_accel_z = new SKOutputFloat("sensors.accelerometer.accel_z");

    // Create SKOutputFloat objects for magnetometer data
    sk_output_mag_x = new SKOutputFloat("sensors.magnetometer.mag_x");
    sk_output_mag_y = new SKOutputFloat("sensors.magnetometer.mag_y");
    sk_output_mag_z = new SKOutputFloat("sensors.magnetometer.mag_z");

    // Create SKOutputFloat objects for user-friendly values
    sk_output_total_acceleration = new SKOutputFloat("sensors.acceleration.total");                // Total acceleration in m/s²
    sk_output_magnetic_heading = new SKOutputFloat("navigation.headingMagnetic");                  // Magnetic heading in degrees
    sk_output_total_angular_velocity = new SKOutputFloat("sensors.gyroscope.angular_velocity");    // Total angular velocity in DPS
    sk_output_heel_angle = new SKOutputFloat("navigation.attitude.heel");                          // Heel angle (based on roll)
    sk_output_rate_of_turn = new SKOutputFloat("navigation.rateOfTurn");                           // Rate of turn (yaw rate)
    sk_output_lateral_acceleration = new SKOutputFloat("sensors.accelerometer.lateral");           // Lateral acceleration
    sk_output_longitudinal_acceleration = new SKOutputFloat("sensors.accelerometer.longitudinal"); // Longitudinal acceleration
    sk_output_pitch_rate = new SKOutputFloat("sensors.gyroscope.pitch_rate");                      // Pitch rate
    sk_output_roll_rate = new SKOutputFloat("sensors.gyroscope.roll_rate");                        // Roll rate
    sk_output_true_heading = new SKOutputFloat("navigation.headingTrue");                          // True heading
    sk_output_g_force = new SKOutputFloat("sensors.gforce");                                       // G-force
    sk_output_trim_angle = new SKOutputFloat("navigation.attitude.trim");                          // Trim angle (based on pitch)

    // Start networking, SK server connections, and other SensESP internals
    sensesp_app->start();
}

void loop()
{
    // Read accelerometer, gyroscope, and magnetometer data
    lsm.read(); // Reads all sensor data (accel, gyro, mag)

    // Scale accelerometer data to g
    float accelX = lsm.accelData.x * (2.0 / 32768.0);
    float accelY = lsm.accelData.y * (2.0 / 32768.0);
    float accelZ = lsm.accelData.z * (2.0 / 32768.0);

    // Convert accelerometer to m/s²
    float accelX_ms2 = accelX * 9.81;
    float accelY_ms2 = accelY * 9.81;
    float accelZ_ms2 = accelZ * 9.81;

    // Calculate total acceleration in m/s²
    float total_acceleration = sqrt(accelX_ms2 * accelX_ms2 + accelY_ms2 * accelY_ms2 + accelZ_ms2 * accelZ_ms2);

    // Scale magnetometer data to gauss
    float magX = lsm.magData.x * (4.0 / 32768.0);
    float magY = lsm.magData.y * (4.0 / 32768.0);
    float magZ = lsm.magData.z * (4.0 / 32768.0);

    // Calculate magnetic heading (in degrees)
    float magnetic_heading = atan2(magY, magX) * (180.0 / PI);
    if (magnetic_heading < 0)
    {
        magnetic_heading += 360; // Ensure heading is in the range [0, 360]
    }

    // Calculate pitch and roll (based on accelerometer data)
    float pitch = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * (180.0 / PI);
    float roll = atan2(accelX, sqrt(accelY * accelY + accelZ * accelZ)) * (180.0 / PI);

    // Calculate yaw (magnetic heading, converted to radians)
    float yaw = atan2(magY, magX) * (PI / 180.0); // Convert to radians

    // Normalize yaw to 0-2PI radians
    if (yaw < 0)
    {
        yaw += 2 * PI;
    }

    // Scale gyroscope data to degrees per second (DPS)
    float gyroX = lsm.gyroData.x * (245.0 / 32768.0);
    float gyroY = lsm.gyroData.y * (245.0 / 32768.0);
    float gyroZ = lsm.gyroData.z * (245.0 / 32768.0);

    // Calculate total angular velocity (magnitude) in DPS
    float total_angular_velocity = sqrt(gyroX * gyroX + gyroY * gyroY + gyroZ * gyroZ);

    // Heel Angle (equivalent to roll)
    float heel_angle = roll;

    // Rate of Turn (based on gyroscope Z-axis)
    float rate_of_turn = gyroZ;

    // Lateral and Longitudinal Acceleration (after adjusting for boat's frame of reference)
    float lateral_acceleration = accelY_ms2;
    float longitudinal_acceleration = accelX_ms2;

    // Pitch and Roll Rates
    float pitch_rate = gyroY;
    float roll_rate = gyroX;

    // True Heading (add magnetic declination)
    float declination = 13.0; // Replace X with the magnetic declination for your location
    float true_heading = magnetic_heading + declination;

    // Normalize true heading to 0-360 degrees
    if (true_heading >= 360)
        true_heading -= 360;
    if (true_heading < 0)
        true_heading += 360;

    // G-Force (total acceleration divided by gravity)
    float g_force = total_acceleration / 9.81;

    // Trim Angle (equivalent to pitch)
    float trim_angle = pitch;

    // Send yaw, pitch, and roll as an object to SignalK path "navigation.attitude"
    sk_output_attitude->set_input({yaw, pitch, roll});
    // Send yaw, pitch, and roll to SignalK paths
    sk_output_yaw->set_input(magnetic_heading); // Update heading based on magnetometer
    sk_output_pitch->set_input(pitch);          // Send pitch
    sk_output_roll->set_input(roll);            // Send roll

    // Send accelerometer data to SignalK paths
    sk_output_accel_x->set_input(accelX_ms2); // Send accelerometer X in m/s²
    sk_output_accel_y->set_input(accelY_ms2); // Send accelerometer Y in m/s²
    sk_output_accel_z->set_input(accelZ_ms2); // Send accelerometer Z in m/s²

    // Send magnetometer data to SignalK paths
    sk_output_mag_x->set_input(magX);
    sk_output_mag_y->set_input(magY);
    sk_output_mag_z->set_input(magZ);

    // Send user-friendly values to SignalK
    sk_output_total_acceleration->set_input(total_acceleration);               // Send total acceleration
    sk_output_magnetic_heading->set_input(magnetic_heading);                   // Send magnetic heading
    sk_output_total_angular_velocity->set_input(total_angular_velocity);       // Send total angular velocity
    sk_output_heel_angle->set_input(heel_angle);                               // Send heel angle
    sk_output_rate_of_turn->set_input(rate_of_turn);                           // Send rate of turn
    sk_output_lateral_acceleration->set_input(lateral_acceleration);           // Send lateral acceleration
    sk_output_longitudinal_acceleration->set_input(longitudinal_acceleration); // Send longitudinal acceleration
    sk_output_pitch_rate->set_input(pitch_rate);                               // Send pitch rate
    sk_output_roll_rate->set_input(roll_rate);                                 // Send roll rate
    sk_output_true_heading->set_input(true_heading);                           // Send true heading
    sk_output_g_force->set_input(g_force);                                     // Send G-force
    sk_output_trim_angle->set_input(trim_angle);                               // Send trim angle

    // Debug output for user-friendly values
    Serial.print("Total Acceleration (m/s²): ");
    Serial.print(total_acceleration);
    Serial.print("\tMagnetic Heading (degrees): ");
    Serial.print(magnetic_heading);
    Serial.print("\tTrue Heading (degrees): ");
    Serial.print(true_heading);
    Serial.print("\tTotal Angular Velocity (DPS): ");
    Serial.println(total_angular_velocity);
    Serial.print("\tPitch (degrees): ");
    Serial.print(pitch);
    Serial.print("\tRoll (degrees): ");
    Serial.println(roll);
    Serial.print("\tHeel Angle (degrees): ");
    Serial.print(heel_angle);
    Serial.print("\tRate of Turn (DPS): ");
    Serial.print(rate_of_turn);
    Serial.print("\tG-Force: ");
    Serial.print(g_force);
    Serial.print("\tTrim Angle (degrees): ");
    Serial.println(trim_angle);

    app.tick();
}