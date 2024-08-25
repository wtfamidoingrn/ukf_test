#include "Arduino.h"
#include "SoftwareSerial.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU9250.h"
#include "konfig.h"
#include "Servo.h"

SoftwareSerial SoftSerial(2, 3);
unsigned char buffer_gps[64];
int gps_count = 0;

unsigned char incoming_buffer[64];
int incoming_count = 0;

Servo ServoPitch;
#define pin_pitch 5
int pitch_degrees = 100;

Servo ServoYaw;
#define pin_yaw 3
int yaw_degrees = 0;

MPU9250 accelerationGyro;

I2Cdev I2C_M;
uint8_t buffer_m[6];

int16_t ax, ay, az;

int16_t gx, gy, gz;
int16_t mx, my, mz;
float heading;

float pitch;
float roll;
float tiltheading;
float Axyz[3];

float Gxyz[3];
float Mxyz[3];
float Grav[3];

float GyrBias[3];

#define GYRO_SAMPLES 50
#define GRAV_SAMPLES 1
#define sample_num_mdate  5000
#define send_frequency 50

volatile float mx_sample[3];
volatile float my_sample[3];
volatile float mz_sample[3];

static float mx_centre = 0;
static float my_centre = 0;
static float mz_centre = 0;

volatile int mx_max = 0;
volatile int my_max = 0;
volatile int mz_max = 0;

volatile int mx_min = 0;
volatile int my_min = 0;
volatile int mz_min = 0;

void clearGpsBufferArray() {
    for (int i = 0; i < gps_count; i++) {
        buffer_gps[i] = NULL;
    }
}

void getCompass_Data(void) {
    I2C_M.writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01); //enable the magnetometer
    delay(10);
    I2C_M.readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6, buffer_m);

    mx = ((int16_t) (buffer_m[1]) << 8) | buffer_m[0];
    my = ((int16_t) (buffer_m[3]) << 8) | buffer_m[2];
    mz = ((int16_t) (buffer_m[5]) << 8) | buffer_m[4];

    Mxyz[0] = (double) mx * 1200 / 4096;
    Mxyz[1] = (double) my * 1200 / 4096;
    Mxyz[2] = (double) mz * 1200 / 4096;
}

void getHeading(void) {
    heading = 180 * atan2(Mxyz[1], Mxyz[0]) / PI;
    if (heading < 0) heading += 360;
}

void getTiltHeading(void) {
    pitch = asin(-Axyz[0]);
    roll = asin(Axyz[1] / cos(pitch));

    float xh = Mxyz[0] * cos(pitch) + Mxyz[2] * sin(pitch);
    float yh = Mxyz[0] * sin(roll) * sin(pitch) + Mxyz[1] * cos(roll) - Mxyz[2] * sin(roll) * cos(pitch);
    float zh = -Mxyz[0] * cos(roll) * sin(pitch) + Mxyz[1] * sin(roll) + Mxyz[2] * cos(roll) * cos(pitch);
    tiltheading = 180 * atan2(yh, xh) / PI;
    if (yh < 0) tiltheading += 360;
}

void get_one_sample_date_mxyz() {
    getCompass_Data();
    mx_sample[2] = Mxyz[0];
    my_sample[2] = Mxyz[1];
    mz_sample[2] = Mxyz[2];
}

void get_calibration_Data() {
    for (int i = 0; i < sample_num_mdate; i++) {
        get_one_sample_date_mxyz();
        /*
        Serial.print(mx_sample[2]);
        Serial.print(" ");
        Serial.print(my_sample[2]);                            //you can see the sample data here .
        Serial.print(" ");
        Serial.println(mz_sample[2]);
        */



        if (mx_sample[2] >= mx_sample[1])mx_sample[1] = mx_sample[2];
        if (my_sample[2] >= my_sample[1])my_sample[1] = my_sample[2]; //find max value
        if (mz_sample[2] >= mz_sample[1])mz_sample[1] = mz_sample[2];

        if (mx_sample[2] <= mx_sample[0])mx_sample[0] = mx_sample[2];
        if (my_sample[2] <= my_sample[0])my_sample[0] = my_sample[2]; //find min value
        if (mz_sample[2] <= mz_sample[0])mz_sample[0] = mz_sample[2];

    }

    mx_max = mx_sample[1];
    my_max = my_sample[1];
    mz_max = mz_sample[1];

    mx_min = mx_sample[0];
    my_min = my_sample[0];
    mz_min = mz_sample[0];


    mx_centre = (mx_max + mx_min) / 2;
    my_centre = (my_max + my_min) / 2;
    mz_centre = (mz_max + mz_min) / 2;

}

void getAccel_Data(void) {
    accelerationGyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    Axyz[0] = (double) ax / 16384;
    Axyz[1] = (double) ay / 16384;
    Axyz[2] = (double) az / 16384;
}

void getGyro_Data(void) {
    accelerationGyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    Gxyz[0] = (double) gx * 250 / 32768;
    Gxyz[1] = (double) gy * 250 / 32768;
    Gxyz[2] = (double) gz * 250 / 32768;
}


void getCompassDate_calibrated() {
    getCompass_Data();
    Mxyz[0] = Mxyz[0] - mx_centre;
    Mxyz[1] = Mxyz[1] - my_centre;
    Mxyz[2] = Mxyz[2] - mz_centre;
}

/* Function to interface with the Processing script in the PC */
void serialFloatPrint(float f) {
    byte *b = (byte *) &f;
    for (int i = 0; i < 4; i++) {
        byte b1 = (b[i] >> 4) & 0x0f;
        byte b2 = (b[i] & 0x0f);

        char c1 = (b1 < 10) ? ('0' + b1) : 'A' + b1 - 10;
        char c2 = (b2 < 10) ? ('0' + b2) : 'A' + b2 - 10;

        Serial.print(c1);
        Serial.print(c2);
    }
}

void calibrateGravitation() {
    for (int i = 1; i <= GRAV_SAMPLES; ++i) {
        getAccel_Data();
        Grav[0] = Grav[0] * (i - 1) / i + Axyz[0] / i;
        Grav[1] = Grav[1] * (i - 1) / i + Axyz[1] / i;
        Grav[2] = Grav[2] * (i - 1) / i + Axyz[2] / i;
    }
}

void calibrateGyroscope() {
    for (int i = 1; i <= GYRO_SAMPLES; ++i) {
        getGyro_Data();
        GyrBias[0] = GyrBias[0] * (i - 1) / i + Gxyz[0] / i;
        GyrBias[1] = GyrBias[1] * (i - 1) / i + Gxyz[1] / i;
        GyrBias[2] = GyrBias[2] * (i - 1) / i + Gxyz[2] / i;
    }
}

void setup() {
    /* Serial initialization -------------------------------------- */
    Wire.begin();
    Serial.begin(57600);
    // Set this to the last configured Baud rate
    SoftSerial.begin(9600);
    Serial.println("Calibrating IMU bias...");

    /* IMU initialization ----------------------------------------- */
    /* initzialize IMU */
    accelerationGyro.initialize();
//    accelerationGyro.setFullScaleAccelRange(1);
    accelerationGyro.setFullScaleGyroRange(3);
    //verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelerationGyro.testConnection() ? "MPU9250 Connection successful" : "MPU9250 connection failed");
    calibrateGyroscope();
    calibrateGravitation();
    Serial.print("Gra:");
    Serial.print(Grav[0]);
    Serial.print(",");
    Serial.print(Grav[1]);
    Serial.print(",");
    Serial.println(Grav[2]);

    // baud rate 57600
    SoftSerial.write("$PMTK251,38400*27\r\n");
    SoftSerial.begin(38400);
    // only GSV
    SoftSerial.write("$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n");
    // $PMTK869,1,0*34
    SoftSerial.write("$PMTK869,1,0*34\r\n");
    //
    SoftSerial.write("$PMTK220,100*2F\r\n");

    ServoPitch.attach(pin_pitch);
    ServoYaw.attach(pin_yaw);
    ServoPitch.write(110);
    ServoYaw.write(90);
}


void loop() {
    if (Serial.available() > 0) {
        while (Serial.available()) {
            incoming_buffer[incoming_count++] = Serial.read();
        }
        bool yaw_written = false;
        bool pitch_written = false;
        for (int i = 0; i < incoming_count; i++) {
            unsigned char incomingByte = incoming_buffer[i];
            incoming_buffer[i] = 0;
            // Check for control character
            if (incomingByte > 13) {
                // remap [14;104] to [0;90]
                incomingByte -= 14;

                // Check first bit to decide which servo to take
                bool yawbit = false;
                if (((incomingByte >> 7) & 1) > 0) {
                    yawbit = true;
                }

                // set first bit to 0
                incomingByte &= ~(1 << 7);

                //remap [0;90] to [0;180]
                int degrees = incomingByte * 2;

                // decide which servo to take
                if (yawbit) {
                    ServoYaw.write(degrees);
                    yaw_written = true;
                    delay(20);
                    Serial.print("Yaw:");
                    Serial.print(degrees);
                } else {
                    ServoPitch.write(90 + degrees);
                    pitch_written = true;
                    delay(20);
                    Serial.print("Pitch:");
                    Serial.print(90 + degrees);
                }
            }

            if (yaw_written && pitch_written) {
                for (int j = i; j < incoming_count; j++){
                    incoming_buffer[j] = 0;
                }
                break;
            }
        }
        incoming_count = 0;
   }

    getAccel_Data();
    getGyro_Data();
    getCompassDate_calibrated();
    getHeading();
    getTiltHeading();

    Serial.print("Acc:");
    Serial.print(Axyz[0], 15);
    Serial.print(",");
    Serial.print(Axyz[1], 15);
    Serial.print(",");
    Serial.print(Axyz[2], 15);

    Serial.print("Gyr:");
    Serial.print(Gxyz[0] - GyrBias[0], 15);
    Serial.print(",");
    Serial.print(Gxyz[1] - GyrBias[1], 15);
    Serial.print(",");
    Serial.print(Gxyz[2] - GyrBias[2], 15);

    Serial.print("Mag:");
    Serial.print(Mxyz[0], 15);
    Serial.print(",");
    Serial.print(Mxyz[1], 15);
    Serial.print(",");
    Serial.print(Mxyz[2], 15);

    Serial.print("GPS:");
    if (SoftSerial.available()) {
        while (SoftSerial.available()) {
            buffer_gps[gps_count++] = SoftSerial.read();
            if (gps_count == 64) break;
        }
        Serial.write(buffer_gps, gps_count);
        clearGpsBufferArray();
        gps_count = 0;

    }
    Serial.println();
}