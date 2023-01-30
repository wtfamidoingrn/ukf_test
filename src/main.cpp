#include "Arduino.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU9250.h"
#include "elapsedMillis.h"
#include "konfig.h"
#include "matrix.h"
#include "ukf.h"

MPU9250 accelerationGyro;
I2Cdev I2C_M;

uint8_t buffer_m[6];

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

float heading;
float tiltheading;

float Axyz[3];
float Gxyz[3];
float Mxyz[3];

#define sample_num_mdate  5000

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

/* ================================================== The AHRS/IMU variables ================================================== */
/* Gravity vector constant (align with global Z-axis) */
#define IMU_ACC_Z0          (1)
/* Magnetic vector constant (align with local magnetic vector) */
float_prec IMU_MAG_B0_data[3] = {static_cast<float>(cos(0)), static_cast<float>(sin(0)), 0.000000};
Matrix IMU_MAG_B0(3, 1, IMU_MAG_B0_data);
/* The hard-magnet bias */
float_prec HARD_IRON_BIAS_data[3] = {8.832973, 7.243323, 23.95714};
Matrix HARD_IRON_BIAS(3, 1, HARD_IRON_BIAS_data);



/* ============================================ UKF variables/function declaration ============================================ */
/* Just example; in konfig.h:
 *  SS_X_LEN = 4
 *  SS_Z_LEN = 6
 *  SS_U_LEN = 3
 */
/* UKF initialization constant -------------------------------------------------------------------------------------- */
#define P_INIT      (10.)
#define Rv_INIT     (1e-6)
#define Rn_INIT_ACC (0.0015)
#define Rn_INIT_MAG (0.0015)
/* P(k=0) variable -------------------------------------------------------------------------------------------------- */
float_prec UKF_PINIT_data[SS_X_LEN*SS_X_LEN] = {P_INIT, 0,      0,      0,
                                                0,      P_INIT, 0,      0,
                                                0,      0,      P_INIT, 0,
                                                0,      0,      0,      P_INIT};
Matrix UKF_PINIT(SS_X_LEN, SS_X_LEN, UKF_PINIT_data);
/* Q constant ------------------------------------------------------------------------------------------------------- */
float_prec UKF_RVINIT_data[SS_X_LEN*SS_X_LEN] = {Rv_INIT, 0,      0,      0,
                                                 0,      Rv_INIT, 0,      0,
                                                 0,      0,      Rv_INIT, 0,
                                                 0,      0,      0,      Rv_INIT};
Matrix UKF_RvINIT(SS_X_LEN, SS_X_LEN, UKF_RVINIT_data);
/* R constant ------------------------------------------------------------------------------------------------------- */
float_prec UKF_RNINIT_data[SS_Z_LEN*SS_Z_LEN] = {Rn_INIT_ACC, 0,          0,          0,          0,          0,
                                                 0,          Rn_INIT_ACC, 0,          0,          0,          0,
                                                 0,          0,          Rn_INIT_ACC, 0,          0,          0,
                                                 0,          0,          0,          Rn_INIT_MAG, 0,          0,
                                                 0,          0,          0,          0,          Rn_INIT_MAG, 0,
                                                 0,          0,          0,          0,          0,          Rn_INIT_MAG};
Matrix UKF_RnINIT(SS_Z_LEN, SS_Z_LEN, UKF_RNINIT_data);
/* Nonlinear & linearization function ------------------------------------------------------------------------------- */
bool Main_bUpdateNonlinearX(Matrix& X_Next, const Matrix& X, const Matrix& U);
bool Main_bUpdateNonlinearY(Matrix& Y, const Matrix& X, const Matrix& U);


/* UKF variables ---------------------------------------------------------------------------------------------------- */
Matrix quaternionData(SS_X_LEN, 1);
Matrix Y(SS_Z_LEN, 1);
Matrix U(SS_U_LEN, 1);
/* UKF system declaration ------------------------------------------------------------------------------------------- */
UKF UKF_IMU(quaternionData, UKF_PINIT, UKF_RvINIT, UKF_RnINIT, Main_bUpdateNonlinearX, Main_bUpdateNonlinearY);



/* ========================================= Auxiliary variables/function declaration ========================================= */
elapsedMillis timerLed, timerUKF;
uint64_t u64compuTime;
char bufferTxSer[100];
/* The command from the PC */
char cmd;


void getCompass_Data(void)
{
    I2C_M.writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01); //enable the magnetometer
    delay(10);
    I2C_M.readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6, buffer_m);

    mx = ((int16_t)(buffer_m[1]) << 8) | buffer_m[0] ;
    my = ((int16_t)(buffer_m[3]) << 8) | buffer_m[2] ;
    mz = ((int16_t)(buffer_m[5]) << 8) | buffer_m[4] ;

    Mxyz[0] = (double) mx * 1200 / 4096;
    Mxyz[1] = (double) my * 1200 / 4096;
    Mxyz[2] = (double) mz * 1200 / 4096;
}

void getHeading(void)
{
    heading = 180 * atan2(Mxyz[1], Mxyz[0]) / PI;
    if (heading < 0) heading += 360;
}

void getTiltHeading(void)
{
    float pitch = asin(-Axyz[0]);
    float roll = asin(Axyz[1] / cos(pitch));

    float xh = Mxyz[0] * cos(pitch) + Mxyz[2] * sin(pitch);
    float yh = Mxyz[0] * sin(roll) * sin(pitch) + Mxyz[1] * cos(roll) - Mxyz[2] * sin(roll) * cos(pitch);
    float zh = -Mxyz[0] * cos(roll) * sin(pitch) + Mxyz[1] * sin(roll) + Mxyz[2] * cos(roll) * cos(pitch);
    tiltheading = 180 * atan2(yh, xh) / PI;
    if (yh < 0)    tiltheading += 360;
}

void get_one_sample_date_mxyz()
{
    getCompass_Data();
    mx_sample[2] = Mxyz[0];
    my_sample[2] = Mxyz[1];
    mz_sample[2] = Mxyz[2];
}

void get_calibration_Data ()
{
    for (int i = 0; i < sample_num_mdate; i++)
    {
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

void getAccel_Data(void)
{
    accelerationGyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    Axyz[0] = (double) ax / 16384;
    Axyz[1] = (double) ay / 16384;
    Axyz[2] = (double) az / 16384;
}

void getGyro_Data(void)
{
    accelerationGyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    Gxyz[0] = (double) gx * 250 / 32768;
    Gxyz[1] = (double) gy * 250 / 32768;
    Gxyz[2] = (double) gz * 250 / 32768;
}


void getCompassDate_calibrated ()
{
    getCompass_Data();
    Mxyz[0] = Mxyz[0] - mx_centre;
    Mxyz[1] = Mxyz[1] - my_centre;
    Mxyz[2] = Mxyz[2] - mz_centre;
}

/* Function to interface with the Processing script in the PC */
void serialFloatPrint(float f) {
    byte * b = (byte *) &f;
    for (int i = 0; i < 4; i++) {
        byte b1 = (b[i] >> 4) & 0x0f;
        byte b2 = (b[i] & 0x0f);

        char c1 = (b1 < 10) ? ('0' + b1) : 'A' + b1 - 10;
        char c2 = (b2 < 10) ? ('0' + b2) : 'A' + b2 - 10;

        Serial.print(c1);
        Serial.print(c2);
    }
}

void setup() {
    /* Serial initialization -------------------------------------- */
    Wire.begin();
    Serial.begin(38400);
    Serial.println("Calibrating IMU bias...");
    delay(500);
    Serial.println("Calibrating IMU bias...");
    delay(500);
    Serial.println("Calibrating IMU bias...");
    delay(500);
    Serial.println("Calibrating IMU bias...");
    delay(500);
    Serial.println("Calibrating IMU bias...");
    delay(500);
    Serial.println("Calibrating IMU bias...");

    /* IMU initialization ----------------------------------------- */
    /* initzialize IMU */
    accelerationGyro.initialize();

    //verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelerationGyro.testConnection() ? "MPU9250 Connection successful" : "MPU9250 connection failed");

    /* UKF initialization ----------------------------------------- */
    /* x(k=0) = [1 0 0 0]' */
    quaternionData.vSetToZero();
    quaternionData[0][0] = 1.0;
    UKF_IMU.vReset(quaternionData, UKF_PINIT, UKF_RvINIT, UKF_RnINIT);

    snprintf(bufferTxSer, sizeof(bufferTxSer)-1, "UKF in Teensy 4.0 (%s)\r\n",
             (FPU_PRECISION == PRECISION_SINGLE)?"Float32":"Double64");
    Serial.print(bufferTxSer);
}


void loop() {
    getAccel_Data();
    getGyro_Data();
    getCompassDate_calibrated();
    getHeading();
    getTiltHeading();

    if (timerUKF >= SS_DT_MILIS) {
        timerUKF = 0;


        /* ================== Read the sensor data / simulate the system here ================== */
        /* Read the raw data */
        float Ax = Axyz[0];
        float Ay = Axyz[1];
        float Az = Axyz[2];
        float Bx = Mxyz[0];
        float By = Mxyz[1];
        float Bz = Mxyz[2];
        float p = Gxyz[0];
        float q = Gxyz[1];
        float r = Gxyz[2];
//        float p = IMU.getGyroX_rads();
//        float q = IMU.getGyroY_rads();
//        float r = IMU.getGyroZ_rads();
        /* Input 1:3 = gyroscope */
        U[0][0] = p;  U[1][0] = q;  U[2][0] = r;
        /* Output 1:3 = accelerometer */
        Y[0][0] = Ax; Y[1][0] = Ay; Y[2][0] = Az;
        /* Output 4:6 = magnetometer */
        Y[3][0] = Bx; Y[4][0] = By; Y[5][0] = Bz;

        /* Compensating Hard-Iron Bias for magnetometer */
        Y[3][0] = Y[3][0]-HARD_IRON_BIAS[0][0];
        Y[4][0] = Y[4][0]-HARD_IRON_BIAS[1][0];
        Y[5][0] = Y[5][0]-HARD_IRON_BIAS[2][0];

        /* Normalizing the output vector */
        float_prec _normG = sqrt(Y[0][0] * Y[0][0]) + (Y[1][0] * Y[1][0]) + (Y[2][0] * Y[2][0]);
        Y[0][0] = Y[0][0] / _normG;
        Y[1][0] = Y[1][0] / _normG;
        Y[2][0] = Y[2][0] / _normG;
        float_prec _normM = sqrt(Y[3][0] * Y[3][0]) + (Y[4][0] * Y[4][0]) + (Y[5][0] * Y[5][0]);
        Y[3][0] = Y[3][0] / _normM;
        Y[4][0] = Y[4][0] / _normM;
        Y[5][0] = Y[5][0] / _normM;
        /* ------------------ Read the sensor data / simulate the system here ------------------ */


        /* ============================= Update the Kalman Filter ============================== */
        u64compuTime = micros();
        if (!UKF_IMU.bUpdate(Y, U)) {
            quaternionData.vSetToZero();
            quaternionData[0][0] = 1.0;
            UKF_IMU.vReset(quaternionData, UKF_PINIT, UKF_RvINIT, UKF_RnINIT);
            Serial.println("Whoop ");
        }
        u64compuTime = (micros() - u64compuTime);
        /* ----------------------------- Update the Kalman Filter ------------------------------ */
    }


    /* The serial data is sent by responding to command from the PC running Processing scipt */
    if (Serial.available()) {
        cmd = Serial.read();
        if (cmd == 'v') {
            snprintf(bufferTxSer, sizeof(bufferTxSer)-1, "UKF in Teensy 4.0 (%s)\r\n",
                     (FPU_PRECISION == PRECISION_SINGLE)?"Float32":"Double64");
            Serial.print(bufferTxSer);
            Serial.print('\n');
        } else if (cmd == 'q') {
            /* =========================== Print to serial (for plotting) ========================== */
            quaternionData = UKF_IMU.GetX();

            while (!Serial.available());
            uint8_t count = Serial.read();
            for (uint8_t _i = 0; _i < count; _i++) {
                serialFloatPrint(quaternionData[0][0]);
                Serial.print(",");
                serialFloatPrint(quaternionData[1][0]);
                Serial.print(",");
                serialFloatPrint(quaternionData[2][0]);
                Serial.print(",");
                serialFloatPrint(quaternionData[3][0]);
                Serial.print(",");
                serialFloatPrint((float) u64compuTime);
                Serial.print(",");
                Serial.println("");
            }
            /* --------------------------- Print to serial (for plotting) -------------------------- */
        }
    }
}

bool Main_bUpdateNonlinearX(Matrix& X_Next, const Matrix& X, const Matrix& U)
{
    /* Insert the nonlinear update transformation here
     *          x(k+1) = f[x(k), u(k)]
     *
     * The quaternion update function:
     *  q0_dot = 1/2. * (  0   - p*q1 - q*q2 - r*q3)
     *  q1_dot = 1/2. * ( p*q0 +   0  + r*q2 - q*q3)
     *  q2_dot = 1/2. * ( q*q0 - r*q1 +  0   + p*q3)
     *  q3_dot = 1/2. * ( r*q0 + q*q1 - p*q2 +  0  )
     *
     * Euler method for integration:
     *  q0 = q0 + q0_dot * dT;
     *  q1 = q1 + q1_dot * dT;
     *  q2 = q2 + q2_dot * dT;
     *  q3 = q3 + q3_dot * dT;
     */
    float_prec q0, q1, q2, q3;
    float_prec p, q, r;

    q0 = X[0][0];
    q1 = X[1][0];
    q2 = X[2][0];
    q3 = X[3][0];

    p = U[0][0];
    q = U[1][0];
    r = U[2][0];

    X_Next[0][0] = (0.5 * (+0.00 -p*q1 -q*q2 -r*q3))*SS_DT + q0;
    X_Next[1][0] = (0.5 * (+p*q0 +0.00 +r*q2 -q*q3))*SS_DT + q1;
    X_Next[2][0] = (0.5 * (+q*q0 -r*q1 +0.00 +p*q3))*SS_DT + q2;
    X_Next[3][0] = (0.5 * (+r*q0 +q*q1 -p*q2 +0.00))*SS_DT + q3;


    /* ======= Additional ad-hoc quaternion normalization to make sure the quaternion is a unit vector (i.e. ||q|| = 1) ======= */
    if (!X_Next.bNormVector()) {
        /* System error, return false, so we can reset the UKF */
        return false;
    }

    return true;
}

bool Main_bUpdateNonlinearY(Matrix& Y, const Matrix& X, const Matrix& U)
{
    /* Insert the nonlinear measurement transformation here
     *          y(k)   = h[x(k), u(k)]
     *
     * The measurement output is the gravitational and magnetic projection to the body:
     *     DCM     = [(+(q0**2)+(q1**2)-(q2**2)-(q3**2)),                    2*(q1*q2+q0*q3),                    2*(q1*q3-q0*q2)]
     *               [                   2*(q1*q2-q0*q3), (+(q0**2)-(q1**2)+(q2**2)-(q3**2)),                    2*(q2*q3+q0*q1)]
     *               [                   2*(q1*q3+q0*q2),                    2*(q2*q3-q0*q1), (+(q0**2)-(q1**2)-(q2**2)+(q3**2))]
     *
     *  G_proj_sens = DCM * [0 0 1]             --> Gravitational projection to the accelerometer sensor
     *  M_proj_sens = DCM * [Mx My Mz]          --> (Earth) magnetic projection to the magnetometer sensor
     */
    float_prec q0, q1, q2, q3;
    float_prec q0_2, q1_2, q2_2, q3_2;

    q0 = X[0][0];
    q1 = X[1][0];
    q2 = X[2][0];
    q3 = X[3][0];

    q0_2 = q0 * q0;
    q1_2 = q1 * q1;
    q2_2 = q2 * q2;
    q3_2 = q3 * q3;

    Y[0][0] = (2*q1*q3 -2*q0*q2) * IMU_ACC_Z0;

    Y[1][0] = (2*q2*q3 +2*q0*q1) * IMU_ACC_Z0;

    Y[2][0] = (+(q0_2) -(q1_2) -(q2_2) +(q3_2)) * IMU_ACC_Z0;

    Y[3][0] = (+(q0_2)+(q1_2)-(q2_2)-(q3_2)) * IMU_MAG_B0[0][0]
              +(2*(q1*q2+q0*q3)) * IMU_MAG_B0[1][0]
              +(2*(q1*q3-q0*q2)) * IMU_MAG_B0[2][0];

    Y[4][0] = (2*(q1*q2-q0*q3)) * IMU_MAG_B0[0][0]
              +(+(q0_2)-(q1_2)+(q2_2)-(q3_2)) * IMU_MAG_B0[1][0]
              +(2*(q2*q3+q0*q1)) * IMU_MAG_B0[2][0];

    Y[5][0] = (2*(q1*q3+q0*q2)) * IMU_MAG_B0[0][0]
              +(2*(q2*q3-q0*q1)) * IMU_MAG_B0[1][0]
              +(+(q0_2)-(q1_2)-(q2_2)+(q3_2)) * IMU_MAG_B0[2][0];

    return true;
}





void SPEW_THE_ERROR(char const * str)
{
#if (SYSTEM_IMPLEMENTATION == SYSTEM_IMPLEMENTATION_PC)
    cout << (str) << endl;
#elif (SYSTEM_IMPLEMENTATION == SYSTEM_IMPLEMENTATION_EMBEDDED_ARDUINO)
    Serial.println(str);
#else
    /* Silent function */
#endif
    while(1);
}