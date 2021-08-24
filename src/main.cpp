#include "mbed.h"
#include "MPU6050.h"
#include <math.h>
#define pi 3.141592654

static BufferedSerial serial_port(USBTX, USBRX, 230400);

MPU6050 AccGyro(PF_0, PF_1); // Create an MPU object called AccGyro

double Ax_f, Ay_f, Az_f;
double Gx_f, Gy_f, Gz_f;
double Ax_f_sum, Ay_f_sum, Az_f_sum, Gx_f_sum, Gy_f_sum, Gz_f_sum;
double roll, pitch, yaw;

int main()
{

    double AccelReadings[3] = {0, 0, 0};
    double GyroReadings[3] = {0, 0, 0};

    AccGyro.initialize();

    while (true)
    {

        ThisThread::sleep_for(200ms);

        Ax_f_sum = 0;
        Ay_f_sum = 0;
        Az_f_sum = 0;
        Gx_f_sum = 0;
        Gy_f_sum = 0;
        Gz_f_sum = 0;

        for (int i = 0; i < 10; i = i + 1) // Take ten analog input readings
        {
            AccGyro.readAccel(AccelReadings); // Extract accelerometer measurements
            AccGyro.readGyro(GyroReadings);   // Extract gyroscope measurements


            // Add every reading to the sum variables
            Ax_f_sum = Ax_f_sum + AccelReadings[0];
            Ay_f_sum = Ay_f_sum + AccelReadings[1];
            Az_f_sum = Az_f_sum + AccelReadings[2];
            Gx_f_sum = Gx_f_sum + GyroReadings[0];
            Gy_f_sum = Gy_f_sum + GyroReadings[1];
            Gz_f_sum = Gz_f_sum + GyroReadings[2];
        }

        // Divide by 10 to get the averaged value
        Ax_f = Ax_f_sum / 10.0;
        Ay_f = Ay_f_sum / 10.0;
        Az_f = Az_f_sum / 10.0;
        Gx_f = Gx_f_sum / 10.0;
        Gy_f = Gy_f_sum / 10.0;
        Gz_f = Gz_f_sum / 10.0;

        // 1. Calculate actual roll, pitch and yaw angles in degrees
        // 2. Calibrate readings by adding or substracting the off-set
        roll = (180 / pi) * (atan(Ax_f / (sqrt((Ay_f * Ay_f) + (Az_f * Az_f))))) - 4.36;
        pitch = (180 / pi) * (atan(Ay_f / (sqrt((Ax_f * Ax_f) + (Az_f * Az_f))))) - 0.063;
        yaw = (180 / pi) * (atan((sqrt((Ax_f * Ax_f) + (Ay_f * Ay_f))) / Az_f)) - 3.93;


        printf("Gyro(deg/s) X: %.3lf Y: %.3lf Z: %.3lf \n", Gx_f, Gy_f, Gz_f);
        printf("Accel(g) X: %.3lf Y: %.3lf Z: %.3lf \n", Ax_f, Ay_f, Az_f);
        printf("Accel(deg) X: %.3lf Y: %.3lf Z: %.3lf \n", roll, pitch, yaw);
    }
}
