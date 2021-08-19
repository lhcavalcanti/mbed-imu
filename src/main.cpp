#include "mbed.h"
#include "MPU6050.h"
#include <math.h>
#define pi 3.141592654

static BufferedSerial serial_port(USBTX, USBRX, 230400);

MPU6050 AccGyro(PF_0, PF_1); // Create an MPU object called AccGyro

double Ax, Ay, Az, Gx, Gy, Gz;
double Ax_f, Ay_f, Az_f;
double Gx_f, Gy_f, Gz_f;
double Ax_f_sum, Ay_f_sum, Az_f_sum, Gx_f_sum, Gy_f_sum, Gz_f_sum;
double roll, pitch, yaw;

int main()
{

    double AccelReadings[3] = {0, 0, 0};
    double GyroReadings[3] = {0, 0, 0};

    AccGyro.initialize();

    //     while (true)
    // {

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

            // 2s complement acclerometer and gyroscope values
            Ax = AccelReadings[0];
            Ay = AccelReadings[1];
            Az = AccelReadings[2];
            Gx = GyroReadings[0];
            Gy = GyroReadings[1];
            Gz = GyroReadings[2];

            // Add every reading to the sum variables
            Ax_f_sum = Ax_f_sum + (double)Ax;
            Ay_f_sum = Ay_f_sum + (double)Ay;
            Az_f_sum = Az_f_sum + (double)Az;
            Gx_f_sum = Gx_f_sum + (double)Gx;
            Gy_f_sum = Gy_f_sum + (double)Gy;
            Gz_f_sum = Gz_f_sum + (double)Gz;
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

        // Convert gyroscope readings into degrees/s
        // Gx_f = Gx_f / 16.4;
        // Gy_f = Gy_f / 16.4;
        // Gz_f = Gz_f / 16.4;

        // printf("Gyro(deg/s) X: %.3f Y: %.3f Z: %.3f || Accel(deg) Roll: %.3f, Pitch: %.3f, Yaw: %.3f \n", Gx_f, Gy_f, Gz_f, roll, pitch, yaw);
        printf("Gyro(deg/s) X: %.3lf Y: %.3lf Z: %.3lf \n", Gx_f, Gy_f, Gz_f);
        printf("Accel(deg) X: %.3lf Y: %.3lf Z: %.3lf |  X: %.3lf Y: %.3lf Z: %.3lf \n", Ax_f, Ay_f, Az_f, Ax_f / 2048.0, Ay_f / 2048.0, Az_f / 2048.0);
        printf("DEBUG: Accel(deg) X: %.3lf Y: %.3lf Z: %.3lf \n", Ax_f, Ay_f, Az_f);
    // }
}
