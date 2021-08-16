#include "mbed.h"
#include "MPU-6050.h"

static BufferedSerial serial_port(USBTX, USBRX, 230400);

MPU6050 mpu(PF_0, PF_1);

int16_t ax, ay, az;
int16_t gx, gy, gz;

int main()
{
  printf("MPU6050 test\n\n");
  printf("MPU6050 initialize \n");

  mpu.initialize();

  printf("MPU6050 testConnection \n");

  bool mpu6050TestResult = mpu.testConnection();
  if (mpu6050TestResult)
  {
    printf("MPU6050 test passed \n");
  }
  else
  {
    printf("MPU6050 test failed \n");
  }

  printf("Mode: %d, Gyro: %d, Aceel: %d\n", mpu.getDHPFMode(), mpu.getFullScaleGyroRange(), mpu.getFullScaleAccelRange());

  for (int i = 0; i < 20; i++)
  {
    ThisThread::sleep_for(200ms);
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    //writing current accelerometer and gyro position
    // printf("%d;%d;%d;%d;%d;%d\n", ax, ay, az, gx, gy, gz);
    mpu.getRotation(&gx, &gy, &gz);
    printf("Rotation: %f, %f, %f\n", gx/131.0, gy/131.0, gz/131.0);
  }
}