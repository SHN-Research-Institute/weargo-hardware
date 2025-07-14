
#include <Arduino.h>
#include <Arduino_BMI270_BMM150.h> // Make sure this library is in your platformio.ini: arduino-libraries/Arduino_BMI270_BMM150

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;

  if (!IMU.begin())
  {
    Serial.println("Failed to initialize BMI270/BMM150 IMU!");
    while (1)
      ;
  }
  Serial.println("BMI270/BMM150 IMU initialized!");
}

void loop()
{
  float ax = 0, ay = 0, az = 0;
  float gx = 0, gy = 0, gz = 0;
  float mx = 0, my = 0, mz = 0;

  if (IMU.accelerationAvailable())
    IMU.readAcceleration(ax, ay, az);
  if (IMU.gyroscopeAvailable())
    IMU.readGyroscope(gx, gy, gz);
  if (IMU.magneticFieldAvailable())
    IMU.readMagneticField(mx, my, mz);

  // Output as CSV: ax,ay,az,gx,gy,gz,mx,my,mz\n
  Serial.print(ax, 4);
  Serial.print(",");
  Serial.print(ay, 4);
  Serial.print(",");
  Serial.print(az, 4);
  Serial.print(",");
  Serial.print(gx, 4);
  Serial.print(",");
  Serial.print(gy, 4);
  Serial.print(",");
  Serial.print(gz, 4);
  Serial.print(",");
  Serial.print(mx, 4);
  Serial.print(",");
  Serial.print(my, 4);
  Serial.print(",");
  Serial.println(mz, 4);

  delay(50);
}
