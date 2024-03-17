#include <Arduino_LSM9DS1.h>

// Define a macro to select the sensor type
#define USE_ACCELEROMETER  // Comment or uncomment to switch between sensors
//#define USE_GYROSCOPE  // Uncomment to use gyroscope
//#define USE_MAGNETOMETER  // Uncomment to use magnetometer

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

  // Initialize the LSM9DS1 sensor
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

#ifdef USE_ACCELEROMETER
  // Display accelerometer information
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in g's");
#elif defined(USE_GYROSCOPE)
  // Display gyroscope information
  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Gyroscope in degrees/second");
#elif defined(USE_MAGNETOMETER)
  // Display magnetometer information
  Serial.print("Magnetic field sample rate = ");
  Serial.print(IMU.magneticFieldSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Magnetic Field in uT");
#endif

  Serial.println("X\tY\tZ");
}

void loop() {
  float x, y, z;

#ifdef USE_ACCELEROMETER
  // Read and display accelerometer data
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.println(z);
  }
#elif defined(USE_GYROSCOPE)
  // Read and display gyroscope data
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x, y, z);
    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.println(z);
  }
#elif defined(USE_MAGNETOMETER)
  // Read and display magnetometer data
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(x, y, z);
    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.println(z);
  }
#endif

  delay(500); // Introduce a delay of 500 milliseconds (0.5 seconds)
}
