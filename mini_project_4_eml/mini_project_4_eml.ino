
#include <Arduino_LSM9DS1.h>
#include <TensorFlowLite.h> 
#include "model.h"
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_log.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "constants.h"
#include "main_functions.h"
#include "output_handler.h"


//to give the user inout for activating sensor
int sensor_type = 0;  


// Globals, used for compatibility with Arduino-style sketches.
namespace {
  //model initialization for accelerometer
const tflite::Model* model_acc = nullptr;
tflite::MicroInterpreter* interpreter_acc = nullptr;
TfLiteTensor* input_acc = nullptr;
TfLiteTensor* output_acc = nullptr;
//set the buffer size 
constexpr int kTensorArenaSizeAcc = 4096;
// Keep aligned to 16 bytes for CMSIS
alignas(16) uint8_t tensor_arena_acc[kTensorArenaSizeAcc];

//model initialization for gyroscope
const tflite::Model* model_gyro = nullptr;
tflite::MicroInterpreter* interpreter_gyro = nullptr;
TfLiteTensor* input_gyro= nullptr;
TfLiteTensor* output_gyro = nullptr;
//set the buffer size 
constexpr int kTensorArenaSizeGyro = 4096;
// Keep aligned to 16 bytes for CMSIS
alignas(16) uint8_t tensor_arena_gyro[kTensorArenaSizeGyro];

//model initialization for magnetometer
const tflite::Model* model_mag = nullptr;
tflite::MicroInterpreter* interpreter_mag = nullptr;
TfLiteTensor* input_mag = nullptr;
TfLiteTensor* output_mag = nullptr;
//set the buffer size 
constexpr int kTensorArenaSizeMag = 4096;
// Keep aligned to 16 bytes for CMSIS
alignas(16) uint8_t tensor_arena_mag[kTensorArenaSizeMag];
}  // namespace

// array to declare posture index 
const char* posture[] = {
  "prone",
  "side",
  "sitting",
  "supine",
  "unknown"
};

//sensor data -x,y and z to be stored in the array
float sensor_data[3];

//macro to store the postures data
#define num_posture (sizeof(posture) / sizeof(posture[0]))

//for reading anfd predicting accelerometer data 
void ReadAccelerometer(){
  //to calculate the output max value
    float y;
    int index = -1;
    float max_value =0;
    //Read the accelerometer value if the sensor is available
  if (IMU.accelerationAvailable()) {
    //read the accelerometer data and store it the array
    IMU.readAcceleration(sensor_data[0], sensor_data[1], sensor_data[2]);
    Serial.print("x_a:");
    Serial.print(sensor_data[0]);
    Serial.print('\t');
    Serial.print("y_a:");
    Serial.print(sensor_data[1]);
    Serial.print('\t');
    Serial.print("z_a:");
    Serial.println(sensor_data[2]);
    //store the inout data into the accelerometer inout variable for data to be predicted
    for (int i = 0; i < input_acc->bytes / sizeof(float); i++) {
      // for (int i = 0; i < num_posture; i++) {
      input_acc->data.f[i] = sensor_data[i];
    }
    //delay for the model to read the data
    delay(2000);
    //invoke the interpreter
    TfLiteStatus invoke_status = interpreter_acc->Invoke();
    //sanity check for interpreter
     if (invoke_status != kTfLiteOk) {
        MicroPrintf("Inference failed.");
        return;
     }
    delay(1000);
    for (int i = 0; i < num_posture; i++) {
      y = output_acc->data.f[i];
      if(y > max_value){
        index = i;
        max_value = y;
      }
    }
    Serial.print("Predicted Posture:");
    Serial.println(posture[index]);
    Serial.print("Confidence Score:");
    Serial.println(max_value);
    //delay for claribration
    delay(1000);
  }
}
//for reading anfd predicting Magnetometer data 
void ReadMagnetometer(){
  //to calculate the output max value
    float y;
    int index = -1;
    float max_value =0;
    //Read the magnetometer value if the sensor is available
  if (IMU.magneticFieldAvailable()) {
    //read the magnetometer data and store it the array
    IMU.readMagneticField(sensor_data[0], sensor_data[1], sensor_data[2]);
    Serial.print("x_m:");
    Serial.print(sensor_data[0]);
    Serial.print('\t');
    Serial.print("y_m:");
    Serial.print(sensor_data[1]);
    Serial.print('\t');
    Serial.print("z_m:");
    Serial.println(sensor_data[2]);
    
    //store the inout data into the magnetometer inout variable for data to be predicted
    for (int i = 0; i < input_mag->bytes / sizeof(float); i++) {
      input_mag->data.f[i] = sensor_data[i];
    }
    //delay to let the interpreter initialisation and prediction happen
    delay(1000);
    //invoke the interpreter
    TfLiteStatus invoke_status = interpreter_mag->Invoke();
    //sanity check for interpreter
     if (invoke_status != kTfLiteOk) {
       //check if invoke failed the return
        MicroPrintf("Inference failed.");
        return;
     }
    delay(1000);
     for (int i = 0; i < num_posture; i++) {
      y = output_mag->data.f[i];
      if(y > max_value){
        index = i;
        max_value = y;
      }
    }
    Serial.print("Predicted Posture:");
    Serial.println(posture[index]);
    Serial.print("Confidence Score:");
    Serial.println(max_value);
    //delay for claribration
    delay(1000);
  }
}

void ReadGyroscope(){
  //to calculate the output max value
    float y;
    int index = -1;
    float max_value =0;
    //Read the gyroscope value if the sensor is available
  if (IMU.gyroscopeAvailable()) {
    //read the gyroscope data and store it the array
    IMU.readGyroscope(sensor_data[0], sensor_data[1], sensor_data[2]);
    Serial.print("x_g:");
    Serial.print(sensor_data[0]);
    Serial.print('\t');
    Serial.print("y_g:");
    Serial.print(sensor_data[1]);
    Serial.print('\t');
    Serial.print("z_g:");
    Serial.println(sensor_data[2]);
    //store the inout data into the gyroscope inout variable for data to be predicted
    for (int i = 0; i < input_gyro->bytes / sizeof(float); i++) {
      // for (int i = 0; i < num_posture; i++) {
      input_gyro->data.f[i] = sensor_data[i];
    }
    //delay to let the interpreter initialisation and prediction happen
    delay(2000);
    //invoke the interpreter
    TfLiteStatus invoke_status = interpreter_gyro->Invoke();
    //sanity check for interpreter
     if (invoke_status != kTfLiteOk) {
        MicroPrintf("Inference failed.");
     }
    delay(1000);
     for (int i = 0; i < num_posture; i++) {
      y = output_gyro->data.f[i];
      if(y > max_value){
        index = i;
        max_value = y;
      }
    }
    Serial.print("Predicted Posture:");
    Serial.println(posture[index]);
    Serial.print("Confidence Score:");
    Serial.println(max_value);
    //delay for claribration
    delay(1000);
  }
}


void setup() {
  Serial.begin(9600);
    while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  //setup the IMU Sensors
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in g's");
  Serial.println("X_A\tY_A\tZ_A");

  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Gyroscope in degrees/second");
  Serial.println("X_G\tY_G\tZ_G");

  Serial.print("Magnetic field sample rate = ");
  Serial.print(IMU.magneticFieldSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Magnetic Field in uT");
  Serial.println("X_M\tY_M\tZ_M");

  // Map the model into a usable data structure. 
  model_acc = tflite::GetModel(posture_acc_tflite);
  if (model_acc->version() != TFLITE_SCHEMA_VERSION) {
    MicroPrintf(
        "Model provided is schema version %d not equal "
        "to supported version %d.",
        model_acc->version(), TFLITE_SCHEMA_VERSION);
    return;
  }

    // Map the model into a usable data structure. 
  model_gyro = tflite::GetModel(posture_gyro_tflite);
  if (model_gyro->version() != TFLITE_SCHEMA_VERSION) {
    MicroPrintf(
        "Model provided is schema version %d not equal "
        "to supported version %d.",
        model_gyro->version(), TFLITE_SCHEMA_VERSION);
    return;
  }
  // Map the model into a usable data structure. 
    model_mag = tflite::GetModel(posture_mag_tflite);
  if (model_mag->version() != TFLITE_SCHEMA_VERSION) {
    MicroPrintf(
        "Model provided is schema version %d not equal "
        "to supported version %d.",
        model_mag->version(), TFLITE_SCHEMA_VERSION);
    return;
  }

  // This pulls in all the operation implementations we need.
  static tflite::AllOpsResolver resolver;

  // Build an interpreter to run the model with for accelerator
  static tflite::MicroInterpreter static_interpreter_acc(
      model_acc, resolver, tensor_arena_acc, kTensorArenaSizeAcc);
  interpreter_acc = &static_interpreter_acc;

  // Build an interpreter to run the model with for gyroscope
    static tflite::MicroInterpreter static_interpreter_gyro(
      model_gyro, resolver, tensor_arena_gyro, kTensorArenaSizeGyro);
  interpreter_gyro = &static_interpreter_gyro;

  // Build an interpreter to run the model with for magnetometer
    static tflite::MicroInterpreter static_interpreter_mag(
      model_mag, resolver, tensor_arena_mag, kTensorArenaSizeMag);
  interpreter_mag = &static_interpreter_mag;


  // Allocate memory from the tensor_arena for the model's tensors for accelerometer
  TfLiteStatus allocate_status_acc = interpreter_acc->AllocateTensors();
  if (allocate_status_acc != kTfLiteOk) {
    MicroPrintf("AllocateTensors() failed %d",allocate_status_acc);
    return;
  }

  // Allocate memory from the tensor_arena for the model's tensors for gyroscope
  TfLiteStatus allocate_status_gyro = interpreter_gyro->AllocateTensors();
  if (allocate_status_gyro != kTfLiteOk) {
    MicroPrintf("AllocateTensors() failed %d",allocate_status_gyro);
    return;
  }

    // Allocate memory from the tensor_arena for the model's tensors for magnetometer
  TfLiteStatus allocate_status_mag = interpreter_mag->AllocateTensors();
  if (allocate_status_mag != kTfLiteOk) {
    MicroPrintf("AllocateTensors() failed %d",allocate_status_mag);
    return;
  }

  // Obtain pointers to the model's input and output tensors for accelerometer
  input_acc = interpreter_acc->input(0);
  output_acc = interpreter_acc->output(0);

  // Obtain pointers to the model's input and output tensors for gyroscope
  input_gyro = interpreter_gyro->input(0);
  output_gyro = interpreter_gyro->output(0);

  // Obtain pointers to the model's input and output tensor for magnetometer
  input_mag = interpreter_mag->input(0);
  output_mag = interpreter_mag->output(0);

 //wait for user inout through UART
  Serial.println("Choose a sensor: 1) Accelerometer 2) Gyroscope 3) Magnetometer");

}


void loop() {
//if there is any user input , check and take action accordingly
  if (Serial.available() > 0) {
    int userChoice = Serial.parseInt();

    if (userChoice >= 1 && userChoice <= 3) {
      sensor_type = userChoice;  // Set the chosen sensor type
      Serial.print("Selected sensor: ");
      //check if user typed 1 for accelerometer data and prediction
      if (sensor_type == 1) {
        Serial.println("Accelerometer");
        ReadAccelerometer();

      } else if (sensor_type == 2) { // check if user typed 2 for gyroscope data and prediction
        Serial.println("Gyroscope");
        ReadGyroscope();

      } else if (sensor_type == 3) {  //check if user typed 3 for magnetometer data and prediction
        Serial.println("Magnetometer");
        ReadMagnetometer();

      }
    } else {
      //check for invalid option
      Serial.println("Invalid choice. Please choose 1, 2, or 3.");
    }

    // Clear the input buffer to prevent potential issues with leftover characters
    while (Serial.available() > 0) {
      Serial.read();
    }
  }

}


 