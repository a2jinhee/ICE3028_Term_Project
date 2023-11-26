/*
  Smartglove for cyclists: https://www.instructables.com/member/Matlek/
  This code is made for an Arduino Nano 33 BLE Sense board:
  it detects left hand gestures, and sends the gestures information through BLE (to another microcontroller with an LED matrix);
  It is a mix of the following codes, after a few modifications:
    -"LED" example from the "ArduinoBLE" library (Peripheral>LED).
    -"IMU_Classifier" found here: https://github.com/arduino/ArduinoTensorFlowLiteTutorials/blob/master/GestureToEmoji/ArduinoSketches/IMU_Classifier/IMU_Classifier.ino.
*/
#include <TensorFlowLite.h>

#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"

#include "magic_wand_model_data.h"
#include "rasterize_stroke.h"
#include "imu_provider.h"

volatile int state_flag = 0;
const float gyroscopeThreshold = 300;
const int numSamples = 64;
int samplesRead = numSamples;

namespace {

  const int VERSION = 0x00000000;

  // Constants for image rasterization
  constexpr int raster_width = 32;
  constexpr int raster_height = 32;
  constexpr int raster_channels = 3;
  constexpr int raster_byte_count = raster_height * raster_width * raster_channels;
  int8_t raster_buffer[raster_byte_count];

  
  // Create an area of memory to use for input, output, and intermediate arrays.
  // The size of this will depend on the model you're using, and may need to be
  // determined by experimentation.
  constexpr int kTensorArenaSize = 30 * 1024;
  uint8_t tensor_arena[kTensorArenaSize];
  
  tflite::ErrorReporter* error_reporter = nullptr;
  const tflite::Model* model = nullptr;
  tflite::MicroInterpreter* interpreter = nullptr;
  
  // -------------------------------------------------------------------------------- //
  // UPDATE THESE VARIABLES TO MATCH THE NUMBER AND LIST OF GESTURES IN YOUR DATASET  //
  // -------------------------------------------------------------------------------- //
  constexpr int label_count = 3;
  const char* labels[label_count] = {"0", "1", "2"};

  String a;
  int gesture=0;
  int gesture_selected=0;

}  // namespace

int Button=2; //디지털 IO 3번핀에 스위치 연결
void setup() {
  Serial.begin(9600);
  pinMode(Button,INPUT); //버튼은 입력으로 설정

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  SetupIMU();

  // initialize the BLE hardware
  BLE.begin();

  // start scanning for peripherals
  //BLE.scanForUuid("e4297ee0-8c88-11ea-bc55-0242ac130003");
  if (!BLE.begin()) {
    Serial.println("Failed to initialized BLE!");
    while (1);
  }

  // Set up logging. Google style is to avoid globals or statics because of
  // lifetime uncertainty, but since this has a trivial destructor it's okay.
  static tflite::MicroErrorReporter micro_error_reporter;  // NOLINT
  error_reporter = &micro_error_reporter;

  // Map the model into a usable data structure. This doesn't involve any
  // copying or parsing, it's a very lightweight operation.
  model = tflite::GetModel(g_magic_wand_model_data);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    TF_LITE_REPORT_ERROR(error_reporter,
                         "Model provided is schema version %d not equal "
                         "to supported version %d.",
                         model->version(), TFLITE_SCHEMA_VERSION);
    return;
  }

  // Pull in only the operation implementations we need.
  // This relies on a complete list of all the ops needed by this graph.
  // An easier approach is to just use the AllOpsResolver, but this will
  // incur some penalty in code space for op implementations that are not
  // needed by this graph.
  static tflite::MicroMutableOpResolver<4> micro_op_resolver;  // NOLINT
  micro_op_resolver.AddConv2D();
  micro_op_resolver.AddMean();
  micro_op_resolver.AddFullyConnected();
  micro_op_resolver.AddSoftmax();

  // Build an interpreter to run the model with.
  static tflite::MicroInterpreter static_interpreter(
      model, micro_op_resolver, tensor_arena, kTensorArenaSize, error_reporter);
  interpreter = &static_interpreter;

  // Allocate memory from the tensor_arena for the model's tensors.
  interpreter->AllocateTensors();

  // Set model input settings
  TfLiteTensor* model_input = interpreter->input(0);
  if ((model_input->dims->size != 4) || (model_input->dims->data[0] != 1) ||
      (model_input->dims->data[1] != raster_height) ||
      (model_input->dims->data[2] != raster_width) ||
      (model_input->dims->data[3] != raster_channels) ||
      (model_input->type != kTfLiteInt8)) {
    TF_LITE_REPORT_ERROR(error_reporter,
                         "Bad input tensor parameters in model");
    return;
  }

  // Set model output settings
  TfLiteTensor* model_output = interpreter->output(0);
  if ((model_output->dims->size != 2) || (model_output->dims->data[0] != 1) ||
      (model_output->dims->data[1] != label_count) ||
      (model_output->type != kTfLiteInt8)) {
    TF_LITE_REPORT_ERROR(error_reporter,
                         "Bad output tensor parameters in model");
    return;
  }
  
  pinMode(Button, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(Button), state, RISING);
}

void state(){
  state_flag = 1;
}


void loop() {
  
  //repeating timer이용? 심박수
  Serial.println(state_flag);

  if(state_flag == 1){

    if(gesture<1){

      const bool data_available = IMU.accelerationAvailable() || IMU.gyroscopeAvailable();
      if (!data_available) {
          return;
      }

      int accelerometer_samples_read;
      int gyroscope_samples_read;
      ReadAccelerometerAndGyroscope(&accelerometer_samples_read, &gyroscope_samples_read);

      // Parse and process IMU data
      bool done_just_triggered = false;
      if (gyroscope_samples_read > 0) {
          EstimateGyroscopeDrift(current_gyroscope_drift);
          UpdateOrientation(gyroscope_samples_read, current_gravity, current_gyroscope_drift);
          UpdateStroke(gyroscope_samples_read, &done_just_triggered);

      }
      if (accelerometer_samples_read > 0) {
          EstimateGravityDirection(current_gravity);
          UpdateVelocity(accelerometer_samples_read, current_gravity);
      }

        // Wait for a gesture to be done
      if (done_just_triggered) {     //to do.  When Swtich clicked Operate 
          // Rasterize the gesture
          RasterizeStroke(stroke_points, *stroke_transmit_length, 0.6f, 0.6f, raster_width, raster_height, raster_buffer);
          
          // Pass to the model and run the interpreter
          TfLiteTensor* model_input = interpreter->input(0);
          for (int i = 0; i < raster_byte_count; ++i) {
            model_input->data.int8[i] = raster_buffer[i];
          }
          TfLiteStatus invoke_status = interpreter->Invoke();
          if (invoke_status != kTfLiteOk) {
            TF_LITE_REPORT_ERROR(error_reporter, "Invoke failed");
            return;
          }
          TfLiteTensor* output = interpreter->output(0);

          // Parse the model output
          int8_t max_score;
          int max_index;
          for (int i = 0; i < label_count; ++i) {
            const int8_t score = output->data.int8[i];
            if ((i == 0) || (score > max_score)) {
              max_score = score;
              max_index = i;
            }
          }
          Serial.println(max_index);
          TF_LITE_REPORT_ERROR(error_reporter, "Found %s (%d)", labels[max_index], max_score);
          gesture_selected=max_index;
          gesture++;
      }
    }

    else if(gesture==1){

      switch(gesture_selected){  
        case 0: Serial.println("Door Selected"); 
                a = "19b1";
                break;

        case 1: Serial.println("Window Selected"); 
                a = "19b2";    
                break;
                // else if gesture[1]==1: a = "19c2"; break;
                // else if gesture[1]==2:  a = "19d2"; break;
                 
        case 2: Serial.println("LED Selected");
                a = "19b3"; break;
                // else if gesture[1]==1: a = "19c1"; break;
                // else if gesture[1]==2:  a = "19d1"; break;
                
        default:  Serial.println("Init State");
                a = "19B10003-E8F2-537E-4F6C-D104768BBDDS"; break;
      }
     
      Serial.println(a); 
      gesture++; 
    
      
    }
    
    else{
      BLE.scanForUuid(a);

      BLEDevice peripheral = BLE.available();
      String myString = peripheral.localName();
      const char* charArray = myString.c_str();

      Serial.print(peripheral.localName());
      if (peripheral.advertisedServiceUuid() == a) {
        Serial.print("Found ");
        Serial.print(peripheral.address());
        Serial.print(" '");
        Serial.print(peripheral.localName());
        Serial.print("' ");
        Serial.print(peripheral.advertisedServiceUuid());
        Serial.println();

        BLE.stopScan();
          
        Serial.println("Connecting ...");

        if (peripheral.connect()) {
            Serial.println("Connected");
            gesture=0;
        } 
        else {
            Serial.println("Failed to connect!");
            gesture=0;
            return;
        }

          Serial.println("Gesture operate");
          delay(1000);
          peripheral.disconnect();
          state_flag = 0;
      
      }
    }

  }
}