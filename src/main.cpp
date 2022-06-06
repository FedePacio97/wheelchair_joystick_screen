#include <Arduino.h>
#include "ECU.h"
#include "buzzer.h"
#include "WiFi.h" //For BLE communication
#include <esp_now.h>
#include "messages.h"
#include "Screen.h"
/*
  DEBUG_LEVEL
    2 high
    1 medium
    0 none

*/


#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

//#define PLOTTING 1

#ifdef PLOTTING
//For plotting
#include "Plotter.h"
Plotter p; // create plotter
int CURRENT_RX_plotting, CURRENT_LX_plotting;
#endif

// define ECU object
ECU ecu;

// define Screen object
Screen screen;

// define two tasks for Blink & AnalogRead
void TaskBlink( void *pvParameters );
void TaskHandleBuzzer( void *pvParameters );
void TaskHandleJoystick( void *pvParameters );
void TaskHandleScreen( void *pvParameters );

/* Queue used to send and receive complete struct AMessage structures. */
QueueHandle_t xBuzzerQueue = NULL;

/* Queue used to receive Telemetry messages from ECU. */
QueueHandle_t xTelemetry_From_ECU_Queue = NULL;
/* Queue used to receive Stabiity messages from ECU. */
QueueHandle_t xStability_Info_From_ECU_Queue = NULL;

//Used to access stability_info_updates
SemaphoreHandle_t xSemaphore_mutex_stability_info_updates;
Stability_message stability_info_updates = {0};

//#define DEBUG_LEVEL 2

// the setup function runs once when you press reset or power the board
void setup() {
  
  // initialize serial communication at 115200 bits per second:
  #ifdef PLOTTING
  p.Begin(); // start plotter
  p.AddTimeGraph( "RPM", 4500, "rx", CURRENT_RX_plotting, "lx", CURRENT_LX_plotting ); // add any graphs you want
  #else
  Serial.begin(115200);
  #endif

  WiFi.mode(WIFI_MODE_STA);
  delay(10);

  #if DEBUG_LEVEL > 1
  Serial.println(WiFi.macAddress());
  #endif

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    #if DEBUG_LEVEL > 1
    Serial.println("Error initializing ESP-NOW");
    #endif
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, engineCU_controller_MAC, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    #if DEBUG_LEVEL > 1
    Serial.println("Failed to add peer");
    #endif
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecvJoystickController);

  //Initialize buzzer queue
  // Create the queue used to send complete struct BuzzerMessage structures.
  xBuzzerQueue = xQueueCreate(
                         // The number of items the queue can hold. 
                         10,
                         // Size of each item is big enough to hold the
                         //whole structure.
                         sizeof( BuzzerMessage ) );

  
  // Create the queue used to send complete struct BuzzerMessage structures.
  xTelemetry_From_ECU_Queue = xQueueCreate(
                         // The number of items the queue can hold. 
                         10,
                         // Size of each item is big enough to hold the
                         //whole structure.
                         sizeof( Telemetry_message ) );

  // Create the queue used to receive Stability_message from IMU (in wheelchair_ecu).
  xStability_Info_From_ECU_Queue = xQueueCreate(
                         // The number of items the queue can hold. 
                         10,
                         // Size of each item is big enough to hold the
                         //whole structure.
                         sizeof( Stability_message ) );


  xSemaphore_mutex_stability_info_updates = xSemaphoreCreateMutex();
  
  if( xSemaphore_mutex_stability_info_updates == NULL )
  {
      // The semaphore wasn't created successfully
      //buzz();
      //abort
  }


  //Initilize Nextion screen
  // if(screen.initialize())
  //   Serial.printf("Screen ok");
  // else
  //   Serial.printf("Screen FAIL");
  
  // screen.attach_listeners();


  // Now set up two tasks to run independently.
  xTaskCreatePinnedToCore(
    TaskBlink
    ,  "TaskBlink"   // A name just for humans
    ,  1024  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    TaskHandleJoystick
    ,  "HandleJoystickMovement"
    ,  2048  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);

  // xTaskCreatePinnedToCore(
  //   TaskHandleScreen
  //   ,  "HandleScreen"
  //   ,  2048  // Stack size
  //   ,  NULL
  //   ,  1  // Priority
  //   ,  NULL 
  //   ,  ARDUINO_RUNNING_CORE);


/*  xTaskCreatePinnedToCore(
    TaskHandleBuzzer
    ,  "HandleBuzzer"
    ,  2048  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);
*/
  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop()
{
  // Empty. Things are done in Tasks.
  #ifdef PLOTTING
  p.Plot();
  #endif
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskBlink(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.
    
  If you want to know what pin the on-board LED is connected to on your ESP32 model, check
  the Technical Specs of your board.
*/

  // initialize digital LED_BUILTIN on pin 13 as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  for (;;) // A Task shall never return or exit.
  {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    vTaskDelay(1000 /portTICK_PERIOD_MS);  // 1000ms delay
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    vTaskDelay(1000 /portTICK_PERIOD_MS);  // 1000ms delay
  }
}

void TaskHandleJoystick(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  
/*
  TaskJoystickReading
  Reads joystick potentiometers
  Map read values in RPM references
  Send RPM over BLE

  Graphical representation is available using (serial plotter (Tools > Serial Plotter menu) on Arduino IDE OR https://community.platformio.org/t/plotting-using-platformio/6597/7
  Attach the center pin of a potentiometer to pin XX, and the outside pins to +5V and ground. (The same for YY)

*/
  Stability_message stability_message_received_from_BLE = {0};
  for (;;)
  {

      //Retrieve stability_message_received_from_BLE
      //Protect by mutex
      xSemaphoreTake( xSemaphore_mutex_stability_info_updates, portMAX_DELAY );
      stability_message_received_from_BLE = stability_info_updates;
      xSemaphoreGive( xSemaphore_mutex_stability_info_updates );

      ecu.update_CURRENT_reference_rear_wheels(stability_message_received_from_BLE);
        //ecu.update_wheelchair_reference_speed();
      //float reference_linear_speed = ecu.get_reference_linear_speed();
      //float reference_angular_speed = ecu.get_reference_angular_speed();

        //ecu.update_CURRENT_reference_rear_wheels_old();
      float CURRENT_lx = ecu.get_reference_CURRENT_lx();
      float CURRENT_rx = ecu.get_reference_CURRENT_rx();

      #ifdef PLOTTING
      CURRENT_LX_plotting = CURRENT_lx;
      CURRENT_RX_plotting = CURRENT_rx;
      #endif

      //Serial.printf("reference_linear_speed %f\t reference_angular_speed%f\n",reference_linear_speed,reference_angular_speed);

      //#if DEBUG_LEVEL > 1
      Serial.printf("CURRENT_lx %f\t CURRENT_rx %f\n",CURRENT_lx,CURRENT_rx);
      //#endif

      CURRENT_message_sent_on_BLE current_message_sent_on_BLE;
      current_message_sent_on_BLE.OPCODE = CURRENT_REFERENCE_OPCODE;
      current_message_sent_on_BLE.rpm.CURRENT_LX = CURRENT_lx;
      current_message_sent_on_BLE.rpm.CURRENT_RX = CURRENT_rx;

      //Send data via BLE
      esp_now_send(engineCU_controller_MAC, (uint8_t *) &current_message_sent_on_BLE, sizeof(CURRENT_message_sent_on_BLE));

      //SEND REFERENCE TO wheelchair_ecu via BLE
      //interfaceEngineCU.set_RPM_motors(CURRENT_lx,CURRENT_rx);

      /*      
      //get velocities from IMU (controller.get_velocities()) e un'altra task con update_velocities (aggiorna i campi che ritornerà get_velocites()) con rate più veloce per oversampling and filtering
      //implementare controllore
      uint8_t tau_LX = 5, tau_RX = 0;

      //Send to roboteq
      interfaceEngineCU_ECU.send_tauLX_tauRX(tau_LX,tau_RX);

      */


    /*// read the input on analog pin A3:
    int sensorValueA3 = analogRead(A3);
    // print out the value you read:
    Serial.println(sensorValueA3);*/
    vTaskDelay(15/portTICK_PERIOD_MS);  // 50ms delay == 1/(50 * 10^-3) = 100Hz
  }
}

void TaskHandleScreen(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  
/*
  TaskHandleScreen
  Read telemetry info from BLE.
  Read IMU (pitch and roll) info from BLE.
  Show on screen
  Listen for touch event

*/

  Telemetry_message telemetry_message;
  Stability_message stability_info;
  for (;;)
  {
    //Shoudl be called as much as possible to be more responsive
    screen.listen_touch_event();

    if( xTelemetry_From_ECU_Queue != NULL ){
      /* Receive a message from the created queue to hold complex struct Telemetry_message_sent_on_BLE
      structure. Not blocking ( 0 and not portMAX_DELAY) since shoudl be listen for touch event.
      */
      if(
        xQueueReceive( xTelemetry_From_ECU_Queue, 
                        &( telemetry_message ),
                        0 ) == pdPASS)
      {

        //Set info in the screen
        float km = telemetry_message.km;
        screen.set_km(km);

        
        int battery_level = telemetry_message.battery_level;
        screen.set_battery_level(battery_level);

        //TO DO: if negative, not shown (maybe Nextion bug because negative speed correctly received)
        int speed = telemetry_message.speed;
        screen.set_speed(speed);

      }
    }

    if( xStability_Info_From_ECU_Queue != NULL ){
      /* Receive a message from the created queue to hold complex struct Stability_message
      structure. Not blocking ( 0 and not portMAX_DELAY) since shoudl be listen for touch event.
      */
      if(
        xQueueReceive( xStability_Info_From_ECU_Queue, 
                        &( stability_info ),
                        0 ) == pdPASS)
      {
        //Send to Screen
        //#if DEBUG_LEVEL > 1
        Serial.printf("stability -> pitch %d\n",stability_info.pitch);
        Serial.printf("stability -> roll %d\n",stability_info.roll);

        Serial.printf("stability -> accX %f\t accY %f \t accZ %f\n",stability_info.accel_x, stability_info.accel_y, stability_info.accel_z);
        Serial.printf("stability -> gyroX %f\t gyroY %f \t gyroZ %f \n",stability_info.gyro_x, stability_info.gyro_y, stability_info.gyro_z);
        Serial.printf("stability -> magX %f\t magY %f \t magZ %f\n",stability_info.mag_x, stability_info.mag_y, stability_info.mag_z);
        //#endif
        screen.set_pitch(stability_info.pitch);
        screen.set_roll(stability_info.roll);

        xSemaphoreTake( xSemaphore_mutex_stability_info_updates, portMAX_DELAY );
        stability_info_updates = stability_info;
        xSemaphoreGive( xSemaphore_mutex_stability_info_updates );
        
      }
    }
    
    vTaskDelay(10/portTICK_PERIOD_MS);  // 50ms delay == 1/(50 * 10^-3) = 100Hz
  }
   
}

void TaskHandleBuzzer(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

/*
  Buzzer
  Buzz for duration and tone passed as input with queue 
*/

  //To send
  /* Send the entire structure to the queue created to hold 10 structures. */
  BuzzerMessage buzzerMessageToSend;
  buzzerMessageToSend.duration=500;
  buzzerMessageToSend.note = NOTE_A4;
  xQueueSend( // The handle of the queue.
               xBuzzerQueue,
               // The address of the xMessage variable.  sizeof( struct AMessage )
               //bytes are copied from here into the queue.
               ( void * ) &buzzerMessageToSend,
               // Block time of 0 says don't block if the queue is already full.
               //Check the value returned by xQueueSend() to know if the message
               //was sent to the queue successfully.
               ( TickType_t ) 0 );


  // initialize digital LED_BUILTIN on pin 13 as an output.
  pinMode(BUZZER_PIN, OUTPUT);
  struct BuzzerMessage buzzerMessageReceived;

  for (;;) // A Task shall never return or exit.
  {

    if( xBuzzerQueue != NULL )
    {
      // Receive a message from the created queue to hold complex struct BuzzerMessage
      //structure.  Block until a message is not available.
      //The value is read into a struct BuzzerMessage variable, so after calling
      //xQueueReceive() buzzerMessage will hold a copy of xMessage.
      //xQueueReceive( xBuzzerQueue, &( buzzerMessageReceived ),portMAX_DELAY);

      //tone
      tone(BUZZER_PIN, buzzerMessageReceived.note, buzzerMessageReceived.duration, BUZZER_CHANNEL);
      noTone(BUZZER_PIN, BUZZER_CHANNEL);

      /*tone(BUZZER_PIN, NOTE_A4, 500, BUZZER_CHANNEL);
      tone(BUZZER_PIN,NOTE_A4, 500, BUZZER_CHANNEL);  
      tone(BUZZER_PIN,NOTE_A4, 500, BUZZER_CHANNEL);
      tone(BUZZER_PIN,NOTE_F4, 350, BUZZER_CHANNEL);
      tone(BUZZER_PIN,NOTE_C5, 150, BUZZER_CHANNEL); 
      tone(BUZZER_PIN,NOTE_A4, 500, BUZZER_CHANNEL);
      tone(BUZZER_PIN,NOTE_F4, 350, BUZZER_CHANNEL);
      tone(BUZZER_PIN,NOTE_C5, 150, BUZZER_CHANNEL);
      tone(BUZZER_PIN,NOTE_A4, 650, BUZZER_CHANNEL);
    
      delay(500);
    
      tone(BUZZER_PIN,NOTE_E5, 500, BUZZER_CHANNEL);
      tone(BUZZER_PIN,NOTE_E5, 500, BUZZER_CHANNEL);
      tone(BUZZER_PIN,NOTE_E5, 500, BUZZER_CHANNEL); 
      tone(BUZZER_PIN,NOTE_F5, 350, BUZZER_CHANNEL);
      tone(BUZZER_PIN,NOTE_C5, 150, BUZZER_CHANNEL);
      tone(BUZZER_PIN,NOTE_GS4, 500, BUZZER_CHANNEL);
      tone(BUZZER_PIN,NOTE_F4, 350, BUZZER_CHANNEL);
      tone(BUZZER_PIN,NOTE_C5, 150, BUZZER_CHANNEL);
      tone(BUZZER_PIN,NOTE_A4, 650, BUZZER_CHANNEL);
    
      delay(500);
    
      tone(BUZZER_PIN,NOTE_A5, 500, BUZZER_CHANNEL);
      tone(BUZZER_PIN,NOTE_A4, 300, BUZZER_CHANNEL);
      tone(BUZZER_PIN,NOTE_A4, 150, BUZZER_CHANNEL);
      tone(BUZZER_PIN,NOTE_A5, 500, BUZZER_CHANNEL);
      tone(BUZZER_PIN,NOTE_GS5, 325, BUZZER_CHANNEL);
      tone(BUZZER_PIN,NOTE_G5, 175, BUZZER_CHANNEL);
      tone(BUZZER_PIN,NOTE_FS5, 125, BUZZER_CHANNEL);
      tone(BUZZER_PIN,NOTE_F5, 125, BUZZER_CHANNEL);   
      tone(BUZZER_PIN,NOTE_FS5, 250, BUZZER_CHANNEL);
    
      delay(325);
    
      tone(BUZZER_PIN,NOTE_AS4, 250, BUZZER_CHANNEL);
      tone(BUZZER_PIN,NOTE_DS5, 500, BUZZER_CHANNEL);
      tone(BUZZER_PIN,NOTE_D5, 325, BUZZER_CHANNEL); 
      tone(BUZZER_PIN,NOTE_CS5, 175, BUZZER_CHANNEL); 
      tone(BUZZER_PIN,NOTE_C5, 125, BUZZER_CHANNEL); 
      tone(BUZZER_PIN,NOTE_AS4, 125, BUZZER_CHANNEL);
      tone(BUZZER_PIN,NOTE_C5, 250, BUZZER_CHANNEL); 
    
      delay(350);*/
    }
  }
}

/*
void TaskCheckAlivenessEngineCU(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  
  
  TaskCheckAlivenessEngineCU
  Used to check if within interfaceEngineCU_ECU.KEEP_ALIVE_PERIOD_ms at least a mex has been received from the EngineCU: if not so, something wrong is happening
  so enable security_stop_procedure

*/  
/*
  TickType_t xLastWakeTime;
  // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();
  for (;;)
  {
    // Wait for the next cycle.
    vTaskDelayUntil( &xLastWakeTime, interfaceEngineCU_ECU.get_KEEP_ALIVE_PERIOD_ms()/portTICK_PERIOD_MS ); //Fixed time execution at KEEP_ALIVE_PERIOD_ms (1000ms by default) rate

    // Perform action here

    if(interfaceEngineCU_ECU.get_received_message_within_keep_alive_period() == true){
      //Correct behaviour
      interfaceEngineCU_ECU.reset_received_message_within_keep_alive_period();
    }else{
      //Uncorrect behaviour

      //Send a "notification" to the TaskHandleWheelchairMovement
      //Set ECU_EngineCU_notification
      xSemaphoreTake( xSemaphore_mutex_ECU_EngineCU_notification, portMAX_DELAY );
      ECU_EngineCU_notification = EngineCU_DEAD;
      xSemaphoreGive( xSemaphore_mutex_ECU_EngineCU_notification );

    }
  }
}*/
/*
void TaskIMUupdates(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  
  
  
  //TaskIMUupdates
  //Handles IMU updates and check if pitch and roll are below safe thresholds: if not so, send notification to ECU


  TickType_t xLastWakeTime;
  // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();
  for (;;)
  {
    // Wait for the next cycle.
    vTaskDelayUntil( &xLastWakeTime, 20/portTICK_PERIOD_MS ); //Fixed time execution at KEEP_ALIVE_PERIOD_ms (1000ms by default) rate
    //May execute at rate of 1/50ms, i.e 20Hz

    // Perform action here
    imu.updateSensors();
    uint16_t stability_result = imu.check_stability();
    if(stability_result == WHEELCHAIR_STABLE){
      //Correct behaviour
      Serial.printf("[TaskIMUupdates] Wheelchair is stable!\n");
    }else{
      //Uncorrect behaviour
      //Send a "notification" to the TaskHandleWheelchairMovement
      //Extract ECU_IMU_notification
      xSemaphoreTake( xSemaphore_mutex_ECU_IMU_notification, portMAX_DELAY );
      ECU_IMU_notification = stability_result;
      xSemaphoreGive( xSemaphore_mutex_ECU_IMU_notification );

    }
  }
}*/
/*
void TaskCheckConsistencyAmongRPM_IMUvelocity(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  
  
  //TaskCheckConsistencyAmongRPM_IMUvelocity
  //Used to check if received rpm from EngineCU is consistent with IMU measurements: 
  //if not so, something wrong is happening so enable security_stop_procedure.

  //E.g: rolling != 0, velocity != while rpm = 0..

  

  TickType_t xLastWakeTime;
  // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();

  uint32_t ulNotifiedValue;
  for (;;)
  {
    // Wait for the next cycle.
    vTaskDelayUntil( &xLastWakeTime, interfaceEngineCU_ECU.get_CONSISTENCY_CHECKING_PERIOD_ms()/portTICK_PERIOD_MS ); //Fixed time execution at KEEP_ALIVE_PERIOD_ms (1000ms by default) rate

    // Perform action here
    //Ask for rpm
    interfaceEngineCU_ECU.request_rpm_motors();
    //Wait for response
    //CHECK IF PENDING NOTIFICATION
    xTaskNotifyWait(
                      0x00,      // Don't clear any notification bits on entry. 
                      ULONG_MAX, // Reset the notification value to 0 on exit. 
                      &ulNotifiedValue, // Notified value pass out in ulNotifiedValue. 
                      portMAX_DELAY  );  // Block indefinitely. 

    //Whatever ulNotifiedValue is ok, just unblock from Wait()

    //Parse
    int current_rpm_motor_LX = interfaceEngineCU_ECU.get_rpm_motor_LX();
    int current_rpm_motor_RX = interfaceEngineCU_ECU.get_rpm_motor_RX();

    Serial.printf("[TaskCheckConsistencyAmongRPM_IMUvelocity] current_rpm_motor_LX -> %d \t current_rpm_motor_RX -> %d\n",current_rpm_motor_LX,current_rpm_motor_RX);
    //GET IMU VALUES
  }
}*/




//TO DO
/*
SETUP PER JOYSTICK E AUTOCALIBRATION DURANTE USO SE VEDE CURRENT <> THRESHOLD ..missing
FREQUENCY JOYSTICK 20 HZ -> v
APPLY FILTER ON JOYSTICK -> v
*/