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
int RPM_RX_plotting, RPM_LX_plotting;
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

// the setup function runs once when you press reset or power the board
void setup() {
  
  // initialize serial communication at 115200 bits per second:
  #ifdef PLOTTING
  p.Begin(); // start plotter
  p.AddTimeGraph( "RPM", 4500, "rx", RPM_RX_plotting, "lx", RPM_LX_plotting ); // add any graphs you want
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
  if(screen.initialize())
    Serial.printf("Screen ok");
  else
    Serial.printf("Screen FAIL");
  
  screen.attach_listeners();


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

  xTaskCreatePinnedToCore(
    TaskHandleScreen
    ,  "HandleScreen"
    ,  2048  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);
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

    ecu.update_RPM_reference_rear_wheels(stability_message_received_from_BLE);

    int RPM_lx = ecu.get_reference_RPM_lx();
    int RPM_rx = ecu.get_reference_RPM_rx();

    #ifdef PLOTTING
    RPM_LX_plotting = RPM_lx;
    RPM_RX_plotting = RPM_rx;
    #endif

    #if DEBUG_LEVEL > 1
    Serial.printf("RPM_lx %d\t RPM_rx %d\n",RPM_lx,RPM_rx);
    #endif

    RPM_message_sent_on_BLE rpm_sent_on_BLE;
    rpm_sent_on_BLE.OPCODE = RPM_REFERENCE_OPCODE;
    rpm_sent_on_BLE.rpm.RPM_LX = RPM_lx;
    rpm_sent_on_BLE.rpm.RPM_RX = RPM_rx;

    //Send data via BLE
    esp_now_send(engineCU_controller_MAC, (uint8_t *) &rpm_sent_on_BLE, sizeof(RPM_message_sent_on_BLE));

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

        //TO CHECK: if negative, not shown (maybe Nextion bug because negative speed correctly received)
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
        #if DEBUG_LEVEL > 1
        Serial.printf("stability -> pitch %d\n",stability_info.pitch);
        Serial.printf("stability -> roll %d\n",stability_info.roll);

        Serial.printf("stability -> accX %f\t accY %f \t accZ %f\n",stability_info.accel_x, stability_info.accel_y, stability_info.accel_z);
        Serial.printf("stability -> gyroX %f\t gyroY %f \t gyroZ %f \n",stability_info.gyro_x, stability_info.gyro_y, stability_info.gyro_z);
        Serial.printf("stability -> magX %f\t magY %f \t magZ %f\n",stability_info.mag_x, stability_info.mag_y, stability_info.mag_z);
        #endif
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