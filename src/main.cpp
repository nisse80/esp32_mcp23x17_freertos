///////////// Copyright © 2022 
//
//   Description :
//      Sample program to create FreeRTOS tasks on an ESP32 using 
//      the Adafruit MCP23017 Arduino Library. The programm creates 
//      two RTOS tasks keeping Pins on an MCP23017 I/O Expander to a defined state
//
//   Created On: 24/01/2022
//   Created By: Nils-Henrik Wolf nils.h.wolf@gmail.com
//
//
//   This code is licensed under MIT license
//
////////////////////////////////////////////////////////////////////////////


#include <Arduino.h>
#include "Adafruit_MCP23X17.h"

Adafruit_MCP23X17 mcp;

static xSemaphoreHandle mutex;

typedef struct mcp_state_struct // Declare struct for passing to FreeRTOS Task
{
  int Pin1; //Pin1 to set
  int Pin2; //Pin2 to set
  int start_state_Pin1; // Starting state of PIN1
  int start_state_Pin2; // Starting state of PIN2
  int end_state_Pin1; // End state of PIN1 after 'duration'
  int end_state_Pin2; // End state of PIN2 after 'duration'
  int duration_ms; // duration in milliseconds

} mcp_state_struct;

mcp_state_struct *mcp_task0; // Init a struct for passing to RTOS task
mcp_state_struct *mcp_task1; // Init a struct for passing to RTOS task

void initialize_mcp()
{
  for (int i = 0; i < 16; i++) //initialze as OUTPUT and set LOW
  {

    mcp.pinMode(i, OUTPUT); 
    mcp.digitalWrite(i, LOW);
  }
}

void start_mcp_task(void *mcp_struct)
{
  mcp_state_struct *mcp_task;
  mcp_task = (mcp_state_struct *)mcp_struct;

  while (1) //wait until Tasks gets mutex
  {
    if (xSemaphoreTake(mutex, 10) == pdTRUE) // if mutex was taken
    {
      mcp.digitalWrite(mcp_task->Pin1, mcp_task->start_state_Pin1); // Set PIN1
      mcp.digitalWrite(mcp_task->Pin2, mcp_task->start_state_Pin2); // Set PIN2
      xSemaphoreGive(mutex); // Give back the mutex
      break; // Leave while
    }
  }

  vTaskDelay(mcp_task->duration_ms / portTICK_PERIOD_MS); // Keep Pins high until duration in ms

  while (1) // After duration set Pins to end state
  {
    if (xSemaphoreTake(mutex, 10) == pdTRUE)
    {
      mcp.digitalWrite(mcp_task->Pin1, mcp_task->end_state_Pin1);
      mcp.digitalWrite(mcp_task->Pin2, mcp_task->end_state_Pin2);
      xSemaphoreGive(mutex);
      break;
    }
  }

  vTaskDelete(NULL);
}

void setup()
{
  Serial.begin(115200);

  if (!mcp.begin_I2C()) //Start I2C connection
  {
    Serial.println("Error initializing I2C connection to MCP23X17");
    while (1)
      ;
  }
  
  Serial.println("I2C connection to MCP23X17 initialized");
  
  initialize_mcp(); // Initialize IO Expander and set all Pins to LOW

  mcp_task0 = (mcp_state_struct *)pvPortMalloc(sizeof(mcp_state_struct)); // Allocate Heap memory for truct to pass to RTOS Task
  mcp_task1 = (mcp_state_struct *)pvPortMalloc(sizeof(mcp_state_struct)); // Allocate Heap memory for truct to pass to RTOS Task

  //Set struct variables
  mcp_task0->Pin1 = 0; // I/O Expander Pin
  mcp_task0->Pin2 = 1; // I/O Expander Pin
  mcp_task0->duration_ms = 1000; 
  mcp_task0->start_state_Pin1 = HIGH;
  mcp_task0->start_state_Pin2 = LOW;
  mcp_task0->end_state_Pin1 = LOW;
  mcp_task0->end_state_Pin2 = HIGH;

  mutex = xSemaphoreCreateMutex();

  xTaskCreate(start_mcp_task, "Start TASK 1", 1000, (void *)mcp_task0, 1, NULL); // Pass struct to function (1000 bytes of Stack should be enough for this task)


  mcp_task1->Pin1 = 2;
  mcp_task1->Pin2 = 3;
  mcp_task1->duration_ms = 3000;
  mcp_task1->start_state_Pin1 = LOW;
  mcp_task1->start_state_Pin2 = HIGH;
  mcp_task1->end_state_Pin1 = HIGH;
  mcp_task1->end_state_Pin2 = LOW;

  xTaskCreate(start_mcp_task, "Start TASK 2", 1000, (void *)mcp_task1, 1, NULL);
}

void loop()
{
  // NOOP
}