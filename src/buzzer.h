#include <Arduino.h>
#include <Tone32.h>

#define BUZZER_PIN 19
#define BUZZER_CHANNEL 0

struct BuzzerMessage
{
   int note;
   int duration;
};