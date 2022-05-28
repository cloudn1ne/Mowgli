//
// simple pulse counter with averaging
//
// microswitch against GND on BUTTON_PIN(12)
//
//
// when on 32V charger power: 
// 
//  one tick = one count/protrusion (there are 8 in total in one depth, so 9 makes one revolution)
//
//  PWM speed (PAC motorcontroller) - Time in mS for a Tick
//
//    0xFF - 105mS
//    0xC0 - 106mS
//    0xA0 - 128mS
//    0x80 - 160mS
//    0x60 - 213mS
//    0x40 - 314mS   
//

const int BUTTON_PIN = 12;        // the number of the input pin for the switch
const int DEBOUNCE_DELAY = 10;    // the debounce time

int lastSteadyState = LOW;       // the previous steady state from the input pin
int lastFlickerableState = LOW;  // the previous flickerable state from the input pin
int currentState;                // the current reading from the input pin


const float CIRCUMREFERENCE = 0.622;  // circumreference in m
const int TICK_PER_REV = 9;      // yardforce has 8 protrusions on the wheel, so after the 9th we have one rev 
const int REVOLUTIONS = 1;       // number of revolutions to measure
const int TICK_COUNT = TICK_PER_REV * REVOLUTIONS;  // how many ticks to we need in total 
int16_t count=0;
unsigned long start_time;
unsigned long end_time;

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  // initialize the pushbutton pin as an pull-up input
  // the pull-up input pin will be HIGH when the switch is open and LOW when the switch is closed.
  pinMode(BUTTON_PIN, INPUT_PULLUP);
}

void loop() {
  uint8_t control=0;
  uint16_t time_per_meter;
  uint16_t time_per_revolution;
  uint16_t time_per_tick;
  float meter_per_sec;
  
  // read the state of the switch/button:
  currentState = digitalRead(BUTTON_PIN);

  if(Serial.available())
  { 
    control=Serial.read();
    if (control == 'r')
    {
      count = 0;
    }
  }

  // If the switch/button changed, due to noise or pressing:
  if (currentState != lastFlickerableState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
    // save the the last flickerable state
    lastFlickerableState = currentState;
  }

  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (lastSteadyState == LOW && currentState == HIGH)
    {// Serial.println("The button is released");
      if (count == 0)
      {
        start_time = millis();
        Serial.println("");
        Serial.println("--- start ---");
        Serial.print("[");
      }
      if (count == TICK_COUNT-1)
      {
        end_time = millis();        
        time_per_tick = (end_time-start_time)/(TICK_COUNT-1);
        time_per_revolution = (end_time-start_time+time_per_tick)/REVOLUTIONS;
        Serial.println("]");
        Serial.print("time per revolution (mS): ");
        Serial.println(time_per_revolution);
        Serial.print("time per tick: ");
        Serial.println(time_per_tick);

        time_per_meter = time_per_revolution/CIRCUMREFERENCE;
        Serial.print("time per meter (mS): ");
        Serial.println(time_per_meter);
        meter_per_sec = (1.0/time_per_meter)*1000.0;
        Serial.print("meter per sec (m/S): ");
        Serial.println(meter_per_sec);
        
        count = 0;
      }
      else
      {
        count++;
        Serial.print("x");
       // Serial.print("count: ");
        //Serial.println(count);
      }
    }
    // save the the last steady state
    lastSteadyState = currentState;
  }
}
