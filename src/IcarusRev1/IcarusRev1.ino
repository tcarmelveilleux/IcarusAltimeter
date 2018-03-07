/**
 * @file IcarusRev1.ino
 * @short Icarus Altimeter.
 *
 * @date Mar 28, 2015
 * @author Tennessee Carmel-Veilleux <tcv -at- ro.boto.ca>
 * @copyright 2015-2018 Tennessee Carmel-Veilleux
 *
 * MIT License. See LICENSE FILE
 */

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

typedef enum {
    STATE_GATHER_BASE,
    STATE_GATHER_MIN,
    STATE_DISPLAY  
} altitudeState_t;

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

static int g_sampleCount = 0;

// Pin 13 has a LED connected on most Arduino boards.
static int g_ledPin = 13;

static float g_baseAlpha = 0.05f;
static float g_basePressureHpa = -1.0f;
static float g_minPressureHpa = -1.0f;
// Threshold for take-off below base pressure
static float g_takeOffThreshold = 25.0f; // Approx 2m
static float g_resetThreshold = 104000.0f;
static altitudeState_t g_currentState = STATE_GATHER_BASE;

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
 */
/**************************************************************************/
void setup(void) {
    pinMode(g_ledPin, OUTPUT);
  
    Serial.begin(115200);
    Serial.println("Pressure Sensor Test"); 
    Serial.println("");

    /* Initialise the sensor */
    bmp.begin(BMP085_MODE_HIGHRES);
}

void blink(int n, int delay_ms) {
    while (n-- > 0) {
        digitalWrite(g_ledPin, HIGH);
        delay(delay_ms);
        digitalWrite(g_ledPin, LOW);
        delay(delay_ms);
    }
}

bool display_value(uint32_t value, int nbits, int led_pin, bool inverted, int start_stop_ms, int period_ms, int zero_time_ms, int one_time_ms, int (*stop_hook)(void)) {
    const int LED_ON_VALUE = inverted ? LOW : HIGH;
    const int LED_OFF_VALUE = inverted ? HIGH : LOW;

    // Display preamble
    digitalWrite(led_pin, LED_ON_VALUE);
    delay(start_stop_ms);
    digitalWrite(led_pin, LED_OFF_VALUE);
    delay(start_stop_ms);

    // Display bits MSB-first
    for (int bitpos = 0; bitpos < nbits; bitpos++) {
        int currentBit = value & (1 << (nbits - bitpos - 1));
        
        // Start LED on for bit
        digitalWrite(led_pin, LED_ON_VALUE);
        
        // Keep it on for zero/one bit duration
        int end_time_ms = 0;
        if (currentBit == 0) {
            delay(zero_time_ms);
            end_time_ms = max(0, period_ms - zero_time_ms);
        } else {
            delay(one_time_ms);
            end_time_ms = max(0, period_ms - one_time_ms);
        }
    
        // Turn LED off and finish bit period
        digitalWrite(led_pin, LED_OFF_VALUE);
        delay(end_time_ms);
    
        // Early return if stop hook exists and returns true
        if ((NULL != stop_hook) && (stop_hook() != 0)) {
            digitalWrite(led_pin, LED_OFF_VALUE);
            
            // Return false to signify abnormal completion
            return false;
        }      
    }

    // Wait for stop delay  
    delay(start_stop_ms);
  
    // Return true to signify normal completion
    return true;
}

int stopHook() {
    /* Get pressure from sensor */
    sensors_event_t event;
    bmp.getEvent(&event);

    if (event.pressure) {
        float pressure;
        bmp.getPressure(&pressure);
 
        // Early stop on pressure high threshold (blowing in holes)
        if (pressure > g_resetThreshold) {
            // Blink a long time before returning the stop condition to prevent false trigger on blow release
            blink(30, 20);
            return 1;
        }
    }
  
    return 0;
}

void state_gather_base(void) {
    /* Get pressure from sensor */
    sensors_event_t event;
    bmp.getEvent(&event);

    if (event.pressure)
    {
        float pressure;
        bmp.getPressure(&pressure);
        
        // Handle initializing with first sample
        if (g_basePressureHpa == -1.0f) {
            g_basePressureHpa = pressure;
        } 
          
        // Apply 1 pole low-pass filter with Alpha value
        g_basePressureHpa = g_basePressureHpa * (1.0f - g_baseAlpha) + (pressure * g_baseAlpha);
        
        // Display current pressure
        Serial.print((long)(pressure));
        Serial.print(",");
        Serial.println((long)(g_basePressureHpa));
        
        // Switch to flight data acquisition mode once threshold is met for take-off (pressure drop of threshold below baseline pressure)
        if (pressure < (g_basePressureHpa - g_takeOffThreshold)) {
            g_currentState = STATE_GATHER_MIN;
            g_minPressureHpa = -1.0f;
            Serial.println("GATHER_BASE->GATHER_MIN");
        }
    }
}

void state_gather_min(void) {
    /* Get pressiure from sensor */
    sensors_event_t event;
    bmp.getEvent(&event);

    if (event.pressure) {
        float pressure;
        bmp.getPressure(&pressure);
        
        if (g_minPressureHpa == -1.0f) {
            // Handle initializing with first sample
            g_minPressureHpa = pressure;
        } 
      
        // Find minimum value
        g_minPressureHpa = min(pressure, g_minPressureHpa);
            
        // Switch to display state once back under 3/4 of take-off threshold
        if (pressure > (g_basePressureHpa - (g_takeOffThreshold * 0.75f))) {
            g_currentState = STATE_DISPLAY;
            Serial.println("GATHER_MIN->DISPLAY");
        }
    }
}

void state_display(void) {
    // Determine altitude above ground level
    
    // Step 1, determine altitude of ground, assuming 101.3kPa at Sea level (assumption valid for altitude differences)
    float ground_altitude = bmp.pressureToAltitude(g_basePressureHpa, (101300.0 / 100.0));
    
    // Step 2, determine apogee altitude
    float apogee_altitude = bmp.pressureToAltitude(g_minPressureHpa, (101300.0 / 100.0));
    
    // Step 3, determine altitude difference
    float altitude_above_ground = apogee_altitude - ground_altitude;
    
    // Display result (0-819.2 meter in tenths of meter, range 0-8192)
    uint32_t alt_tenth_meters_13_bits = (uint32_t)(altitude_above_ground * 10.0) & 0x1fff;
    
    // Display min pressure on terminal
    Serial.println((long)(alt_tenth_meters_13_bits));
    
    // Display value on LEDs, changing back to gathering base pressure if early-terminated by stop hook
    if (!display_value(alt_tenth_meters_13_bits, 13, g_ledPin, false, 2000, 1000, 100, 500, &stopHook)) {
        g_currentState = STATE_GATHER_BASE;
        g_basePressureHpa = -1.0f;
        Serial.println("DISPLAY->GATHER_BASE");
    }
}

void loop(void) {
    switch(g_currentState) {
        case STATE_GATHER_BASE:
            state_gather_base();
            break;
        case STATE_GATHER_MIN:
            state_gather_min();
            break;
        case STATE_DISPLAY:
            state_display();
            break;
    }
}
