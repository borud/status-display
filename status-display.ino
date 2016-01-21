/**
 * Driver for Arduino-based status display with NUM_SERVOS servos that
 * are mounted inside dials.  Uses a custom designed interface card
 * for the servos, but you can connect the servos directly to the pins
 * as well.
 *
 * Needs a bit of cleanup, but this is workable.
 *
 * 
 * TODO(borud): Fix return codes so they are more consistent and
 *              document them
 *
 * TODO(borud): Make wakeup time proportional to longest travel needed
 *              so we can reduce amount of time needed for servos to
 *              be on.  They are noisy and annoying.
  *
 * @author borud
 */

#include <string.h>
#include <Servo.h>

#define NUM_SERVOS   4
#define SERVO_MIN    1
#define SERVO_MAX  179

#define SERVO_MIN_PULSE_WIDTH 500
#define SERVO_MAX_PULSE_WIDTH 2500

#define FIRST_SERVO_PIN     7
#define ENABLE_PIN          3
#define SERIAL_BUFFER_SIZE 80

// Whenever we write to the servos we keep the power on for
// a few seconds to make sure the servos have time to react.
#define WAKEUP_MILLIS           2100

// Wake up servos every 10 minutes to adjust for drift
#define PERIODIC_WAKEUP_MILLIS  600000l

// Define how we map input values to the servos and read them from the
// servo lib.  This just maps a 0-100 value range to the dials.
#define MAP_SERVO(x) map(x, 0, 100, SERVO_MAX, SERVO_MIN)
#define MAP_SERVO_REVERSE(x) map(x, SERVO_MAX, SERVO_MIN, 0, 100)

static Servo servos[NUM_SERVOS];
static char buffer[SERIAL_BUFFER_SIZE];
static byte buffer_offset = 0;
static boolean overflow_mode = false;

// Variables for tracking wakeup state and the timestamp of the last
// wakeup period start.
static boolean wakeup = false;
static long last_wakeup = 0;


void setup() {
    Serial.begin(9600);

    // Set up the enable pin
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, LOW);
    
    // Attach all the servos and "zero" the values
    for (int i = 0; i < NUM_SERVOS; i++) {
        servos[i].attach(FIRST_SERVO_PIN + i, SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH);
        servos[i].write(MAP_SERVO(0));
    }
    
    memset(buffer, 0, SERIAL_BUFFER_SIZE);

    show_initial();

    Serial.print(F("100 READY - "));
    Serial.print(NUM_SERVOS);
    Serial.println(F(" SERVOS"));
}

/**
 * Whenever this is called we turn on the enable pin for the servos
 * and record the time when the servos were turned on.  After
 * WAKEUP_MILLIS milliseconds we turn them off again.
 */
void servo_wakeup() {
    digitalWrite(ENABLE_PIN, HIGH);
    last_wakeup = millis();
    wakeup = true;

    // Attach all the servos
    for (int i = 0; i < NUM_SERVOS; i++) {
        servos[i].attach(FIRST_SERVO_PIN + i, SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH);
    }
    
    Serial.println("110 SERVO ON");
}

/**
 * This gets called from the main loop to determine if we are in
 * wakeup mode and if so, if it is time to set the enable pin low and
 * shut down the servos.
 *
 */
void servo_wakeup_timeout() {
    long now = millis();
    
    if (wakeup) {
        if ((now - last_wakeup) > WAKEUP_MILLIS) {
            digitalWrite(ENABLE_PIN, LOW);
            wakeup = false;

            // For some reason the lower spec servos I've tried tend to 
            for (int i = 0; i < NUM_SERVOS; i++) {
                servos[i].detach();
            }
            
            Serial.println("111 SERVO OFF");
        }
    }
}

/**
 * Upon startup, just run all of the servos to the maximum value and
 * then to 0.
 */
void show_initial() {
    servo_wakeup();
    for (int i = 0; i < NUM_SERVOS; i++) {
        servos[i].write(MAP_SERVO(100));
    }

    delay(750);
    
    for (int i = 0; i < NUM_SERVOS; i++) {
        servos[i].write(MAP_SERVO(0));
    }
}

/**
 * Show the help text on the serial line.
 */
void show_help() {
    Serial.println(F("101 s (status)  - show status of all servos"));
    Serial.println(F("101 a <n>       - set all servos to <n>"));
    Serial.println(F("101 c (clear)   - set all servos to 0"));        
    Serial.println(F("101 w (wakeup)  - wake up the servos for a few seconds"));
    Serial.println(F("101 h (help)    - show this text"));
    Serial.println(F("101 <n> <v>     - set servo n to value v (0..100)"));
}

/**
 * Show the state of each servo.
 */
void show_status() {
    for (int i = 0; i < NUM_SERVOS; i++) {
        Serial.print(F("102 "));
        Serial.print(i);
        Serial.print(":");
        Serial.println(MAP_SERVO_REVERSE(servos[i].read()));
    }
}

/**
 * Set all servos to the same value.  Useful for testing.
 */
void set_all(int value) {
    if (value > 100) {
        return;
    }
    
    for (int i = 0; i < NUM_SERVOS; i++) {
        servos[i].write(MAP_SERVO(value));
    }
    servo_wakeup();
}

/**
 * The format of this protocol is:
 *
 *   <servo> <value>
 *
 * Where servo is between 1 and NUM_SERVOS-1 and value is between 0
 * and 100.
 */
char* parse_command(char* s) {
    static char message_buffer[50];

    // Deal with the regular commands first
    switch (s[0]) {
        case 'w':
            servo_wakeup();
            return "206 WAKEUP OK";

        case 'h':
            show_help();
            return "201 OK";
            
        case 's':
            show_status();
            return "201 OK";
        
        case 'c':
            set_all(0);
            return "203 OK - ALL SERVOS CLEARED";

        case 'a':
            set_all(atoi(s+1));
            return "204 OK - ALL SERVOS SET";

        default:
            break;
    }

    // Invariant: none of the above commands matched so we will assume
    // that the next command is for setting servo values.
    char* val = NULL;
    for (byte b = 0; b < SERIAL_BUFFER_SIZE; b++) {
        if (s[b] == ' ') {
            s[b] = NULL;
            val = s + b + 1;
            break;
        }
    }

    if (NULL == val) {
        return "502 SYNTAX ERROR";
    }

    int servo = atoi(s);
    int value = atoi(val);

    if ((servo < 0) || (servo > (NUM_SERVOS - 1))) {
        return "503 SERVO OUT OF RANGE";
    }

    if ((value < 0) || (value > 100)) {
        return "504 VALUE OUT OF RANGE";
    }

    servos[servo].write(MAP_SERVO(value));
    servo_wakeup();

    snprintf(message_buffer, sizeof(message_buffer), "200 OK - %d:%d", servo, value);
    return message_buffer;
}

/**
 * 
 */
void read_serial() {
    int loop_count = 0;
    
    while (Serial.available()) {
        buffer[buffer_offset] = Serial.read();

        // If we are in overflow mode just dump chars until we see a
        // NL or we have reached the loop count.  Reaching the max
        // loop count dumps us out and gives the rest of the main
        // loop() a chance to get something done.
        if (overflow_mode) {
            loop_count++;

            if ('\n' == buffer[buffer_offset]) {
                overflow_mode = false;
                return;
            }

            if (loop_count == SERIAL_BUFFER_SIZE) {
                Serial.println("501 OVERFLOW");
                return;
            }
        }

        // If we encounter a newline we have an entire command so we
        // parse it and reset the buffer offset.
        if ('\n' == buffer[buffer_offset]) {
            buffer[buffer_offset] = 0;
            buffer_offset = 0;            
            Serial.println(parse_command(buffer));
            return;
        }

        // Advance buffer offset if the previous character was not a NL
        buffer_offset++;

        // If the buffer is full we just reset the buffer and dump the
        // contents.
        if (SERIAL_BUFFER_SIZE == buffer_offset) {
            buffer_offset = 0;
            overflow_mode = true;
            Serial.println("501 OVERFLOW");
            return;
        }
    }
}

/**
 * Check if it is time for executing a periodic wakeup of the servos.
 */
void periodic_wakeup() {
    static long last_periodic_wakeup = 0;

    long now = millis();
    if ((now - last_periodic_wakeup) > PERIODIC_WAKEUP_MILLIS) {
        servo_wakeup();
        last_periodic_wakeup = now;
        Serial.println(PERIODIC_WAKEUP_MILLIS);
    }
}

/**
 * The main loop.
 */
void loop() {
    read_serial();
    periodic_wakeup();
    servo_wakeup_timeout();
}

