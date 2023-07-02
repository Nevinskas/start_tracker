#include <AccelStepper.h>

#define encoder_btn 2
#define encoder_clk 3
#define encoder_dat 4

#define btn1 5
#define btn2 6

#define HALFSTEP 8

#define motor_pin1 11   // IN1 on the ULN2003 driver 1
#define motor_pin2 10   // IN2 on the ULN2003 driver 1
#define motor_pin3 9  // IN3 on the ULN2003 driver 1
#define motor_pin4 8  // IN4 on the ULN2003 driver 1

#define switch_pin 12

#define internal_led LED_BUILTIN

AccelStepper stepper1(HALFSTEP, motor_pin1, motor_pin3, motor_pin2, motor_pin4);

float stepper_speed = 150;
int encoder_last_clk_state;

void blink() {
	digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on
	delay(10);                        // wait for half a second
	digitalWrite(LED_BUILTIN, LOW);   // turn the LED off
	delay(10);                        // wait for half a second
}

void setup_motor() {
	float stepper_max_speed = 1000;
	float stepper_acceleration = 100;

	stepper1.setMaxSpeed(stepper_max_speed);
	stepper1.setAcceleration(stepper_acceleration);
	stepper1.setSpeed(stepper_speed);
}

void setup_encoder() {
	// Set encoder pins as inputs
	pinMode(encoder_clk, INPUT_PULLUP);
	pinMode(encoder_dat, INPUT_PULLUP);
	pinMode(encoder_btn, INPUT_PULLUP);

	// Read the initial state of CLK
	encoder_last_clk_state = digitalRead(encoder_clk);

	// Call encoder_isr() when any high/low changed seen
	attachInterrupt(digitalPinToInterrupt(encoder_clk), encoder_isr, CHANGE);
}


void setup_btn() {
	pinMode(btn1, INPUT_PULLUP);
	digitalWrite(btn1, HIGH);

	pinMode(btn2, INPUT_PULLUP);
	digitalWrite(btn2, HIGH);
}

void setup() {
	Serial.begin(9600);

	// Setup onboard led
	pinMode(internal_led, OUTPUT);

	// Setup main switch:
	pinMode(switch_pin, INPUT_PULLUP);
	digitalWrite(switch_pin, HIGH);

	setup_motor();
	setup_encoder();
	setup_btn();
}

void loop() {
	int sw_state = 0;
	int sw_last_state = 0;

	// read the state of the switch value:
	sw_state = digitalRead(switch_pin);

	if (sw_state == HIGH) {
		stepper1.runSpeed();
		digitalWrite(internal_led, HIGH);
	} else {
		digitalWrite(internal_led, LOW);
	}

#if 0
	if (digitalRead(btn1) == LOW) {
		blink();
		stepper_speed -= 0.1;
		stepper1.setSpeed(stepper_speed);
	}

	if (digitalRead(btn2) == LOW) {
		blink();
		stepper_speed += 0.1;
		stepper1.setSpeed(stepper_speed);
	}
#endif
}

void encoder_isr() {
	int encoder_cur_clk_state;
	int encoder_cur_dat_state;
	String currentDir = "";

	// Read the current state of CLK
	encoder_cur_clk_state = digitalRead(encoder_clk);
	encoder_cur_dat_state = digitalRead(encoder_dat);

	// If last and current state of CLK are different, then pulse occurred
	// React to only 1 state change to avoid double count
	if (encoder_cur_clk_state != encoder_last_clk_state && encoder_cur_clk_state == 0) {
		float step = 0.1;

		if (!digitalRead(encoder_btn)) {
			step = 10;
		}

		if (encoder_cur_dat_state > encoder_cur_clk_state) {
			stepper_speed -= step;
			currentDir = "CW";
		} else {
			// Encoder is rotating CW so increment
			stepper_speed += step;
			currentDir = "CCW ";
		}

		stepper1.setSpeed(stepper_speed);

#if 1
		Serial.print(currentDir);
		Serial.print(" | ");
		Serial.println(stepper_speed);
#endif
	}

	// Remember last CLK state
	encoder_last_clk_state = encoder_cur_clk_state;
}
