/*
 * Copyright (c) 2016 Trybots. All Rights Reserved.
 *
 *
 */

#include<Servo.h>

#define LOG_ENABLED
//#define TEST_MODE
#define UNIT_ID					1
#define CURIE_BLE

/******************************************************************************
 * Consts
 ******************************************************************************/
#define TIMEOUT					500

#define JOYSTICK_FORWARD_PIN	4
#define JOYSTICK_RIGHT_PIN		5
#define JOYSTICK_FORWARD2_PIN	6
#define JOYSTICK_LEFT_PIN		7

#ifdef CURIE_BLE
/* XXX: Do not use pin 13 for Arduino 101 */
#define MOTOR_PIN				12
#else
#define MOTOR_PIN				13
#endif

#define NUM_SERVO				2
#define SERVO_ABS_MIN			20
#define SERVO_ABS_MAX			160
#define SERVO_CENTER			90
#define SERVO_NECK_ID			0
#define SERVO_FOOT_ID			1

#define SERVO_NECK_PIN			9
#ifdef CURIE_BLE
#define SERVO_FOOT_PIN			3
#else
#define SERVO_FOOT_PIN			10
#endif

#if (UNIT_ID == 0)
#define SERVO_NECK_OFFSET		-9
#define SERVO_NECK_MIN			75
#define SERVO_NECK_MAX			105
#define SERVO_FOOT_OFFSET		10
#define SERVO_FOOT_MIN			75
#define SERVO_FOOT_MAX			105
#elif (UNIT_ID == 1)
#define SERVO_NECK_OFFSET		15
#define SERVO_NECK_MIN			75
#define SERVO_NECK_MAX			105
#define SERVO_FOOT_OFFSET		10
#define SERVO_FOOT_MIN			75
#define SERVO_FOOT_MAX			105
#else
#error "Unknown Unit ID"
#endif

#ifdef TEST_MODE
#define TEST_ID					SERVO_FOOT_ID
#define TEST_DELAY				500
#endif

/******************************************************************************
 * Servo
 ******************************************************************************/
typedef struct {
	Servo servo;
	int id;
	int pin;
	int offset;
	int min;
	int max;
	int prevAngle;
} ServoDef_t;

void ServoInit(ServoDef_t *servo) {
#ifdef LOG_ENABLED
	Serial.print("Servo Init: id=");
	Serial.print(servo->id);
	Serial.print(", pin=");
	Serial.println(servo->pin);
#endif
	servo->prevAngle = 0;
	servo->servo.attach(servo->pin);
}

void ServoWrite(ServoDef_t *servo, int angle) {
	int actualAngle;
	actualAngle = angle;
	if (actualAngle > servo->max) {
		actualAngle = servo->max;
	}
	if (actualAngle < servo->min) {
		actualAngle = servo->min;
	}
	actualAngle += servo->offset;
	if (actualAngle > SERVO_ABS_MAX) {
		actualAngle = SERVO_ABS_MAX;
	}
	if (actualAngle < SERVO_ABS_MIN) {
		actualAngle = SERVO_ABS_MIN;
	}
	if (actualAngle != servo->prevAngle) {
#ifdef LOG_ENABLED
		Serial.print("Servo Write: id=");
		Serial.print(servo->id);
		Serial.print(", angle=");
		Serial.print(angle);
		Serial.print(", actualAngle=");
		Serial.println(actualAngle);
#endif
		servo->servo.write(actualAngle);
		servo->prevAngle = actualAngle;
	}
}

/******************************************************************************
 * BLE (Curie)
 ******************************************************************************/
#ifdef CURIE_BLE
#include <BLEAttribute.h>
#include <BLECentral.h>
#include <BLECharacteristic.h>
#include <BLECommon.h>
#include <BLEDescriptor.h>
#include <BLEPeripheral.h>
#include <BLEService.h>
#include <BLETypedCharacteristic.h>
#include <BLETypedCharacteristics.h>
#include <BLEUuid.h>
#include <CurieBLE.h>

#define STATUS_PIN		11
BLEPeripheral blePeripheral;
BLEService tbService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
BLECharacteristic servoChar("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", BLEWriteWithoutResponse, 2);
BLECharacteristic motorChar("6E400004-B5A3-F393-E0A9-E50E24DCCA9E", BLEWriteWithoutResponse, 1);
#endif

/******************************************************************************
 * Main
 ******************************************************************************/
ServoDef_t servoNeck;
ServoDef_t servoFoot;
ServoDef_t *servoTable[NUM_SERVO];
int forward1, forward2, right, left;

void setup() {
#ifdef LOG_ENABLED
	Serial.begin(115200);
#endif
	/* Servo - Neck */
	servoNeck.id = SERVO_NECK_ID;
	servoNeck.pin = SERVO_NECK_PIN;
	servoNeck.offset = SERVO_NECK_OFFSET;
	servoNeck.min = SERVO_NECK_MIN;
	servoNeck.max = SERVO_NECK_MAX;
	servoTable[SERVO_NECK_ID] = &servoNeck;

	/* Servo - Foot */
	servoFoot.id = SERVO_FOOT_ID;
	servoFoot.pin = SERVO_FOOT_PIN;
	servoFoot.offset = SERVO_FOOT_OFFSET;
	servoFoot.min = SERVO_FOOT_MIN;
	servoFoot.max = SERVO_FOOT_MAX;
	servoTable[SERVO_FOOT_ID] = &servoFoot;

	/* Init Servo */
	for (int i = 0; i < NUM_SERVO; i++) {
		ServoInit(servoTable[i]);
		ServoWrite(servoTable[i], SERVO_CENTER);
	}

	/* Init GPIO */
	pinMode(JOYSTICK_FORWARD_PIN, INPUT);
	pinMode(JOYSTICK_FORWARD2_PIN, INPUT);
	pinMode(JOYSTICK_RIGHT_PIN, INPUT);
	pinMode(JOYSTICK_LEFT_PIN, INPUT);
	pinMode(MOTOR_PIN, OUTPUT);

	digitalWrite(MOTOR_PIN, LOW);
#ifdef CURIE_BLE
	pinMode(STATUS_PIN, OUTPUT);

	/* Init BLE */
	blePeripheral.setLocalName("Trybots");
	blePeripheral.setAdvertisedServiceUuid(tbService.uuid());
	blePeripheral.addAttribute(tbService);
	blePeripheral.addAttribute(servoChar);
	blePeripheral.addAttribute(motorChar);

	blePeripheral.begin();
#ifdef LOG_ENABLED
	Serial.println("Bluetooth device active, waiting for connections...");
#endif
#endif
}

#ifdef TEST_MODE
void loop() {
#ifdef TEST_MODE_LOOP
	ServoDef_t *servo = servoTable[TEST_ID];
	int pos = SERVO_CENTER;
	int delta = 1;
	while (true) {
		ServoWrite(servo, pos);
		pos += delta;
		if (pos >= servo->max) {
			delta = -1;
		}
		if (pos <= servo->min) {
			delta = 1;
		}
		delay(TEST_DELAY);
	}
#endif
}
#else
#ifdef CURIE_BLE
void loop() {
	BLECentral central = blePeripheral.central();
	if (central) {
#ifdef LOG_ENABLED
		Serial.print("Connected to central: ");
		Serial.println(central.address());
#endif
		digitalWrite(STATUS_PIN, HIGH);
		while (central.connected()) {
			if (servoChar.written()) {
#ifdef LOG_ENABLED
				Serial.print("Servo Write: ");
				Serial.println(servoChar.value()[0]);
#endif
				ServoWrite(servoTable[SERVO_NECK_ID], servoChar.value()[0]);
				ServoWrite(servoTable[SERVO_FOOT_ID], servoChar.value()[1]);
			}
			if (motorChar.written()) {
#ifdef LOG_ENABLED
				Serial.print("Motor Write: ");
				Serial.println(motorChar.value()[0]);
#endif
				digitalWrite(MOTOR_PIN, motorChar.value()[0] ? HIGH : LOW);
			}
		}
		/* Default */
		ServoWrite(servoTable[SERVO_NECK_ID], SERVO_CENTER);
		ServoWrite(servoTable[SERVO_FOOT_ID], SERVO_CENTER);
		digitalWrite(MOTOR_PIN, LOW);
		digitalWrite(STATUS_PIN, LOW);
#ifdef LOG_ENABLED
		Serial.print("Disconnected from central: ");
		Serial.println(central.address());
#endif
	}
}
#else
void tick() {
	/* Forward */
#ifdef LOG_ENABLED
	Serial.print("Forward: ");
	Serial.println(forward1 || forward2);
#endif
	digitalWrite(MOTOR_PIN, (forward1 || forward2) ? HIGH : LOW);

	if (right) {
		/* Right */
#ifdef LOG_ENABLED
	Serial.println("Right");
#endif
		ServoWrite(servoTable[SERVO_NECK_ID], SERVO_NECK_MAX);
		ServoWrite(servoTable[SERVO_FOOT_ID], SERVO_FOOT_MAX);
	} else if (left) {
		/* Left */
#ifdef LOG_ENABLED
	Serial.println("Left");
#endif
		ServoWrite(servoTable[SERVO_NECK_ID], SERVO_NECK_MIN);
		ServoWrite(servoTable[SERVO_FOOT_ID], SERVO_FOOT_MIN);
	} else {
		/* Center */
#ifdef LOG_ENABLED
	Serial.println("Center");
#endif
		ServoWrite(servoTable[SERVO_NECK_ID], SERVO_CENTER);
		ServoWrite(servoTable[SERVO_FOOT_ID], SERVO_CENTER);
	}
}

void loop() {
	static int counter = 0;
	forward1 = digitalRead(JOYSTICK_FORWARD_PIN);
	forward2 = digitalRead(JOYSTICK_FORWARD2_PIN);
	right = digitalRead(JOYSTICK_RIGHT_PIN);
	left = digitalRead(JOYSTICK_LEFT_PIN);
	if (++counter > TIMEOUT) {
		counter = 0;
		tick();
	}
}
#endif
#endif
