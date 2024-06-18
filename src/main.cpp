#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Preferences.h>

#include <NmraDcc.h>
#include <NeoPixelBus.h>
#include <ESPTelnet.h>

#include <aifes.h>
#include "model/aifes_e_f32_fnn.h"

#include "secrets.h"
#include "circularBuffer.h"

TaskHandle_t motorTaskHandle;

const uint16_t PixelCount = 8; // this example assumes 4 pixels, making it smaller will cause a failure
const uint8_t PixelPin = 13;  // make sure to set this to the correct pin, ignored for Esp8266
NeoPixelBus<NeoGrbFeature, NeoWs2812xMethod> strip(PixelCount, PixelPin);

ESPTelnet telnet;
IPAddress ip;
uint16_t  telnetPort = 23;

// This is the default DCC Address
#define DEFAULT_DECODER_ADDRESS 3

#define DCC_PIN     23

#define ENCODER_PIN 22

#define MOTOR_A_PIN 26
#define MOTOR_B_PIN 27

const int pwmFrequency = 19500;  // PWM frequency
const int pwmResolution = 12;  // PWM resolution (bits)
const int16_t pwmMax = pow(2, pwmResolution) - 1;

const int positionRange = 2000;//100; // max numeber of rotations before slowdown bias kicks in
// float rotationIntervals[positionRange * 5]; //if there is a motor stall we still want the data but the rotation count doesn't go up x3 seems to work well
// float pwmPercents[positionRange * 5];
int rotationIndex;

CircularBuffer<float, 32> rotations;
CircularBuffer<float, 32> pwms;

float targetInterval = 0.1;
uint32_t maxRotationIntervalMicros = 500 * 1000; //500ms

volatile uint32_t intervalStartTime = 0;
volatile uint32_t resetTime = 0;
volatile uint32_t intervalEndTime = 0;
volatile int8_t direction = 1;
volatile int16_t position = 0;

void rotationResetISR() {
	resetTime = micros();
}

void rotationEndISR() { 
	uint32_t currentTime = micros();
	if (currentTime - resetTime > 1000 && currentTime - intervalStartTime > 1000) { //no reasonable motor would run more than 60,000rpm in a model train
		if (intervalEndTime == 0) {
			intervalEndTime = currentTime;
		}

		position += direction;
	}
}

void encoderISR() {
	if (digitalRead(ENCODER_PIN) == HIGH)
	{
		rotationEndISR();
	}
	else {
		rotationResetISR();
	}

}

#include <QuickPID.h>
float Kp = 0.0012, Ki = 0.10, Kd = 0.0;
float PIDSetpoint, PIDInput, PIDOutput;
QuickPID myPID(&PIDInput, &PIDOutput, &PIDSetpoint, Kp, Ki, Kd,  /* OPTIONS */
	myPID.pMode::pOnError,                   /* pOnError, pOnMeas, pOnErrorMeas */
	myPID.dMode::dOnMeas,                    /* dOnError, dOnMeas */
	myPID.iAwMode::iAwCondition,             /* iAwCondition, iAwClamp, iAwOff */
	myPID.Action::direct);                   /* direct, reverse */


int16_t runPID(float& targetInterval) {
	PIDInput = 60.0 / (float)rotations[0]; //convert to rpm
	myPID.Compute();

	return (int16_t)(0.5 + ((float)PIDOutput * (float)pwmMax));
}

float kickStart = 0.6;
float stall = 0.5;

int16_t runFuzzyLogic(float& targetInterval) {
	float pwmOutput;
	if (rotations[0] == 1.0) {
		pwmOutput = kickStart;
		kickStart *= 1.005;

		if (rotations[1] != 1.0 && pwms[1] > 0.0 && pwms[1] > stall) {
			stall = pwms[1];
		}

	}
	else if (rotations[1] == 1.0) {
		pwmOutput = stall * 1.01;
		kickStart *= 0.99;
	}
	else if (rotations[0] > targetInterval) {
		pwmOutput = stall * 1.01;
	}
	else if (rotations[0] < targetInterval) {
		pwmOutput = stall * 1.01;
		stall *= 0.9999;
	}
	else {
		pwmOutput = pwms[0];
	}

	pwmOutput = constrain(pwmOutput, 0.010, 0.7);

	// if (targetInterval - 0.05 < rotationIntervals[rotationIndex - 1] < targetInterval + 0.05){
	// 	targetPWM = (targetPWM * 0.9) + (rotationIntervals[rotationIndex - 1] * 0.1);
	// }

	return (int16_t)(pwmOutput * (float)pwmMax);
}


int16_t runNeuralNetwork(float& targetInterval) {
	if (rotations[0] == 1.0) {
		targetInterval *= 0.95;
	}
	else {
		if (rotations[0] < 0.025) {
			targetInterval *= 1.1;
		}
		else if (rotations[0] > 0.1) {
			targetInterval *= 0.9;
		}
		targetInterval = constrain(targetInterval, rotations[0] - 0.02, rotations[0] + 0.02);
	}

	targetInterval = constrain(targetInterval, 0.01, 0.5);

	// telnet.printf("\n========\n");

	// float input_data[] = {
	// 	rotationIntervals[rotationIndex - 5],
	// 	pwmPercents[rotationIndex - 6],
	// 	rotationIntervals[rotationIndex - 4],
	// 	pwmPercents[rotationIndex - 5],
	// 	rotationIntervals[rotationIndex - 3],
	// 	pwmPercents[rotationIndex - 4],
	// 	rotationIntervals[rotationIndex - 2],
	// 	pwmPercents[rotationIndex - 3],
	// 	rotationIntervals[rotationIndex - 1],
	// 	pwmPercents[rotationIndex - 2],
	// 	targetInterval,
	// 	pwmPercents[rotationIndex - 1] };

	float input_data[] = {
		rotations[14],
		pwms[15],
		rotations[13],
		pwms[14],
		rotations[12],
		pwms[13],
		rotations[11],
		pwms[12],
		rotations[10],
		pwms[11],
		rotations[9],
		pwms[10],
		rotations[8],
		pwms[9],
		rotations[7],
		pwms[8],
		rotations[6],
		pwms[7],
		rotations[5],
		pwms[6],
		rotations[4],
		pwms[5],
		rotations[3],
		pwms[4],
		rotations[2],
		pwms[3],
		rotations[1],
		pwms[2],
		rotations[0],
		pwms[1],
		targetInterval,
		pwms[0]};

	// for (size_t i = 0; i < 8; i++){
	// 	telnet.printf("%f,\n", input_data[i]);
	// }
	// telnet.printf("========\n");

	float output_data[1];  // AIfES output data
	aifes_e_f32_fnn_inference((float*)input_data, (float*)output_data); //641 Parameter model ~0.43ms 

	// telnet.printf("%f,\n", output_data[0]);
	// telnet.printf("========\n");
	return (int16_t)(output_data[0] * (float)pwmMax);
}

void driveMotor(int16_t currentPWM) {

	if (direction > 0) {
		analogWrite(MOTOR_A_PIN, currentPWM);
		analogWrite(MOTOR_B_PIN, 0);
	}
	else {
		analogWrite(MOTOR_A_PIN, 0);
		analogWrite(MOTOR_B_PIN, currentPWM);
	}
}

void motorDatasetTask(void* parameter) {
	telnet.begin(telnetPort);
	telnet.loop();

	analogWriteFrequency(pwmFrequency);
	analogWriteResolution(pwmResolution);
	analogWrite(MOTOR_A_PIN, 0);
	analogWrite(MOTOR_B_PIN, 0);
	attachInterrupt(ENCODER_PIN, encoderISR, CHANGE);

	myPID.SetOutputLimits(0.0, 1.0);
	myPID.SetSampleTimeUs(1000);
	myPID.SetTunings(Kp, Ki, Kd);
	myPID.SetMode(myPID.Control::automatic);
	PIDSetpoint = 600;

	// 	// Access the last 6 items in the buffer
	// 	for (int i = 0; i < 6; i++) {
	// 		telnet.print(myBuffer[i]); // Print the i-th most recent element
	// 		telnet.print(" ");
	// 	}
	// 	telnet.println();
	// 	telnet.loop();
	// 	vTaskDelay(1000 / portTICK_PERIOD_MS); //let everything settle
	// }


	while (telnet.isConnected() == false) {
		telnet.loop();
	}

	while (true) {
		if (telnet.isConnected()) {
			int16_t currentPWM = 0;
			rotationIndex = 0;
			int targetPosition = random(0, positionRange);
			uint8_t pwmRandomnessMultiplier = 0;

			vTaskDelay(2000 / portTICK_PERIOD_MS); //let everything settle

			pwms.add(0.0);
			rotations.add(1.0);
			telnet.printf("1.000000,0.0000\r\n");

			direction = (targetPosition > position) ? 1 : -1;

			pwmRandomnessMultiplier = (uint8_t)random(0, 16);

			while (abs(position - targetPosition) > 10) {

				// if (rotations[0] < targetInterval){
				// 	pwmRandomnessMultiplier = (uint8_t)random(0, 16);
				// } else {
				// 	pwmRandomnessMultiplier = (uint8_t)random(0, 4);
				// }
				

				currentPWM = runNeuralNetwork(targetInterval);
				// currentPWM = runFuzzyLogic(targetInterval);
				// currentPWM = runPID(targetInterval);

				// PWM Noise Generator (critical for a good dataset)
				// if (rotations[0] == 1.0) {
				// 	currentPWM += pwmRandomnessMultiplier * (int16_t)random(-80, 80);
				// }
				// else if (rotations[1] == 1.0){
				// 	currentPWM += pwmRandomnessMultiplier * (int16_t)random(-320, 40);
				// }
				// else if (rotations[0] > targetInterval) {
				// 	currentPWM += pwmRandomnessMultiplier * (int16_t)random(-9, 10);
				// }
				// else if (rotations[0] < targetInterval) {
				// 	currentPWM += pwmRandomnessMultiplier * (int16_t)random(-10, 9);
				// }

				currentPWM += (int16_t)random(-80, 80);

				currentPWM = constrain(currentPWM, 0, pwmMax * 0.8);
				driveMotor(currentPWM);

				intervalStartTime = micros();
				intervalEndTime = 0;

				// pwmPercents[rotationIndex] = (float)currentPWM / (float)pwmMax;
				pwms.add((float)currentPWM / (float)pwmMax);

				do {
					telnet.loop();
				} while (intervalEndTime == 0 && micros() < intervalStartTime + maxRotationIntervalMicros);

				if (intervalEndTime == 0) {
					// rotationIntervals[rotationIndex] = 1.0; // motor stalled
					rotations.add(1.0);
				}
				else {
					// rotationIntervals[rotationIndex] = (float)(intervalEndTime - intervalStartTime) / (1000000.0);
					rotations.add((float)(intervalEndTime - intervalStartTime) / (1000000.0));
				}

				// telnet.printf("%0.3f,%0.6f,%0.4f\r\n", targetInterval, rotationIntervals[rotationIndex], pwmPercents[rotationIndex]);
				// telnet.printf("%0.3f,%0.6f,%0.4f\r\n", targetInterval, rotations[0], pwms[0]);
				telnet.printf("%0.6f,%0.4f\r\n", rotations[0], pwms[0]);
				// telnet.printf("%0.6f,%0.4f\r\n", rotationIntervals[rotationIndex], pwmPercents[rotationIndex]);

				rotationIndex++;
			}

			driveMotor(0);

		}
		else {
			telnet.loop();
			vTaskDelay(100 / portTICK_PERIOD_MS);
		}

	}
}

void otaTask(void* parameter) {
	ArduinoOTA.setPort(3232);

	// Hostname defaults to esp3232-[MAC]
	ArduinoOTA.setHostname("NIMRS");

	// No authentication by default
	// ArduinoOTA.setPassword("admin");

	// Password can be set with it's md5 value as well
	// MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
	// ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

	ArduinoOTA
		.onStart([]() {

		digitalWrite(25, LOW);
		analogWrite(MOTOR_A_PIN, 0);
		analogWrite(MOTOR_B_PIN, 0);

		digitalWrite(16, HIGH);

		String type;
		if (ArduinoOTA.getCommand() == U_FLASH)
			type = "sketch";
		else  // U_SPIFFS
			type = "filesystem";

		// NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
		Serial.println("Start updating " + type);
			})
		.onEnd([]() {
		// preferences.end();
		Serial.println("\nEnd");
			})
		.onProgress([](unsigned int progress, unsigned int total) {
		Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
		digitalWrite(16, !digitalRead(16));
			})
		.onError([](ota_error_t error) {
		Serial.printf("Error[%u]: ", error);
		if (error == OTA_AUTH_ERROR)
			Serial.println("Auth Failed");
		else if (error == OTA_BEGIN_ERROR)
			Serial.println("Begin Failed");
		else if (error == OTA_CONNECT_ERROR)
			Serial.println("Connect Failed. Firewall Issue ?");
		else if (error == OTA_RECEIVE_ERROR)
			Serial.println("Receive Failed");
		else if (error == OTA_END_ERROR)
			Serial.println("End Failed");
			});

	ArduinoOTA.setTimeout(30000);
	ArduinoOTA.begin();

	while (true) {
		ArduinoOTA.handle();
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
}

void setup() {
	// this resets all the addressable leds to an off state
	strip.Begin();
	strip.Show();

	Serial.begin(115200);

	WiFi.mode(WIFI_STA);
	WiFi.begin(ssid, password);
	WiFi.setTxPower(WIFI_POWER_19_5dBm); // Set WiFi RF power output level
	WiFi.setAutoReconnect(true);
	WiFi.setSleep(false);

	for (size_t i = 0; i < 50 && WiFi.waitForConnectResult() != WL_CONNECTED; i++) {
		vTaskDelay(100 / portTICK_PERIOD_MS);
		strip.Show();
	}

	if (WiFi.waitForConnectResult() != WL_CONNECTED) {
		Serial.println("Connection Failed! Rebooting...");
		ESP.restart();
	}

	pinMode(16, OUTPUT);
	digitalWrite(16, LOW);

	analogWrite(MOTOR_A_PIN, 0);
	analogWrite(MOTOR_B_PIN, 0);

	pinMode(25, OUTPUT);
	digitalWrite(25, HIGH);

	pinMode(2, INPUT_PULLUP);
	pinMode(15, OUTPUT);
	digitalWrite(15, LOW);

	pinMode(ENCODER_PIN, INPUT_PULLUP);
	pinMode(MOTOR_A_PIN, OUTPUT);
	pinMode(MOTOR_B_PIN, OUTPUT);

	if (digitalRead(2) == HIGH) {
		xTaskCreatePinnedToCore(otaTask, "otaTask", 10000, NULL, 5, NULL, 0);
		while (true) {
			vTaskDelay(1000 / portTICK_PERIOD_MS);
		}

	}
	else {
		xTaskCreatePinnedToCore(otaTask, "otaTask", 10000, NULL, 5, NULL, 0);
		vTaskDelay(1000 / portTICK_PERIOD_MS);

		xTaskCreatePinnedToCore(motorDatasetTask, "motorDatasetTask", 40000, NULL, 0, &motorTaskHandle, 0);
	}
}


void loop() {
	vTaskDelay(1000 / portTICK_PERIOD_MS);
}



// // Some global state variables
// uint8_t newLedState = 0;
// uint8_t lastLedState = 0;

// uint8_t dccDirection = 0;
// uint8_t lastDirection = 0;

// uint8_t dccSpeed = 0;
// // uint8_t lastSpeed = 0;
// uint8_t numSpeedSteps = SPEED_STEP_128;

// uint8_t motorStart;
// uint8_t motorMax;

// int motorMilliVolts = 0;

// // Structure for CV Values Table
// struct CVPair
// {
// 	uint16_t  CV;
// 	uint8_t   Value;
// };

// // CV Addresses we will be using
// #define CV_VSTART  2
// #define CV_VHIGH   5

// // Default CV Values Table
// CVPair FactoryDefaultCVs[] =
// {
// 	// The CV Below defines the Short DCC Address
//   {CV_MULTIFUNCTION_PRIMARY_ADDRESS, DEFAULT_DECODER_ADDRESS},

//   // Three Step Speed Table
//   {CV_VSTART, 150}, //155
//   {CV_VHIGH, 180}, //180

//   // These two CVs define the Long DCC Address
//   {CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB, CALC_MULTIFUNCTION_EXTENDED_ADDRESS_MSB(DEFAULT_DECODER_ADDRESS)},
//   {CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB, CALC_MULTIFUNCTION_EXTENDED_ADDRESS_LSB(DEFAULT_DECODER_ADDRESS)},

//   // ONLY uncomment 1 CV_29_CONFIG line below as approprate
//   //  {CV_29_CONFIG,                                      0}, // Short Address 14 Speed Steps
//   //{CV_29_CONFIG,                       CV29_F0_LOCATION}, // Short Address 28/128 Speed Steps
//   {CV_29_CONFIG, CV29_EXT_ADDRESSING | CV29_F0_LOCATION}, // Long  Address 28/128 Speed Steps  
// };

// NmraDcc  Dcc;

// uint8_t FactoryDefaultCVIndex = 0;

// // This call-back function is called when a CV Value changes so we can update CVs we're using
// void notifyCVChange(uint16_t CV, uint8_t Value)
// {
// 	switch (CV)
// 	{
// 	case CV_VSTART:
// 		motorStart = Value;
// 		break;

// 	case CV_VHIGH:
// 		motorMax = Value;
// 		break;
// 	}
// }

// void notifyCVResetFactoryDefault()
// {
// 	// Make FactoryDefaultCVIndex non-zero and equal to num CV's to be reset 
// 	// to flag to the loop() function that a reset to Factory Defaults needs to be done
// 	FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs) / sizeof(CVPair);
// };

// // This call-back function is called whenever we receive a DCC Speed packet for our address 
// void notifyDccSpeed(uint16_t Addr, DCC_ADDR_TYPE AddrType, uint8_t Speed, DCC_DIRECTION Dir, DCC_SPEED_STEPS SpeedSteps)
// {
// 	dccDirection = Dir;
// 	dccSpeed = Speed;
// 	numSpeedSteps = SpeedSteps;
// };

// // This call-back function is called whenever we receive a DCC Function packet for our address 
// void notifyDccFunc(uint16_t Addr, DCC_ADDR_TYPE AddrType, FN_GROUP FuncGrp, uint8_t FuncState)
// {

// 	if (FuncGrp == FN_0_4)
// 	{
// 		newLedState = (FuncState & FN_BIT_00) ? 1 : 0;
// 	}

// }

// This call-back function is called whenever we receive a DCC Packet
// #ifdef  DEBUG_DCC_MSG
// void notifyDccMsg(DCC_MSG* Msg)
// {
// 	Serial.print("notifyDccMsg: ");
// 	for (uint8_t i = 0; i < Msg->Size; i++)
// 	{
// 		Serial.print(Msg->Data[i], HEX);
// 		Serial.write(' ');
// 	}
// 	Serial.println();
// }
// #endif

// This call-back function is called by the NmraDcc library when a DCC ACK needs to be sent
// Calling this function should cause an increased 60ma current drain on the power supply for 6ms to ACK a CV Read
// So we will just turn the motor on for 8ms and then turn it off again.
// void notifyCVAck(void)
// {
// #ifdef DEBUG_DCC_ACK
// 	Serial.println("notifyCVAck");
// #endif

// 	digitalWrite(MOTOR_A_PIN, HIGH);
// 	digitalWrite(MOTOR_B_PIN, LOW);

// 	delay(8);

// 	digitalWrite(MOTOR_A_PIN, LOW);
// 	digitalWrite(MOTOR_B_PIN, LOW);
// }

// float powerPanicFactor = 1.0;

// float speed = 0.0;

// uint8_t kickStartPWM = 128;
// uint8_t kickStartPeriod = 15;
// uint8_t kickStartEndSpeed = 32;


/*void motorDatasetTask(void* parameter) {
	telnet.onConnect(onTelnetConnect);
	telnet.begin(telnetPort);
	analogReadResolution(12);

	analogWriteFrequency(28000);
	analogWriteResolution(10);

	// Setup the Pins for the Motor H-Bridge Driver
	pinMode(MOTOR_A_PIN, OUTPUT);
	pinMode(MOTOR_B_PIN, OUTPUT);

	Dcc.pin(DCC_PIN, 0);

	Dcc.init(MAN_ID_DIY, 10, FLAGS_MY_ADDRESS_ONLY | FLAGS_AUTO_FACTORY_DEFAULT, 0);

	// Uncomment to force CV Reset to Factory Defaults
	//notifyCVResetFactoryDefault();

	// Read the current CV values for vStart and vHigh
	motorStart = Dcc.getCV(CV_VSTART);
	motorMax = Dcc.getCV(CV_VHIGH);

	while (true)
	{
		telnet.loop();

		// You MUST call the Dcc.process() method frequently for correct library operation
		Dcc.process();

		if (dccSpeed > 1 && speed < 1.0)
		{
			if (dccDirection == DCC_DIR_FWD) {
				analogWrite(MOTOR_A_PIN, 0);
				analogWrite(MOTOR_B_PIN, kickStartPWM * 4);
			}
			else if (dccDirection == DCC_DIR_REV) {
				analogWrite(MOTOR_A_PIN, kickStartPWM * 4);
				analogWrite(MOTOR_B_PIN, 0);
			}

			vTaskDelay(kickStartPeriod / portTICK_PERIOD_MS);
			speed = kickStartEndSpeed;
		}

		speed = 0.5 * speed + 0.5 * (float)map(dccSpeed, 2, numSpeedSteps, 0, 255);

		if (speed < 1.0)
		{
			analogWrite(MOTOR_A_PIN, 0);
			analogWrite(MOTOR_B_PIN, 0);
		}
		else {


			do
			{
				// float newPowerPanicFactor = 0.6 + (12000.0 / (analogReadMilliVolts(36) * 4.3)) * 0.4;

				// if (newPowerPanicFactor > powerPanicFactor) {
				// 	powerPanicFactor = newPowerPanicFactor;
				// }
				// else {
				// 	powerPanicFactor = 0.8 * powerPanicFactor + 0.2 * newPowerPanicFactor;
				// }

				float pwmOut = map(speed, 0.0, 255.0, (float)motorStart * powerPanicFactor, (float)motorMax * powerPanicFactor) * 4.0;
				//float pwmOut = map(speed, 0.0, 255.0, (float)motorStart, (float)motorMax) * 4.0;



				if (dccDirection == DCC_DIR_FWD) {
					analogWrite(MOTOR_A_PIN, 0);
					analogWrite(MOTOR_B_PIN, pwmOut);
				}
				else if (dccDirection == DCC_DIR_REV) {
					analogWrite(MOTOR_A_PIN, pwmOut);
					analogWrite(MOTOR_B_PIN, 0);
				}

				if (powerPanicFactor > 1.05) {
					telnet.println(analogReadMilliVolts(36) * 4.3);
					if (powerPanicFactor > 1.25) {
						digitalWrite(25, LOW); //disable motor
						strip.ClearTo(RgbColor(0, 0, 0));
						strip.Show();
						// while (newPowerPanicFactor > 1.05)
						// {
						// 	vTaskDelay(100 / portTICK_PERIOD_MS);
						// 	newPowerPanicFactor = 0.6 + (12000.0 / (analogReadMilliVolts(36) * 4.3)) * 0.4;
						// 	telnet.println(analogReadMilliVolts(36)*4.3);
						// }

						speed = 0.0;
						digitalWrite(25, HIGH); //enable motor

						// esp_sleep_enable_ext0_wakeup(GPIO_NUM_23, 1);
						// esp_deep_sleep_start();
					}

				}

			} while (powerPanicFactor > 1.05);
		}

		// Handle resetting CVs back to Factory Defaults
		if (FactoryDefaultCVIndex && Dcc.isSetCVReady())
		{
			FactoryDefaultCVIndex--; // Decrement first as initially it is the size of the array
			Dcc.setCV(FactoryDefaultCVs[FactoryDefaultCVIndex].CV, FactoryDefaultCVs[FactoryDefaultCVIndex].Value);
		}

		vTaskDelay(2 / portTICK_PERIOD_MS);

	}


}*/


/*void motorDatasetGererator(void* parameter) {
	telnet.begin(telnetPort);
	//telnet.onConnect(onTelnetConnect);
	telnet.loop();

	const int pwmFrequency = 28000;  // PWM frequency
	const int pwmResolution = 10;  // PWM resolution (bits)
	const int pwmMax = pow(2, pwmResolution);

	analogWriteFrequency(pwmFrequency);
	analogWriteResolution(pwmResolution);

	const int maxRotations = 125; // max numeber of rotations before slowdown bias kicks in
	const int maxRotationIntervalMicros = 1000 * 1000; // max microseconds before calling the test a stall
	float rotationInterval;
	float rotationIntervals[maxRotations * 3]; //if there is a motor stall we still want the data but the rotation count doesn't go up x3 seems to work well
	float pwmPercents[maxRotations * 3];

	Gaussian bellCurveStartPWM = Gaussian(600, 64); // start around the motor kickstart voltage
	Gaussian bellCurveUnbiased = Gaussian(0.0, 16.0); // generate random pwm up and down changes, with a focus on smaller changes
	Gaussian bellCurveSlowDown = Gaussian(4.0, 2.0); // generate random biased pwm changes (to get the train moving if it is stalled and to slow it down once it is moving)


	while (true)
	{
		analogWrite(MOTOR_A_PIN, 0);
		analogWrite(MOTOR_B_PIN, 0);
		telnet.printf("%0.6f,%0.3f\r\n", 1.0, 0.0); //set dataset zero point
		attachInterrupt(ENCODER_PIN, rotationEndISR, RISING);

		direction = true; //set isr to positive rotations mode

		int rotationIndex = 0;
		int currentPWM = 0;
		int nextPWMBias = 0;
		bool stalled = true;
		int totalRotations = random(1, maxRotations); //numeber of rotations before slowdown bias kicks in

		vTaskDelay(1000 / portTICK_PERIOD_MS); //let everything settle

		while (position < totalRotations || stalled == false) {

			if (position >= totalRotations) {
				nextPWMBias = -(int)bellCurveSlowDown.random(); //decrease pwm at end of track
				currentPWM = reflectedConstrain(currentPWM + nextPWMBias + (int)bellCurveUnbiased.random(), 0, 700); //constrain pwm to prevent wheel slip
			}
			else if (stalled) {
				currentPWM = constrain((int)bellCurveStartPWM.random(), 0, 1024);
			}
			else {
				currentPWM = reflectedConstrain(currentPWM + nextPWMBias + (int)bellCurveUnbiased.random(), 0, pwmMax); //constrain pwm and reflect it to stop pwm crowding around min and max
			}

			analogWrite(MOTOR_A_PIN, currentPWM);
			uint32_t intervalStartTime = micros();
			intervalEndTime = 0;
			pwmPercents[rotationIndex] = (float)currentPWM / (float)pwmMax;

			while (intervalEndTime == 0 && micros() < intervalStartTime + maxRotationIntervalMicros) {
				telnet.loop();
			}

			if (intervalEndTime == 0) {
				rotationIntervals[rotationIndex] = 1.0; // motor stalled or <60rpm
				stalled = true;

			}
			else {
				uint32_t interval = intervalEndTime - intervalStartTime;
				rotationIntervals[rotationIndex] = (float)interval / (1000000.0);
				stalled = false;
			}

			telnet.printf("%0.6f,%0.3f\r\n", rotationIntervals[rotationIndex], pwmPercents[rotationIndex]);

			rotationIndex++;
		}

		//Drive the train back to the start of the test track...
		direction = false; //set isr to negative rotations mode
		analogWrite(MOTOR_A_PIN, 0);
		analogWrite(MOTOR_B_PIN, 675);

		// Print the dataset to Telnet
		// for (int i = 0; i < rotationIndex; i++) {
		// 	telnet.printf("%0.6f,%0.3f\r\n", rotationIntervals[i], pwmPercents[i]);
		// 	telnet.loop();
		// }

		while (position > 0) { //wait to get back to start of test track...
			telnet.loop();
		}
	}
}*/

