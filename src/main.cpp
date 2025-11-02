#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <ESP32Servo.h>
#include <PID_v1.h>
#include <Kalman.h>

namespace {

// ===== Hardware Pin Definitions ===== //
constexpr uint8_t SDA_PIN = 22;
constexpr uint8_t SCL_PIN = 23;

constexpr uint8_t CH1_PIN = 34;  // Aileron
constexpr uint8_t CH2_PIN = 35;  // Elevator
constexpr uint8_t CH4_PIN = 32;  // Rudder
constexpr uint8_t CH6_PIN = 33;  // Stabilization mode

constexpr uint8_t SERVO_RIGHT_AILERON_PIN = 21;
constexpr uint8_t SERVO_LEFT_AILERON_PIN = 19;
constexpr uint8_t SERVO_ELEVATOR_PIN = 18;
constexpr uint8_t SERVO_RUDDER_PIN = 5;

// ===== RC Signal Parameters ===== //
constexpr int RC_MIN_PULSE_US = 1000;
constexpr int RC_MAX_PULSE_US = 2000;
constexpr int RC_DEFAULT_PULSE_US = 1500;
constexpr uint32_t RC_SIGNAL_TIMEOUT_US = 50000;  // Treat as lost after 50 ms

// ===== Control Loop Timing ===== //
constexpr uint32_t LOOP_PERIOD_US = 4000;  // 250 Hz loop
constexpr uint32_t TELEMETRY_PERIOD_MS = 100;

// ===== Servo Output Bounds ===== //
constexpr int SERVO_MIN_US = 1000;
constexpr int SERVO_MAX_US = 2000;
constexpr int SERVO_NEUTRAL_US = 1500;
constexpr int SERVO_OUTPUT_RANGE_US = 300;  // Max PID correction magnitude. 0-500 us. <150 us for weaker stabilization, 250-350 for mid range stabilization, 500 for strongest stabilization. 

constexpr bool INVERT_LEFT_AILERON = false;  // Mirror the left aileron output if the mechanical linkage is not already inverted.

// ===== Kalman Filter Settings ===== //
struct KalmanSettings {
	float qAngle;
	float qBias;
	float rMeasure;
};

KalmanSettings kalmanSettingsRoll{0.001f, 0.003f, 0.03f};
KalmanSettings kalmanSettingsPitch{0.001f, 0.003f, 0.03f};

// ===== PID Settings ===== //
struct PIDSettings {
	double kp;
	double ki;
	double kd;
	double outputLimit;
};

// Max setpoint angles. 0-180. Maximum deflection from neutral (90). 
constexpr float MAX_SETPOINT_ROLL_DEG = 90.0f; 
constexpr float MAX_SETPOINT_PITCH_DEG = 90.0f;
constexpr float MAX_SETPOINT_YAW_DEG = 90.0f;

PIDSettings rollPidSettings{1.0, 0.0, 0.0, MAX_SETPOINT_ROLL_DEG};
PIDSettings pitchPidSettings{1.0, 0.0, 0.0, MAX_SETPOINT_PITCH_DEG};
PIDSettings yawPidSettings{1.0, 0.0, 0.0, MAX_SETPOINT_YAW_DEG};

constexpr float ROLL_US_PER_DEG = static_cast<float>(SERVO_OUTPUT_RANGE_US) / MAX_SETPOINT_ROLL_DEG;    // Converts roll correction degrees to servo microseconds.
constexpr float PITCH_US_PER_DEG = static_cast<float>(SERVO_OUTPUT_RANGE_US) / MAX_SETPOINT_PITCH_DEG;  // Converts pitch correction degrees to servo microseconds.
constexpr float YAW_US_PER_DEG = static_cast<float>(SERVO_OUTPUT_RANGE_US) / MAX_SETPOINT_YAW_DEG;      // Converts yaw correction degrees to servo microseconds.

// ===== RC Input Handling ===== //
struct RCChannelState {
	uint8_t pin;
	volatile uint32_t pulseStart;
	volatile uint32_t pulseWidth;
	volatile uint32_t lastUpdate;
};

enum RCChannelIndex : uint8_t { RC_CH1 = 0, RC_CH2, RC_CH4, RC_CH6, RC_CHANNEL_COUNT };

RCChannelState rcChannels[RC_CHANNEL_COUNT] = {
	{CH1_PIN, 0, RC_DEFAULT_PULSE_US, 0},
	{CH2_PIN, 0, RC_DEFAULT_PULSE_US, 0},
	{CH4_PIN, 0, RC_DEFAULT_PULSE_US, 0},
	{CH6_PIN, 0, RC_DEFAULT_PULSE_US, 0},
};

portMUX_TYPE rcMux = portMUX_INITIALIZER_UNLOCKED;

inline void IRAM_ATTR handleChannelEdge(RCChannelState &channel) {
	const uint32_t now = micros();
	if (digitalRead(channel.pin) == HIGH) {
		channel.pulseStart = now;
	} else if (channel.pulseStart != 0U) {
		const uint32_t width = now - channel.pulseStart;
		portENTER_CRITICAL_ISR(&rcMux);
		channel.pulseWidth = width;
		channel.lastUpdate = now;
		portEXIT_CRITICAL_ISR(&rcMux);
		channel.pulseStart = 0U;
	}
}

void IRAM_ATTR handleCh1Interrupt() { handleChannelEdge(rcChannels[RC_CH1]); }
void IRAM_ATTR handleCh2Interrupt() { handleChannelEdge(rcChannels[RC_CH2]); }
void IRAM_ATTR handleCh4Interrupt() { handleChannelEdge(rcChannels[RC_CH4]); }
void IRAM_ATTR handleCh6Interrupt() { handleChannelEdge(rcChannels[RC_CH6]); }

int readChannel(RCChannelIndex index, int minimum = RC_MIN_PULSE_US, int maximum = RC_MAX_PULSE_US, int fallback = RC_DEFAULT_PULSE_US) {
	uint32_t width = RC_DEFAULT_PULSE_US;
	uint32_t lastUpdate = 0;

	portENTER_CRITICAL(&rcMux);
	width = rcChannels[index].pulseWidth;
	lastUpdate = rcChannels[index].lastUpdate;
	portEXIT_CRITICAL(&rcMux);

	const uint32_t now = micros();
	if (lastUpdate == 0U || (now - lastUpdate) > RC_SIGNAL_TIMEOUT_US || width == 0U) {
		return fallback;
	}

	const int pulse = constrain(static_cast<int>(width), minimum, maximum);
	return pulse;
}

// ===== Global Hardware Objects ===== //
MPU6050 mpu;
Servo servoAileronRight;
Servo servoAileronLeft;
Servo servoElevator;
Servo servoRudder;

Kalman kalmanRoll;
Kalman kalmanPitch;

// ===== PID Controller State ===== //
double rollSetpoint{0.0};
double rollInput{0.0};
double rollOutput{0.0};

double pitchSetpoint{0.0};
double pitchInput{0.0};
double pitchOutput{0.0};

double yawSetpoint{0.0};
double yawInput{0.0};
double yawOutput{0.0};

PID rollPID(&rollInput, &rollOutput, &rollSetpoint, rollPidSettings.kp, rollPidSettings.ki, rollPidSettings.kd, DIRECT);
PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, pitchPidSettings.kp, pitchPidSettings.ki, pitchPidSettings.kd, DIRECT);
PID yawPID(&yawInput, &yawOutput, &yawSetpoint, yawPidSettings.kp, yawPidSettings.ki, yawPidSettings.kd, DIRECT);

// ===== Sensor State ===== //
float accelRollDeg{0.0f};
float accelPitchDeg{0.0f};
float gyroXRate{0.0f};
float gyroYRate{0.0f};
float gyroZRate{0.0f};

float rollDeg{0.0f};
float pitchDeg{0.0f};
float yawDeg{0.0f};

uint32_t controlLoopTimerUs{0};
uint32_t lastTelemetryMs{0};

// ===== Utility Functions ===== //
float getGyroScale() {
	switch (mpu.getFullScaleGyroRange()) {
		case 0: return 131.0f;   // ±250 °/s
		case 1: return 65.5f;    // ±500 °/s
		case 2: return 32.8f;    // ±1000 °/s
		case 3: return 16.4f;    // ±2000 °/s
		default: return 131.0f;
	}
}

float getAccelScale() {
	switch (mpu.getFullScaleAccelRange()) {
		case 0: return 16384.0f;  // ±2 g
		case 1: return 8192.0f;   // ±4 g
		case 2: return 4096.0f;   // ±8 g
		case 3: return 2048.0f;   // ±16 g
		default: return 16384.0f;
	}
}

void applyKalmanSettings(Kalman &filter, const KalmanSettings &settings) {
	filter.setQangle(settings.qAngle);
	filter.setQbias(settings.qBias);
	filter.setRmeasure(settings.rMeasure);
}

void applyPidSettings(PID &pid, const PIDSettings &settings) {
	pid.SetTunings(settings.kp, settings.ki, settings.kd);
	pid.SetOutputLimits(-settings.outputLimit, settings.outputLimit);
	pid.SetSampleTime(static_cast<int>(LOOP_PERIOD_US / 1000U));
}

float mapPulseToFloat(int pulseWidth, float minOutput, float maxOutput) {
	const int constrainedPulse = constrain(pulseWidth, RC_MIN_PULSE_US, RC_MAX_PULSE_US);
	const float scale = static_cast<float>(constrainedPulse - RC_MIN_PULSE_US) /
											static_cast<float>(RC_MAX_PULSE_US - RC_MIN_PULSE_US);
	return minOutput + scale * (maxOutput - minOutput);
}

int constrainServoOutput(int pulseWidth) {
	return constrain(pulseWidth, SERVO_MIN_US, SERVO_MAX_US);
}

void configureMPU() {
	Serial.println("Initializing I2C...");
	Wire.begin(SDA_PIN, SCL_PIN);
	Wire.setClock(400000UL);

	Serial.println("Initializing MPU6050...");
	mpu.initialize();
	delay(200);

	if (mpu.testConnection()) {
		Serial.println("MPU6050 connected.");
	} else {
		Serial.println("MPU6050 connection failed!");
	}

	Serial.println("Calibrating sensors... keep the device still.");
	delay(1000);
	mpu.CalibrateAccel(6);
	mpu.CalibrateGyro(6);
	Serial.println("Calibration complete.");

	mpu.setXAccelOffset(mpu.getXAccelOffset());
	mpu.setYAccelOffset(mpu.getYAccelOffset());
	mpu.setZAccelOffset(mpu.getZAccelOffset());
	mpu.setXGyroOffset(mpu.getXGyroOffset());
  	mpu.setYGyroOffset(mpu.getYGyroOffset());
  	mpu.setZGyroOffset(mpu.getZGyroOffset());
}

void primeKalmanFilters() {
	int16_t axRaw = 0, ayRaw = 0, azRaw = 0, gxRaw = 0, gyRaw = 0, gzRaw = 0;
	mpu.getMotion6(&axRaw, &ayRaw, &azRaw, &gxRaw, &gyRaw, &gzRaw);

	const float accelScale = getAccelScale();
	const float ax = static_cast<float>(axRaw) / accelScale;
	const float ay = static_cast<float>(ayRaw) / accelScale;
	const float az = static_cast<float>(azRaw) / accelScale;

	accelRollDeg = atan2f(ay, az) * RAD_TO_DEG;
	accelPitchDeg = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;

	rollDeg = accelRollDeg;
	pitchDeg = accelPitchDeg;
	yawDeg = 0.0f;

	kalmanRoll.setAngle(rollDeg);
	kalmanPitch.setAngle(pitchDeg);
}

void readIMU(float dtSeconds) {
	int16_t axRaw = 0, ayRaw = 0, azRaw = 0, gxRaw = 0, gyRaw = 0, gzRaw = 0;
	mpu.getMotion6(&axRaw, &ayRaw, &azRaw, &gxRaw, &gyRaw, &gzRaw);

	const float accelScale = getAccelScale();
	const float gyroScale = getGyroScale();

	const float ax = static_cast<float>(axRaw) / accelScale;
	const float ay = static_cast<float>(ayRaw) / accelScale;
	const float az = static_cast<float>(azRaw) / accelScale;

	gyroXRate = static_cast<float>(gxRaw) / gyroScale;
	gyroYRate = static_cast<float>(gyRaw) / gyroScale;
	gyroZRate = static_cast<float>(gzRaw) / gyroScale;

	accelRollDeg = atan2f(ay, az) * RAD_TO_DEG;
	accelPitchDeg = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;

	rollDeg = kalmanRoll.getAngle(accelRollDeg, gyroXRate, dtSeconds);
	pitchDeg = kalmanPitch.getAngle(accelPitchDeg, gyroYRate, dtSeconds);
	yawDeg += gyroZRate * dtSeconds;
}

void applyServoOutputs(int aileronRightUs, int aileronLeftUs, int elevatorUs, int rudderUs) {
	servoAileronRight.writeMicroseconds(constrainServoOutput(aileronRightUs));
	servoAileronLeft.writeMicroseconds(constrainServoOutput(aileronLeftUs));
	servoElevator.writeMicroseconds(constrainServoOutput(elevatorUs));
	servoRudder.writeMicroseconds(constrainServoOutput(rudderUs));
}

void printTelemetry(float dtSeconds) {
	/*
	// Print IMU and filter data
	Serial.printf(
		"dt:%.6f\taccR:%.2f\taccP:%.2f\tgx:%.3f\tgy:%.3f\troll:%.2f\tpitch:%.2f\tbiasR:%.5f\tbiasP:%.5f\n",
			dtSeconds, accelRollDeg, accelPitchDeg, gyroXRate, gyroYRate, rollDeg, pitchDeg,
			kalmanRoll.getQbias(), kalmanPitch.getQbias());
	*/

	
	// Print PID values
	Serial.printf(
		"RSP:%.2f\tRIN:%.2f\tROU:%.2f\tPSP:%.2f\tPIN:%.2f\tPOU:%.2f\tYSP:%.2f\tYIN:%.2f\tYOU:%.2f\n",
		rollSetpoint, rollInput, rollOutput,
		pitchSetpoint, pitchInput, pitchOutput,
		yawSetpoint, yawInput, yawOutput);
	
	/*
	// Print RC channel values
	Serial.printf(
		"CH1:%d\tCH2:%d\tCH4:%d\tCH6:%d\n",
		readChannel(RC_CH1), readChannel(RC_CH2),
		readChannel(RC_CH4), readChannel(RC_CH6));
	*/
}

}  // namespace

void setup() {
	Serial.begin(115200);
	delay(500);

	pinMode(CH1_PIN, INPUT);
	pinMode(CH2_PIN, INPUT);
	pinMode(CH4_PIN, INPUT);
	pinMode(CH6_PIN, INPUT);

	attachInterrupt(CH1_PIN, handleCh1Interrupt, CHANGE);
	attachInterrupt(CH2_PIN, handleCh2Interrupt, CHANGE);
	attachInterrupt(CH4_PIN, handleCh4Interrupt, CHANGE);
	attachInterrupt(CH6_PIN, handleCh6Interrupt, CHANGE);

	configureMPU();
	applyKalmanSettings(kalmanRoll, kalmanSettingsRoll);
	applyKalmanSettings(kalmanPitch, kalmanSettingsPitch);
	primeKalmanFilters();

	servoAileronRight.attach(SERVO_RIGHT_AILERON_PIN);
	servoAileronLeft.attach(SERVO_LEFT_AILERON_PIN);
	servoElevator.attach(SERVO_ELEVATOR_PIN);
	servoRudder.attach(SERVO_RUDDER_PIN);

	applyServoOutputs(SERVO_NEUTRAL_US, SERVO_NEUTRAL_US, SERVO_NEUTRAL_US, SERVO_NEUTRAL_US);

	applyPidSettings(rollPID, rollPidSettings);
	applyPidSettings(pitchPID, pitchPidSettings);
	applyPidSettings(yawPID, yawPidSettings);

	rollPID.SetMode(AUTOMATIC);
	pitchPID.SetMode(AUTOMATIC);
	yawPID.SetMode(AUTOMATIC);

	controlLoopTimerUs = micros();
	Serial.println("Setup complete.");
}

void loop() {
	const uint32_t loopStartUs = micros();
	const float dtSeconds = static_cast<float>(loopStartUs - controlLoopTimerUs) / 1000000.0f;
	controlLoopTimerUs = loopStartUs;

	readIMU(dtSeconds);

	const int ch1Pulse = readChannel(RC_CH1);
	const int ch2Pulse = readChannel(RC_CH2);
	const int ch4Pulse = readChannel(RC_CH4);
	const int ch6Pulse = readChannel(RC_CH6);

	const bool stabilizationEnabled = ch6Pulse >= SERVO_NEUTRAL_US;

	int aileronRightUs = ch1Pulse;
	int elevatorUs = ch2Pulse;
	int rudderUs = ch4Pulse;

	if (stabilizationEnabled) {
		rollSetpoint = mapPulseToFloat(ch1Pulse, -MAX_SETPOINT_ROLL_DEG, MAX_SETPOINT_ROLL_DEG);
		pitchSetpoint = mapPulseToFloat(ch2Pulse, -MAX_SETPOINT_PITCH_DEG, MAX_SETPOINT_PITCH_DEG);
		yawSetpoint = mapPulseToFloat(ch4Pulse, -MAX_SETPOINT_YAW_DEG, MAX_SETPOINT_YAW_DEG);

		rollInput = rollDeg;
		pitchInput = pitchDeg;
		yawInput = yawDeg;

		rollPID.Compute();
		pitchPID.Compute();
		yawPID.Compute();

		const int rollCorrectionUs = constrain(static_cast<int>(rollOutput * ROLL_US_PER_DEG), -SERVO_OUTPUT_RANGE_US, SERVO_OUTPUT_RANGE_US);
		const int pitchCorrectionUs = constrain(static_cast<int>(pitchOutput * PITCH_US_PER_DEG), -SERVO_OUTPUT_RANGE_US, SERVO_OUTPUT_RANGE_US);
		const int yawCorrectionUs = constrain(static_cast<int>(yawOutput * YAW_US_PER_DEG), -SERVO_OUTPUT_RANGE_US, SERVO_OUTPUT_RANGE_US);

		aileronRightUs = SERVO_NEUTRAL_US + rollCorrectionUs;
		elevatorUs = SERVO_NEUTRAL_US + pitchCorrectionUs;
		rudderUs = SERVO_NEUTRAL_US + yawCorrectionUs;
	}

	const int aileronLeftUs = INVERT_LEFT_AILERON
		                           ? SERVO_NEUTRAL_US - (aileronRightUs - SERVO_NEUTRAL_US)
		                           : aileronRightUs;

	applyServoOutputs(aileronRightUs, aileronLeftUs, elevatorUs, rudderUs);

	const uint32_t nowMs = millis();
	if (nowMs - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
		lastTelemetryMs = nowMs;
		printTelemetry(dtSeconds);
	}

	while ((micros() - loopStartUs) < LOOP_PERIOD_US) {
		// Wait for the next cycle while allowing interrupts to run.
	}
}