#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 myIMU = Adafruit_BNO055(); 

// Variables

// Conversion
const float pi = 3.14159;
const float rad_to_deg = 360 / (2*pi);

// Global
float pitch_rate_gyro, roll_rate_gyro;
float pitch_acc, roll_acc;
float pitch_acc_f = 0, roll_acc_f = 0;
float pitch_gyro, roll_gyro;
float pitch = 0, roll = 0, yaw = 0;
float Xm = 0, Ym = 0;
float dt;
unsigned long t;

// Kalman Variables
float pk_roll[2][2] = {{0, 0}, {0, 0}};
float pk1_m_roll[2][2] = {{0, 0}, {0, 0}};
float pk1_roll[2][2] = {{0, 0}, {0, 0}};
float pk_pitch[2][2] = {{0, 0}, {0, 0}};
float pk1_m_pitch[2][2] = {{0, 0}, {0, 0}};
float pk1_pitch[2][2] = {{0, 0}, {0, 0}};
float xk_roll[2][1] = {{0}, {0}};
float xk1_m_roll[2][1] = {{0}, {0}};
float xk1_roll[2][1] = {{0}, {0}};
float xk_pitch[2][1] = {{0}, {0}};
float xk1_m_pitch[2][1] = {{0}, {0}};
float xk1_pitch[2][1] = {{0}, {0}};
float kg[2][1] = {{0}, {0}};
float A[2][2] = {{1, dt}, {0, 1}};
float B[2][1] = {{dt}, {0}};
float I[2][2] = {{1, 0}, {0, 1}};
float R = 0.003; // Try 0.03 or alter if need more accuracy
float Q[2][2] = {{.05*.05, 0}, {0, 0.001*0.001}}; // Try 0.0002, and 0.0001
float H[1][2] = {{1, 0}};
float S, uk_g_roll, uk_a_roll, uk_g_pitch, uk_a_pitch, nu_roll, nu_pitch;

void setup() {
  
  Serial.begin(115200);
  delay(1000);
  myIMU.begin();
  int8_t temp=myIMU.getTemp();
  myIMU.setExtCrystalUse(true);
  t = millis();
}

void kalman_roll() {

  uk_g_roll = roll_rate_gyro; // (deg/sec)
  uk_a_roll = roll; // (deg)
  

  // STEP 1: propagate the state and covariance from k to k+1
  // xk1_m = A*xk + B*uk_g; // xk1_m = state at k+1 minus drift calibration
                           // A = state transition model, xk = state at k
                           // B = control-input model, uk_g = control vector at time k from gyro
  xk1_m_roll[0][0] = A[0][0] * xk_roll[0][0] + A[0][1] * xk_roll[1][0] + B[0][0] * uk_g_roll;
  xk1_m_roll[1][0] = A[1][0] * xk_roll[0][0] + A[1][1] * xk_roll[1][0] + B[1][0] * uk_g_roll;

  // pk1_m = A*pk*AT + Q; // pk1_m = covariance at k+1 minus drift calibration
                           // pk = covariance at k, AT = A transpose
                           // Q = covariance of process noise
  pk1_m_roll[0][0] = (A[0][0] * pk1_roll[0][0] + A[0][1] * pk1_roll[1][0]) * A[0][0] + \
                (A[0][0] * pk1_roll[0][1] + A[0][1] * pk1_roll[1][1]) * A[0][1] + Q[0][0];
  pk1_m_roll[0][1] = (A[0][0] * pk1_roll[0][0] + A[0][1] * pk1_roll[1][0]) * A[1][0] + \
                (A[0][0] * pk1_roll[0][1] + A[0][1] * pk1_roll[1][1]) * A[1][1] + Q[0][1];
  pk1_m_roll[1][0] = (A[1][0] * pk1_roll[0][0] + A[1][1] * pk1_roll[1][0]) * A[0][0] + \
                (A[1][0] * pk1_roll[0][1] + A[1][1] * pk1_roll[1][1]) * A[0][1] + Q[1][0];
  pk1_m_roll[1][1] = (A[1][0] * pk1_roll[0][0] + A[1][1] * pk1_roll[1][0]) * A[1][0] + \
                (A[1][0] * pk1_roll[0][1] + A[1][1] * pk1_roll[1][1]) * A[1][1] + Q[1][1];

  // Correct for yaw movement (FIND FROM MAGNETOMETER USAGE FOR yaw)
//   xk1_m_roll[0][0] = xk1_m_roll[0][0] - xk_pitch[0][0] * sin(gyro_z*dt);

  // STEP 2: kalman gain calculation
  // S = H * pk1_m * HT + R // S = innavation (or pre-fit residual) covariance
                            // H = observation model which maps the true ss to the observed ss
                            // R = observation noise intensity (GWN)
  S = (H[0][0] * pk1_m_roll[0][0] + H[0][1] * pk1_m_roll[1][0]) * H[0][0] + \
      (H[0][0] * pk1_m_roll[0][1] + H[0][1] * pk1_m_roll[1][1]) * H[0][0] + R;

  // kg = pk1_m * HT * inv(S) // kg = kalman gain
  kg[0][0] = (pk1_m_roll[0][0] * H[0][0] + pk1_m_roll[0][1] * H[0][1]) / S;
  kg[1][0] = (pk1_m_roll[2][0] * H[0][0] + pk1_m_roll[1][1] * H[0][1]) / S;

  // STEP 3: state and covariance update calculations
  // xk1 = xk1_m + kg * (uk_a - H * xk1_m)  // xk1 = state update corrected for drift
                                            // uk_a = control vector at time k from accelorometer
  nu_roll = uk_a_roll - (H[0][0] * xk1_m_roll[0][0] + H[0][1] * xk1_m_roll[1][0]);
  xk1_roll[0][0] = xk1_m_roll[0][0] + kg[0][0] * nu_roll;
  xk1_roll[1][0] = xk1_m_roll[1][0] + kg[1][0] * nu_roll;
  // pk1 = (I - kk1 * H) * pk1_m  // pk1 = covariance update
  pk1_roll[0][0] = (1 - kg[0][0] * H[0][0]) * pk1_m_roll[0][0] + \
                   (0 - kg[0][0] * H[0][1]) * pk1_m_roll[1][0];
  pk1_roll[0][2] = (1 - kg[0][0] * H[0][0]) * pk1_m_roll[0][1] + \
                   (0 - kg[0][0] * H[0][1]) * pk1_m_roll[1][1];
  pk1_roll[1][0] = (1 - kg[1][0] * H[0][0]) * pk1_m_roll[0][0] + \
                   (0 - kg[1][0] * H[0][1]) * pk1_m_roll[1][0];
  pk1_roll[1][1] = (1 - kg[1][0] * H[0][0]) * pk1_m_roll[0][1] + \
                   (0 - kg[1][0] * H[0][1]) * pk1_m_roll[1][1];


  // Reset values for next iteration
  xk_roll[0][0] = xk1_roll[0][0];
  xk_roll[1][0] = xk1_roll[1][0];
  pk_roll[0][0] = pk1_roll[0][0];
  pk_roll[0][1] = pk1_roll[0][1];
  pk_roll[1][0] = pk1_roll[1][0];
  pk_roll[1][1] = pk1_roll[1][1];
  
}

void kalman_pitch() {
  
  uk_g_pitch = pitch_rate_gyro; // gyro pitch rate (deg/sec)
  uk_a_pitch = pitch; // (deg)
  

  // STEP 1: propagate the state and covariance from k to k+1
  // xk1_m = A*xk + B*uk_g; // xk1_m = state at k+1 minus drift calibration
                           // A = state transition model, xk = state at k
                           // B = control-input model, uk_g = control vector at time k from gyro
  xk1_m_pitch[0][0] = A[0][0] * xk_pitch[0][0] + A[0][1] * xk_pitch[1][0] + B[0][0] * uk_g_pitch;
  xk1_m_pitch[1][0] = A[1][0] * xk_pitch[0][0] + A[1][1] * xk_pitch[1][0] + B[1][0] * uk_g_pitch;

  // pk1_m = A*pk*AT + Q; // pk1_m = covariance at k+1 minus drift calibration
                           // pk = covariance at k, AT = A transpose
                           // Q = covariance of process noise
  pk1_m_pitch[0][0] = (A[0][0] * pk1_pitch[0][0] + A[0][1] * pk1_pitch[1][0]) * A[0][0] + \
                (A[0][0] * pk1_pitch[0][1] + A[0][1] * pk1_pitch[1][1]) * A[0][1] + Q[0][0];
  pk1_m_pitch[0][1] = (A[0][0] * pk1_pitch[0][0] + A[0][1] * pk1_pitch[1][0]) * A[1][0] + \
                (A[0][0] * pk1_pitch[0][1] + A[0][1] * pk1_pitch[1][1]) * A[1][1] + Q[0][1];
  pk1_m_pitch[1][0] = (A[1][0] * pk1_pitch[0][0] + A[1][1] * pk1_pitch[1][0]) * A[0][0] + \
                (A[1][0] * pk1_pitch[0][1] + A[1][1] * pk1_pitch[1][1]) * A[0][1] + Q[1][0];
  pk1_m_pitch[1][1] = (A[1][0] * pk1_pitch[0][0] + A[1][1] * pk1_pitch[1][0]) * A[1][0] + \
                (A[1][0] * pk1_pitch[0][1] + A[1][1] * pk1_pitch[1][1]) * A[1][1] + Q[1][1];

  // Correct for yaw movement (FIND FROM MAGNETOMETER USAGE FOR yaw)
//   xk1_m_pitch[0][0] = xk1_m_pitch[0][0] - xk_pitch[0][0] * sin(gyro_z*dt);

  // STEP 2: kalman gain calculation
  // S = H * pk1_m * HT + R // S = innavation (or pre-fit residual) covariance
                            // H = observation model which maps the true ss to the observed ss
                            // R = observation noise intensity (GWN)
  S = (H[0][0] * pk1_m_pitch[0][0] + H[0][1] * pk1_m_pitch[1][0]) * H[0][0] + \
      (H[0][0] * pk1_m_pitch[0][1] + H[0][1] * pk1_m_pitch[1][1]) * H[0][0] + R;

  // kg = pk1_m * HT * inv(S) // kg = kalman gain
  kg[0][0] = (pk1_m_pitch[0][0] * H[0][0] + pk1_m_pitch[0][1] * H[0][1]) / S;
  kg[1][0] = (pk1_m_pitch[2][0] * H[0][0] + pk1_m_pitch[1][1] * H[0][1]) / S;

  // STEP 3: state and covariance update calculations
  // xk1 = xk1_m + kg * (uk_a - H * xk1_m)  // xk1 = state update corrected for drift
                                            // uk_a = control vector at time k from accelorometer
  nu_pitch = uk_a_pitch - (H[0][0] * xk1_m_pitch[0][0] + H[0][1] * xk1_m_pitch[1][0]);
  xk1_pitch[0][0] = xk1_m_pitch[0][0] + kg[0][0] * nu_pitch;
  xk1_pitch[1][0] = xk1_m_pitch[1][0] + kg[1][0] * nu_pitch;
  // pk1 = (I - kk1 * H) * pk1_m  // pk1 = covariance update
  pk1_pitch[0][0] = (1 - kg[0][0] * H[0][0]) * pk1_m_pitch[0][0] + \
                   (0 - kg[0][0] * H[0][1]) * pk1_m_pitch[1][0];
  pk1_pitch[0][2] = (1 - kg[0][0] * H[0][0]) * pk1_m_pitch[0][1] + \
                   (0 - kg[0][0] * H[0][1]) * pk1_m_pitch[1][1];
  pk1_pitch[1][0] = (1 - kg[1][0] * H[0][0]) * pk1_m_pitch[0][0] + \
                   (0 - kg[1][0] * H[0][1]) * pk1_m_pitch[1][0];
  pk1_pitch[1][1] = (1 - kg[1][0] * H[0][0]) * pk1_m_pitch[0][1] + \
                   (0 - kg[1][0] * H[0][1]) * pk1_m_pitch[1][1];


  // Reset values for next iteration
  // anglepitchk
//  t = t + dt;
  xk_pitch[0][0] = xk1_pitch[0][0];
  xk_pitch[1][0] = xk1_pitch[1][0];
  pk_pitch[0][0] = pk1_pitch[0][0];
  pk_pitch[0][1] = pk1_pitch[0][1];
  pk_pitch[1][0] = pk1_pitch[1][0];
  pk_pitch[1][1] = pk1_pitch[1][1];
  
}

void loop() {
//  uint8_t system_calibration, gyro_calibration acc_calibration, mag_calibration = 0;
//  myIMU.getCalibration(&system_calibration, &gyro_calibration &acc_calibration, &mag_calibration);
  imu::Vector<3> acc = myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyro = myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> mag = myIMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  // input as 
  pitch_acc = atan2(acc.x(),acc.z()) * rad_to_deg;
  roll_acc = atan2(acc.y(),acc.z()) * rad_to_deg;

  // High pass filter on acc to filter noise and vibration in acc
  pitch_acc_f = .95 * pitch_acc_f + .05 * pitch_acc;
  roll_acc_f = .95 * roll_acc_f + .05 * roll_acc;

  dt = (millis() - t) / 1000;
  t = millis();

  // input as rad/second
  pitch_rate_gyro = gyro.y() * rad_to_deg;
  pitch_gyro = pitch_gyro + pitch_rate_gyro * dt;
  roll_rate_gyro = gyro.x() * rad_to_deg;
  roll_gyro = roll_gyro + roll_rate_gyro * dt;

  // Complementary filter to filter drift from gyro and 
  // noise from accelerometer
  pitch = (pitch + pitch_gyro * dt) * 0.99 + pitch_acc_f * 0.01;
  roll = (roll + roll_gyro * dt) * 0.99 + roll_acc_f * 0.01;

  // Yaw from magnetometer adjusted for tilt
  Xm = mag.x() * cos(pitch/rad_to_deg) + mag.y() * sin(roll/rad_to_deg) * \
       sin(pitch/rad_to_deg) + mag.z() * cos(roll/rad_to_deg) * sin(pitch/rad_to_deg);
  Ym = mag.y() * cos(roll/rad_to_deg) + mag.z() * sin(roll/rad_to_deg);
  
  yaw = atan2(Ym,Xm) * rad_to_deg;

  kalman_roll();
  kalman_pitch();

  Serial.println(String(xk_pitch[0][0]) + ", " + String(xk_roll[0][0]) + ", " + String(yaw));
}
