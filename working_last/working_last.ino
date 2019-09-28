//ESP32 PIN          PERIPHERAL PIN
//                   MPU9250
//3.3V           |   3v3
//GND            |   GND,ADO
//D21            |   SDA
//D22            |   SCL
//                   Parallax Feedback 360°(YAW SERVO)
//D32            |   POSITION FEEDBACK
//VIN            |   RED(5V)
//GND            |   BLACK(GND)
//D33            |   WHITE(SIGNAL)
//                   PowerHD HD-3001HB(PITCH SERVO)
//VIN            |   RED(5V)
//GND            |   BROWN(GND)
//D25            |   ORANGE(SIGNAL)

#include <ESP32Servo.h>
#include "MPU9250.h"
#include "WiFi.h"

MPU9250 IMU(Wire,0x68);//MPU-9250 sensor on I2C bus 0 with address 0x68

Servo servo_pitch;
Servo servo_yaw;

int status;

float elapsedTime, times, timePrev;//Variables for time control
float rad_to_deg = 180.0f / PI;

//Magnetometer Calibration Variables
float axb=0.0;
float axs=1.0;
float ayb=0.0;
float ays=1.0;
float azb=0.0;
float azs=1.0;
float xv, yv, zv;
float calibrated_values[3];
float scaler;
boolean scaler_flag = false;
float normal_vector_length;
double calibration_matrix[3][3] = 
  {
    {1.267, 0.003, -0.028},
    {0.008, 1.304, -0.013},
    {-0.049, 0.055, 1.265}  
  };
double bias[3] = 
  {
    25.775,
    -1.54,
    -8.511
  };

//Madgwick Filter Variables
float beta = 0.6f;
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};//Vector to hold quaternion
float pitch, yaw, roll;

//Variables for Parallax Feedback 360° Servo
int feedback_pin=32;
float feedback_angle = 0; //Measured angle from feedback
float parallax_kp = 0.5; //Proportional Gain, higher values for faster response, higher values contribute to overshoot.
float parallax_ki = 0.005; //Integral Gain, higher values to converge faster to zero error, higher values produce oscillations. Higher values are more unstable near a target_angle = 0. 
float parallax_kd = 0.012; //Derivative Gain, higher values dampen oscillations around target_angle. Higher values produce more holding state jitter. May need filter for error noise.
float parallax_PID, parallax_output, parallax_previous_error, parallax_error;
float parallax_pid_p, parallax_pid_i, parallax_pid_d;

//Gyro Variables
float Gyr_rawX, Gyr_rawY, Gyr_rawZ;

//Acc Variables
float Acc_rawX, Acc_rawY, Acc_rawZ;

//Mag Variables
float Mag_rawX, Mag_rawY, Mag_rawZ;

//PID for Pitch
float pitch_PID, pitch_error, pitch_previous_error;
float pitch_pid_p=0;
float pitch_pid_i=0;
float pitch_pid_d=0;

//Pitch PID Constants
float pitch_kp=1;
float pitch_ki=0;
float pitch_kd=0;
float pitch_desired_angle;

//PID for Yaw
float yaw_PID, yaw_error, yaw_previous_error;
float yaw_pid_p=0;
float yaw_pid_i=0;
float yaw_pid_d=0;

//Yaw PID Constants
float yaw_kp=1;
float yaw_ki=0;
float yaw_kd=0;
float yaw_desired_angle;

float PWM_pitch,PWM_yaw;

void setup() 
{
  //Turn off BT and WiFi to reduce interference on magnetometer
  WiFi.mode(WIFI_OFF);
  btStop();
	
  status = IMU.begin();// start communication with IMU 
  
  servo_pitch.attach(25);
  servo_yaw.attach(33);
  
  pinMode(feedback_pin,INPUT);
  
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_2G);
  IMU.setGyroRange(MPU9250::GYRO_RANGE_250DPS);
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_184HZ);
  IMU.setSrd(0);
  IMU.setAccelCalX(axb, axs);
  IMU.setAccelCalY(ayb, ays);
  IMU.setAccelCalZ(azb, azs);
}
//END OF VOID SETUP

void loop() 
{
  float values_from_magnetometer[3];

  if (status < 0) 
  {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
    
  IMU.readSensor();//Read the sensor
  
  //Accelerometer read
  Acc_rawX=IMU.getAccelX_mss();
  Acc_rawY=IMU.getAccelY_mss();
  Acc_rawZ=IMU.getAccelZ_mss();
  
  //Gyroscope read
  Gyr_rawX=IMU.getGyroX_rads();
  Gyr_rawY=IMU.getGyroY_rads();
  Gyr_rawZ=IMU.getGyroZ_rads();
  
  //Magnetometer read
  Mag_rawX=IMU.getMagX_uT();
  Mag_rawY=IMU.getMagY_uT();
  Mag_rawZ=IMU.getMagZ_uT();
  //Hard and soft iron calibration
  values_from_magnetometer[0] = Mag_rawX;
  values_from_magnetometer[1] = Mag_rawY;
  values_from_magnetometer[2] = Mag_rawZ;
  transformation(values_from_magnetometer);
  vector_length_stabilasation();

  timePrev = times;
  times = millis();
  elapsedTime = (times - timePrev) / 1000;//We work in ms so divide to 1000    
  
  MadgwickQuaternionUpdate(Gyr_rawX, Gyr_rawY, Gyr_rawZ, Acc_rawX, Acc_rawY, Acc_rawZ, calibrated_values[0], calibrated_values[1], calibrated_values[2]);
  
  // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
  // In this coordinate system, the positive z-axis is down toward Earth. 
  // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
  // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
  // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
  // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
  // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
  // applied in the correct order which for this configuration is yaw, pitch, and then roll.
  // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles for more information.
  yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
  pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
  roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
  pitch *= rad_to_deg;
  yaw   *= rad_to_deg; 
  yaw   -= 5.8;//Corrected for local declination. https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#declination
  if(yaw<0)
  {
    yaw+=360;
  }

//PITCH PID   

  pitch_desired_angle = 0;//The angle we want the turret to stay is 0 for both axis
  pitch_error = pitch_desired_angle - pitch;
  pitch_pid_p = pitch_kp * pitch_error;
  pitch_pid_i = pitch_pid_i + (pitch_error * elapsedTime);  
  pitch_pid_d = (pitch_error - pitch_previous_error)/elapsedTime;
  pitch_PID = (pitch_pid_p) + (pitch_ki * pitch_pid_i) + (pitch_kd * pitch_pid_d);
  pitch_previous_error = pitch_error;
  if(pitch_PID < -60)
	{
	 pitch_PID = -60;
	}
  if(pitch_PID > 60) 
	{
	 pitch_PID = 60;
	}
  PWM_pitch = 90-pitch_PID;
  servo_pitch.write(PWM_pitch);     
   
//YAW PID

  yaw_desired_angle = 0;
  if (yaw_desired_angle < 0)
	{
		yaw_desired_angle += 360;//Handles negative angles
	}
  yaw_error = yaw_desired_angle - yaw;
  yaw_pid_p = yaw_kp * yaw_error;
  yaw_pid_i = yaw_pid_i + (yaw_error * elapsedTime);
  yaw_pid_d = (yaw_error - yaw_previous_error)/elapsedTime;
  yaw_PID = (yaw_pid_p) + (yaw_ki * yaw_pid_i) + (yaw_kd * yaw_pid_d);
  yaw_previous_error = yaw_error;
  
  //PARALLAX POS. FEEDBACK AND ANGLE CONTROL PID
  feedback360();
  if (yaw_PID<0)
  {
    yaw_PID+=360;  
  }
  parallax_error = yaw_PID - feedback_angle;
  if(parallax_error > 180)
  {
    parallax_error = parallax_error - 360;//Rotate in the other direction because it is a smaller angle that way.
  }
  if (parallax_error < -180)
  {
    parallax_error = 360 - parallax_error - 360;
  }
  parallax_pid_p = parallax_error * parallax_kp;
  parallax_pid_i = parallax_pid_i + (parallax_error * elapsedTime);
  if  (parallax_pid_i > 1)
  {
    parallax_pid_i = 1;
  }
  if (parallax_pid_i <  -1)
  {
    parallax_pid_i = -1;
  }
  parallax_pid_d = (parallax_error - parallax_previous_error)/elapsedTime;
  parallax_PID = (parallax_pid_p) + (parallax_ki * parallax_pid_i) + (parallax_kd * parallax_pid_d);
  parallax_previous_error = parallax_error;
  if(parallax_PID > 30)
  {
    parallax_PID = 30;// Clamp output
  }          
  if(parallax_PID < -30)
  {
    parallax_PID = -30;
  }
  servo_yaw.write(93 + parallax_PID);
}
//END OF VOID LOOP

//USED FUNCTIONS ARE DEFINED HERE
void MadgwickQuaternionUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   //short name local variable for readability
  float norm;
  float hx, hy, _2bx, _2bz;
  float s1, s2, s3, s4;
  float qDot1, qDot2, qDot3, qDot4;
  //Auxiliary variables to avoid repeated arithmetic
  float _2q1mx;
  float _2q1my;
  float _2q1mz;
  float _2q2mx;
  float _4bx;
  float _4bz;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;
  float _2q1q3 = 2.0f * q1 * q3;
  float _2q3q4 = 2.0f * q3 * q4;
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;
  //Normalise accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f/norm;
  ax *= norm;
  ay *= norm;
  az *= norm;
  //Normalise magnetometer measurement
  norm = sqrt(mx * mx + my * my + mz * mz);
  if (norm == 0.0f) return; //handle NaN
  norm = 1.0f/norm;
  mx *= norm;
  my *= norm;
  mz *= norm;
  //Reference direction of Earth's magnetic field
  _2q1mx = 2.0f * q1 * mx;
  _2q1my = 2.0f * q1 * my;
  _2q1mz = 2.0f * q1 * mz;
  _2q2mx = 2.0f * q2 * mx;
  hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
  hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
  _2bx = sqrt(hx * hx + hy * hy);
  _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
  _4bx = 2.0f * _2bx;
  _4bz = 2.0f * _2bz;
  //Gradient decent algorithm corrective step
  s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    //normalise step magnitude
  norm = 1.0f/norm;
  s1 *= norm;
  s2 *= norm;
  s3 *= norm;
  s4 *= norm;
  //Compute rate of change of quaternion
  qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
  qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
  qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
  qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;
  //Integrate to yield quaternion
  q1 += qDot1 * elapsedTime;
  q2 += qDot2 * elapsedTime;
  q3 += qDot3 * elapsedTime;
  q4 += qDot4 * elapsedTime;
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    //normalise quaternion
  norm = 1.0f/norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}
	
void transformation(float uncalibrated_values[3])    
{
  //calculation
  for (int i=0; i<3; ++i) uncalibrated_values[i] = uncalibrated_values[i] - bias[i];
  float result[3] = {0, 0, 0};
  for (int i=0; i<3; ++i)
    for (int j=0; j<3; ++j)
      result[i] += calibration_matrix[i][j] * uncalibrated_values[j];
  for (int i=0; i<3; ++i) calibrated_values[i] = result[i];
}

void vector_length_stabilasation()
{
  //calculate the normal vector length
  if (scaler_flag == false)
  {
    IMU.readSensor();
    xv=IMU.getMagX_uT();
    yv=IMU.getMagY_uT();
    zv=IMU.getMagZ_uT();
    normal_vector_length = sqrt(calibrated_values[0]*calibrated_values[0] + calibrated_values[1]*calibrated_values[1] + calibrated_values[2]*calibrated_values[2]);
    scaler_flag = true;
  } 
  //calculate the current scaler
  scaler = normal_vector_length/sqrt(calibrated_values[0]*calibrated_values[0] + calibrated_values[1]*calibrated_values[1] + calibrated_values[2]*calibrated_values[2]);
  //apply the current scaler to the calibrated coordinates (global array calibrated_values)
  calibrated_values[0] = calibrated_values[0]*scaler;
  calibrated_values[1] = calibrated_values[1]*scaler;
  calibrated_values[2] = calibrated_values[2]*scaler;
}

float feedback360()
{
  int unitsFC = 360;                          // Units in a full circle
  int dutyScale = 1000;                       // Scale duty cycle to 1/1000ths
  int dcMin = 29;                             // Minimum duty cycle
  int dcMax = 971;                            // Maximum duty cycle
  int q2min = unitsFC/4;                      // For checking if in 1st quadrant
  int q3max = q2min * 3;                      // For checking if in 4th quadrant
  int turns = 0;                              // For tracking turns
  // dc is duty cycle, theta is 0 to 359 angle, thetaP is theta from previous
  // loop repetition, tHigh and tLow are the high and low signal times for 
  // duty cycle calculations.
  int dc, theta, thetaP, tHigh, tLow;         

  // Measure feedback signal high/low times.
  tLow = pulseIn(feedback_pin, LOW);            // Measure low time 
  tHigh = pulseIn(feedback_pin, HIGH);           // Measure high time

  // Calcualte initial duty cycle and angle.
  dc = (dutyScale * tHigh) / (tHigh + tLow);
  theta = (unitsFC - 1) - ((dc - dcMin) * unitsFC) / (dcMax - dcMin + 1);
  thetaP = theta;

  while(1)                                    // Main loop for this cog
  {
    // Measure high and low times, making sure to only take valid cycle
    // times (a high and a low on opposite sides of the 0/359 boundary
    // will not be valid.
    int tCycle = 0;                           // Clear cycle time
    while(1)                                  // Keep checking
    {
      tHigh = pulseIn(feedback_pin, HIGH);       // Measure time high
      tLow = pulseIn(feedback_pin, LOW);        // Measure time low
      tCycle = tHigh + tLow;
      if((tCycle > 1000) && (tCycle < 1200))  // If cycle time valid 
        break;                                // break from loop
    }      
    dc = (dutyScale * tHigh) / tCycle;        // Calculate duty cycle
    
    // This gives a theta increasing int the
    // counterclockwise direction.
    theta = (unitsFC - 1) - ((dc - dcMin) * unitsFC) / (dcMax - dcMin + 1); // Calculate angle

    if(theta < 0)                             // Keep theta valid   
      theta = 0; 
    else if(theta > (unitsFC - 1)) 
      theta = unitsFC - 1;

    // If transition from quadrant 4 to  
    // quadrant 1, increase turns count. 
    if((theta < q2min) && (thetaP > q3max))
      turns++;
    // If transition from quadrant 1 to  
    // quadrant 4, decrease turns count. 
    else if((thetaP < q2min) && (theta > q3max))
      turns --;

    // Construct the angle measurement from the turns count and
    // current theta value.
    if(turns >= 0)
      feedback_angle = (turns * unitsFC) + theta;
    else if(turns <  0)
      feedback_angle = ((turns + 1) * unitsFC) - (unitsFC - theta);

    thetaP = theta;                           // Theta previous for next rep

    if (feedback_angle < 0)
    {
      feedback_angle+=360;
    }
    // Construct the angle measurement from the turns count and
    // current theta value.
  
    return feedback_angle;
  }
}