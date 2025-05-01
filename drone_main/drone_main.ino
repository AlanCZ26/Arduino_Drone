/*
   Arduino and MPU6050 Accelerometer and Gyroscope Sensor Tutorial
   by Dejan, https://howtomechatronics.com
*/
#include <Wire.h>
const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
//float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw, gforce;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
unsigned long currentTime, previousTime;
float deltaT;

const byte pins[6] = {2, 4, 7, 8, 12, 13};
short pwms[6] = {-1, -1, -1, -1, -1, -1};
//unsigned pulseLength; // = pulseIn(9, LOW, 55000UL) + pulseIn(9, HIGH, 55000UL);
unsigned long pwmsTimeCounter[6] = {};
bool pwmsTimeTracker[6] = {};
double averageCycleTime = 0;
byte pulseSkipChecker = 100;

byte printer_counter = 0;

float err_x, err_y, err_z, err_ax, err_ay;

#include <Servo.h>
Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;
Servo* esc_ary[] = {&esc1, &esc2, &esc3, &esc4};

void setup() 
{
  pinMode(LED_BUILTIN, OUTPUT);
  for (const byte& i : pins) pinMode(i, INPUT); // Controller in pins
  Serial.begin(19200);
  Serial.println(" === Connecting... ===");
  Wire.begin();                      // Initialize comunication
  Serial.println(" === Transmitting... ===");
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Serial.println(" === Writing... ===");
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Serial.println(" === Ending transmission... ===");
  digitalWrite(LED_BUILTIN, HIGH);
  Wire.endTransmission(true);        //end the transmission
  digitalWrite(LED_BUILTIN, LOW);
  /*
  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x10);                  //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                   // Set the register bits as 00010000 (1000deg/s full scale)
  Wire.endTransmission(true);
  delay(20);
  */
  Serial.println(" === Calibrating... ===");
  calculate_IMU_error(500, err_x, err_y, err_z, err_ax, err_ay);
  Serial.println(" === Calibrated, Waiting for RX ===");
  delay(20);
  esc1.attach(3);
  esc2.attach(5);
  esc3.attach(6);
  esc4.attach(9);
}

unsigned long cycleTimeTracker;
void loop() 
{
  cycleTimeTracker = micros();
  readMPU();
  getPWMfaster();

  int k = (pwms[2]);
  if (k < 1100) k = 1000;
  if (k > 2000) k = 2000;
  for (const auto& esc : esc_ary) {
    esc->writeMicroseconds(k);
  }

  if (++printer_counter > 10) { // Print the values on the serial monitor
    printer_counter = 0;
    for (const byte& PWM : pwms) {
      Serial.print(PWM);
      Serial.print(" | ");
    }
    Serial.print("psk: ");
    Serial.print(pulseSkipChecker);
    Serial.print(" act: ");
    Serial.println(averageCycleTime / 1000.0); 
    
    Serial.print(roll);
    Serial.print("/");
    Serial.print(pitch);
    Serial.print("/");
    Serial.print(yaw);
    Serial.print("- ");
    Serial.println(gforce);
    
  
    Serial.print("sent: ");
    Serial.println(k);
  }
  averageCycleTime = (averageCycleTime * 0.99) + ((micros() - cycleTimeTracker) * 0.01);
  
}



float filter = 1;  //0.96; Complementary filter
float filter2 = 0.99;  //0.99; Stability filter
void readMPU() {
  // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 57.2957795131) - err_ax; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 57.2957795131) - err_ay; // 180/PI = 57.2957795131
  // === Read gyroscope data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  deltaT = (currentTime - previousTime) / 1000.0; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  gyroAngleX += (((Wire.read() << 8 | Wire.read()) / 131.0) - err_x) * deltaT; // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by seconds (s) to get the angle in degrees
  gyroAngleY += (((Wire.read() << 8 | Wire.read()) / 131.0) - err_y) * deltaT; // deg/s * s = deg  
  yaw += (((Wire.read() << 8 | Wire.read()) / 131.0) - err_z) * deltaT;  
  gyroAngleX = (gyroAngleX * filter2) + (accAngleX * (1-filter2)); //Stability filter -- the value drifts without it
  gyroAngleY = (gyroAngleY * filter2) + (accAngleY * (1-filter2));
  roll = (filter * gyroAngleX) + ((1-filter) * accAngleX);// Complementary filter - combine acceleromter and gyro angle values
  pitch = (filter * gyroAngleY) + ((1-filter) * accAngleY);
  gforce = sqrt(AccX*AccX + AccY*AccY + AccZ*AccZ) * 100;
}

void getPWMfaster() {
  bool current;
  unsigned long t = micros();
  bool c[6] = {false, false, false, false, false, false};
  while (!(c[0]&&c[1]&&c[2]&&c[3]&&c[4]&&c[5])) { // until every pin has return positive
    for (byte i = 0; i < 6; i++) {
      current = (digitalRead(pins[i]) == 1);
      t = micros();

      if (current && !pwmsTimeTracker[i]) { // rising edge
        pulseSkipChecker++;
        //pulseLength = (t - pwmsTimeCounter[i]) / 1000; // Pulse length in ms
        pwmsTimeCounter[i] = t;// Note rising edge timing
      }
      else if (!current && pwmsTimeTracker[i]) { // falling edge
        pulseSkipChecker--;
        if ((t - pwmsTimeCounter[i]) < 2200) { // the value should never be over 2000 so if it is the latest rising edge was missed
          pwms[i] = ((pwms[i] * 0.9) + ((t - pwmsTimeCounter[i]) * 0.1)); // Measured uptime, filtered with previous for stability
          c[i] = true;
        }
      }
      pwmsTimeTracker[i] = current; // Set the previous value
    } 
  }
}

void calculate_IMU_error(int cycles, float& ierr_x, float& ierr_y, float& ierr_z, float& ierr_ax, float& ierr_ay) {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  for(int i = 0; i < cycles; i++) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / cycles;
  AccErrorY = AccErrorY / cycles;
  ierr_ax = AccErrorX;
  ierr_ay = AccErrorY;
  // Read gyro values 200 times
  for(int i = 0; i < cycles; i++) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    // GyroX = Wire.read() << 8 | Wire.read();
    // GyroY = Wire.read() << 8 | Wire.read();
    // GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + ((Wire.read() << 8 | Wire.read()) / 131.0);
    GyroErrorY = GyroErrorY + ((Wire.read() << 8 | Wire.read()) / 131.0);
    GyroErrorZ = GyroErrorZ + ((Wire.read() << 8 | Wire.read()) / 131.0);
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / cycles;
  GyroErrorY = GyroErrorY / cycles;
  GyroErrorZ = GyroErrorZ / cycles;
  ierr_x = GyroErrorX;
  ierr_y = GyroErrorY;
  ierr_z = GyroErrorZ;
  // Print the error values on the Serial Monitor
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
}