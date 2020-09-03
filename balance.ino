#include <Wire.h>

/* --- GLOBAL VARIABLES --- */
const int MPU_ADDR   = 0x68;
const int ACCEL_REGS = 0x3B;
const int GYRO_REGS  = 0x43;
const float pi       = 3.14159265;

short gyro_x, gyro_y, gyro_z;
short acc_x, acc_y, acc_z;
int gyro_x_offs, gyro_y_offs, gyro_z_offs, acc_x_offs, acc_y_offs, acc_z_offs;

float pitch_out, roll_out;

float roll_angle, pitch_angle, yaw_angle;
double start_of_loop;

/* --- FUNCTIONS VARIABLES --- */
void configure_MPU();
void calibrate_MPU(const int readings);
//void gather_gyro_data();
//void gather_accel_data();
void calculate_angles(bool configuration);
float square(float s) {return s*s;}
void print_pitch_roll();
void print_raw_MPU_data();

/*
const int ENABLE        = 4;
const int DIR           = 5;
const int STEP          = 6;
const int STEPS_PER_REV = 200;
*/

unsigned int print_inc = 0;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  
  configure_MPU();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.endTransmission();
  Wire.requestFrom(MPU_ADDR, 1);
  int setting = Wire.read();
  Serial.print("Gyro config register contents: ");
  Serial.println(setting);
  
  delay(500);                                          // delay 2 seconds for MPU to initialize
  
  calibrate_MPU(1000);
  /*
  pinMode(STEP,OUTPUT);
  pinMode(DIR,OUTPUT);
  pinMode(ENABLE,OUTPUT);
  digitalWrite(ENABLE,LOW);
 */
}

/* ---- LOOP ---- */

void loop() {
  start_of_loop = micros();
  
  calculate_angles(false);

//  gather_MPU_data();
  
  
  if (print_inc++ % 50 == 0 ) { print_pitch_roll(); }
//  print_raw_MPU_data();
  
  while (micros() - start_of_loop < 5000) { }                        // wait for next loop (200Hz)
  
}

/* ---- LOOP ---- */

void configure_MPU() {
  Wire.beginTransmission(MPU_ADDR);           
  Wire.write(0x6B);                       
  Wire.write(0x00);                       
  Wire.endTransmission(); 
  // setup the gyro 
  Wire.beginTransmission(MPU_ADDR);                                        
  Wire.write(0x1B);                                                   
  Wire.write(0b00001000);                                            // FS_SEL=1 (set bits 3,4 in gyro_config register to 0b01)      
  Wire.endTransmission();                 
  // setup the accel
  Wire.beginTransmission(MPU_ADDR);          
  Wire.write(0x1C);                                                 
  Wire.write(0b00010000);                                            // AFS_SEL=1 (set bits 3,4 in gyro_config register to 0b10)            
  Wire.endTransmission();                                               
}

void calibrate_MPU(const int readings) {
  /* This function records the MPU6050 gyroscope x,y,z values and gets 
   * their average over 3000 recordings.  Requires that the device be 
   * stationary during execution of the function
   */
   //TODO: add functionality that runs motors while calibrating for errant noise 

  if (readings <= 0) return;
  
  gyro_x_offs = gyro_y_offs = gyro_z_offs = 0;
  roll_angle = pitch_angle = yaw_angle    = 0;
  
  int dot_inc = readings / 21;
  
  Serial.print("Calibrating MPU6050...\n");
  
  for (int i = 0; i < readings; i++) {
    if (i % dot_inc == 0) { Serial.print("."); }
    gather_MPU_data();
    gyro_x_offs = gyro_x_offs + gyro_x;
    gyro_y_offs = gyro_y_offs + gyro_y;
    gyro_z_offs = gyro_z_offs + gyro_z;
    delay(3);
  }
  
  gyro_x_offs /= readings;
  gyro_y_offs /= readings;
  gyro_z_offs /= readings;

Serial.print("x avg: ");
Serial.print(gyro_x_offs);
Serial.print("\ty avg: ");
Serial.print(gyro_y_offs);
Serial.print("\tz avg: ");
Serial.println(gyro_z_offs);
  
  calculate_angles(true);
}

/*
void gather_gyro_data() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_REGS);
  Wire.endTransmission(true);
  Wire.requestFrom(MPU_ADDR, 6);
  while(Wire.available() < 6) { }                                   // wait for all bytes to be available to read
  gyro_x = (Wire.read() << 8 | Wire.read()) - gyro_x_offs;
  gyro_y = (Wire.read() << 8 | Wire.read()) - gyro_y_offs;
  gyro_z = (Wire.read() << 8 | Wire.read()) - gyro_z_offs;
  //while(Wire.available() > 0) { Wire.read(); }
}

void gather_accel_data() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_REGS);
  Wire.endTransmission(true);
  Wire.requestFrom(MPU_ADDR, 6);
  while(Wire.available() < 6) { }                                  // wait for all bytes to be available to read
  acc_x = Wire.read() << 8 | Wire.read();
  acc_y = Wire.read() << 8 | Wire.read();
  acc_z = Wire.read() << 8 | Wire.read();
  //while(Wire.available() > 0) { Wire.read(); }
}
*/

void gather_MPU_data() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 14);

  while (Wire.available() < 14);

  acc_x = Wire.read() << 8 | Wire.read();                                  //Add the low and high byte to the acc_x variable
  acc_y = Wire.read() << 8 | Wire.read();                                  //Add the low and high byte to the acc_y variable
  acc_z = Wire.read() << 8 | Wire.read();                                  //Add the low and high byte to the acc_z variable
  int temperature = Wire.read() << 8 | Wire.read();                            //Add the low and high byte to the temperature variable
  gyro_x = Wire.read() << 8 | Wire.read();                                 //Add the low and high byte to the gyro_x variable
  gyro_y = Wire.read() << 8 | Wire.read();                                 //Add the low and high byte to the gyro_y variable
  gyro_z = Wire.read() << 8 | Wire.read(); 
}

void calculate_angles(bool calibration) {
  /* Calculates the current angle of the device. 
   * calibration - true this function is called from calibrate_MPU(),
   *               otherwise this should be false
   */
  float drift_comp  = 0.0003;                                       // change this value to play around with drift settings 
  float filter      = 0.08;                                         // use a complementary filter
  
//  gather_gyro_data();
  gather_MPU_data();

  gyro_x -= gyro_x_offs;
  gyro_y -= gyro_y_offs;
  gyro_z -= gyro_z_offs;
  
  roll_angle   += ((float)gyro_x / 65.5) / 250.0;                            // 250Hz and raw gyro output is 65.5deg/s
  pitch_angle  += ((float)gyro_y / 65.5) / 250.0;
  yaw_angle    += ((float)gyro_z / 65.5) / 250.0;
//    roll_angle  += gyro_x * 0.0007634;
//    pitch_angle += gyro_y * 0.0007634;
//    yaw_angle   += gyro_z * 0.0007634;
  
  roll_angle  -= pitch_angle * sin(yaw_angle * (pi / 180.0));        // if there is any yaw movement, account for this by add to pitch and roll
  pitch_angle += roll_angle * sin(yaw_angle * (pi / 180.0));  
//  roll_angle  -= pitch_angle * sin(gyro_z * 0.00007634 * (pi / 180.0));        // if there is any yaw movement, account for this by add to pitch and roll
//  pitch_angle += roll_angle * sin(gyro_z * 0.00007634 * (pi / 180.0));  


//  gather_accel_data();

  float total_accel = sqrt((acc_x*acc_x) + (acc_y*acc_y) + (acc_z*acc_z));
  float accel_roll  = -1 * asin(float(acc_x)/total_accel) * (180.0 / pi);
  float accel_pitch = asin(float(acc_y)/total_accel) * (180.0 / pi);

  /*
  Serial.print("Roll (acc): ");
  Serial.print(accel_roll);
  Serial.print("\tPitch (acc): ");
  Serial.println(accel_pitch);
  */
  
  if (calibration) {
//    roll_out  = accel_roll;
//    pitch_out = accel_pitch;
    roll_angle  = accel_roll;
    pitch_angle = accel_pitch;
  }
  else {
//    roll_angle  = roll_angle * 0.9996 + accel_roll * 0.0004;
//    pitch_angle = pitch_angle * 0.9996 + accel_pitch * 0.0004;
  }
  // complementary filter
//  roll_out  = roll_out * 0.2 + roll_angle * 0.8;
//  pitch_out = pitch_out * 0.2 + pitch_angle * 0.8;
roll_out = roll_angle;
pitch_out = pitch_angle;
}

void print_pitch_roll() {
  Serial.print("Roll: ");
  Serial.print(roll_out);
  Serial.print("\tPitch: ");
  Serial.println(pitch_out);
}

void print_raw_MPU_data() {
  Serial.print("gyro_x: ");
  Serial.print(gyro_x);
  Serial.print("\tgyro_y: ");
  Serial.print(gyro_y);
  Serial.print("\tgyro_z: ");
  Serial.print(gyro_z);
  Serial.print("\tacc_x: ");
  Serial.print(acc_x);
  Serial.print("\tacc_y: ");
  Serial.print(acc_y);
  Serial.print("\tacc_z: ");
  Serial.println(acc_z);
}

  /*
  digitalWrite(DIR,HIGH);
  for (int i=0; i < STEPS_PER_REV*3; ++i) {
    digitalWrite(STEP,HIGH);
    delayMicroseconds(500);
    digitalWrite(STEP,LOW);
    delayMicroseconds(500);
  }

  delay(500);
  
  digitalWrite(DIR,LOW);
  for (int i=0; i < STEPS_PER_REV; ++i) {
    digitalWrite(STEP,HIGH);
    delayMicroseconds(2000);
    digitalWrite(STEP,LOW);
    delayMicroseconds(2000);
  }
  */
