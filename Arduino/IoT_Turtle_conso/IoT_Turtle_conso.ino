
/* IOT Turtle / Accelerometer, magnetometer, depth, GPS
   CMWX1ZZABZ (STM32L082 and SX1276)
   LSM303AGR
   MX25R6435FZAI 8 MByte SPI NOR flash memory

   Pierre Gogendeau (IOT)

   Code for testing function timing

*/

/********************** TEST CONFIGURATION ************************************/

volatile bool F1 = false;
volatile bool F2 = false;
volatile bool F3 = false;
volatile bool F4 = false;

const int len_buffer = 3;
/*if(F1 == 1){
  const int len_buffer = 10;
  } else {
  const int len_buffer = 3;
  }*/


unsigned long t_start;
unsigned long t_end;

unsigned long t_filter = 0;
unsigned long t_VeDBA = 0;
unsigned long t_jerk = 0;
unsigned long t_reg = 0;
unsigned long t_hp = 0;
unsigned long t_ocdr = 0;
unsigned long t_sF2 = 0;
unsigned long t_sF3 = 0;
unsigned long t_sF4 = 0;
unsigned long t_o = 0;
unsigned long t_traj = 0;
unsigned long t_depth = 0;


/*********************************************************************************/

#include <STM32L0.h>
#include <RTC.h>
#include "LoRaWAN.h"
#include "TimerMillis.h"
#include "GNSS.h"
#include "CayenneLPP.h"
#include "LSM303AGR_Acc.h"
#include "LSM303AGR_Mag.h"
#include "SPIFlash.h"
#include "I2CDev.h"
#include <CircularBuffer.h>
#include "MS5837.h"


// Buffer to filter data

volatile bool serial_print = true;
volatile bool serial_debug = false;
volatile bool enable_gps_live = true;
volatile bool led_sampling = false;
// Sensor activation
volatile bool use_GPS  = false ;

// Size payload
const int payload = 150;
// Length max dive in second
const int len = 100; //sizeof(X1);
uint16_t time_out_GPS = 30000;  // ms (30sec)
const int nb_pos_out_max = 15;
const int nb_eth_out_max = 10;
int nb_pos_out = 0;
int nb_eth_out = 0;
long time_depth_offset = 600000; // 10min in millisecond

/*********************************************************************************/

#define PI 3.1415926
#define deg2rad(angle) (angle * PI / 180.0)
#define rad2deg(angle) (angle * 180.0 / PI)


#define I2C_BUS    Wire               // Define the I2C bus (Wire instance) you wish to use
I2Cdev             i2c_0(&I2C_BUS);   // Instantiate the I2Cdev object and point to the desired I2C bus

const char        *build_date = __DATE__;   // 11 characters MMM DD YYYY
const char        *build_time = __TIME__;   // 8 characters HH:MM:SS

// pin assignments
#define myLed     10 // blue led 
#define myVBat_en  2 // enable VBat read
#define myVBat    A1 // VBat analog read pin

uint32_t UID[3] = {0, 0, 0};
char buffer[32];

bool SerialDebug = true;

uint8_t hours[4] = {12, 12, 12, 12}, minutes[4] = {0, 0, 0, 0}, seconds[4] = {0, 0, 0, 0}, year = 1, month = 1, day = 1;
uint32_t subSeconds[4], milliseconds;

// battery voltage monitor definitions
float VDDA, VBAT, VBUS, STM32L0Temp, battery_level;

//LSM303AGR_Acc definitions
#define LSM303AGR_Acc_intPin1 A4  // interrupt1 pin definitions, data ready
#define LSM303AGR_Acc_intPin2 3   // interrupt2 pin definitions, significant motion

union {           // offset biases
  float fval;
  byte bval[4];
} accelBias[3];


float aRes;
float param_accelBias[3] = {0.0f, 0.0f, 0.0f}; // offset biases copy for fonction call
int16_t accelData[3], accTempData;  // Stores the 10-bit signed accel output if normal mode
float ax_f, ay_f, az_f;  int16_t ax, ay, az;                   // variables to hold latest accel data values
uint8_t Ascale = AFS_2G, AODR_rec;
uint8_t AODR = 0x02;
volatile bool newLSM303AGR_AccData_1 = false; // used for data ready interrupt handling
volatile bool newLSM303AGR_AccData_2 = false; // used for motion ready interrupt handling
bool newAcc = false;
LSM303AGR_Acc LSM303AGR_Acc(&i2c_0); // instantiate LSM303AGR_Acc class

//LSM303AGR_Mag definitions
#define LSM303AGR_Mag_intPin  A2 // interrupt for magnetometer data ready
int16_t magData[3];
uint8_t MODR_rec; // = MODR_10Hz;
uint8_t MODR = 0x00;
float mx_f, my_f, mz_f;    int16_t mx, my, mz;                 // variables to hold latest accel data values

union {           // offset biases
  float fval;
  byte bval[4];
} magBias[3];

float mRes = 0.0015f;            // mag sensitivity
volatile bool newLSM303AGR_MagData = false; // used for data ready interrupt handling
bool newMag = false;
float param_magBias[3] = {0.0f, 0.0f, 0.0f}, param_magScale[3]  = {0.0f, 0.0f, 0.0f}; // Bias corrections for magnetometer
LSM303AGR_Mag LSM303AGR_Mag(&i2c_0); // instantiate LSM303AGR_Mag class

// **** GPS **** //
volatile bool isTracking = false;

GNSSLocation myLocation;
GNSSSatellites mySatellites;

#define GNSS_en      5     // enable for GNSS 3.0 V LDO
#define pps          4     // 1 Hz fix pulse
#define GNSS_backup A0     // RTC backup for MAX M8Q

uint8_t GPS_Hour = 12, GPS_Minute = 0, GPS_Second = 0, GPS_Year = 1, GPS_Month = 1, GPS_Day = 1;
//uint8_t hours = 12, minutes = 0, seconds = 0, year = 1, month = 1, day = 1;
//uint32_t subSeconds, milliseconds;
bool ppsFlag = false, firstSync = false, alarmFlag = true;
uint16_t count = 0, fixType = 0, fixQuality, latBytes[4], longBytes[4], tempBytes[4], pressBytes[4];
int32_t latOut, longOut;
float Long, Lat, Alt, EPE;
float EHPE = 99.9f ;
int Nb_Satellite ;
bool InMotion = true;
bool new_pos_gps = false;

static const char *fixTypeString[] = { "NONE", "TIME", "2D", "3D", };

static const char *fixQualityString[] = { "", "", "/DIFFERENTIAL", "/PRECISE", "/RTK_FIXED", "/RTK_FLOAT", "/ESTIMATED", "/MANUAL", "/SIMULATION", };

float lon_out[20], lat_out[20];


// **** Variable computing **** //
// Low-pass filter : Moving average filter
int16_t sum_buf_x = 0;
int16_t sum_buf_y = 0;
int16_t sum_buf_z = 0;
int16_t agx_b = 0;   // calculate the moving average
int16_t agy_b = 0;   // calculate the moving average
int16_t agz_b = 0;   // calculate the moving average
int16_t sum_buf_mx = 0;
int16_t sum_buf_my = 0;
int16_t sum_buf_mz = 0;
int16_t mgx_b = 0;   // calculate the moving average
int16_t mgy_b = 0;   // calculate the moving average
int16_t mgz_b = 0;   // calculate the moving average
int16_t inc_avg = 0;

//Raw accel
CircularBuffer<int16_t, len_buffer> buf_ax;
CircularBuffer<int16_t, len_buffer> buf_ay;
CircularBuffer<int16_t, len_buffer> buf_az;
//Raw Mag
CircularBuffer<int16_t, len_buffer> buf_mx;
CircularBuffer<int16_t, len_buffer> buf_my;
CircularBuffer<int16_t, len_buffer> buf_mz;

//Gravity Accel
CircularBuffer<int16_t, len_buffer> buf_agx;
CircularBuffer<int16_t, len_buffer> buf_agy;
CircularBuffer<int16_t, len_buffer> buf_agz;
//Gravity Mag
CircularBuffer<int16_t, len_buffer> buf_mgx;
CircularBuffer<int16_t, len_buffer> buf_mgy;
CircularBuffer<int16_t, len_buffer> buf_mgz;
//Dynamic Accel
CircularBuffer<int16_t, len_buffer> buf_adx;
CircularBuffer<int16_t, len_buffer> buf_ady;
CircularBuffer<int16_t, len_buffer> buf_adz;

float acc_gxf, acc_gyf, acc_gzf;
float acc_dxf, acc_dyf, acc_dzf;

int16_t acc_gx, acc_gy, acc_gz;
int16_t acc_dx, acc_dy, acc_dz;

float q_f[3]; int16_t q[3];
float pitch_f, roll_f, yaw_f; int16_t pitch, roll, yaw;
float heading_f; int16_t heading;
float depth_f; int16_t depth;
float h_speed_f; int16_t h_speed;
float VeDBA_f;   int16_t VeDBA;
float decli = +19.6 / 180 * PI;
float d_t = 0;
float tortuisity = 0;

uint16_t  inc_dive = 1;
uint16_t  inc_e = 0;
uint16_t  inc = 1;
uint16_t inc_filter = 1;

// Data to process before sending
int16_t buf_1[len] = {0};
int16_t buf_2[len] = {0};
int16_t z[len] = {0};
int16_t heading_array[len] = {0};
//int16_t speed_h[len] = {0};
int16_t offset_z = 0;
int16_t min_z = 0;

float x_out[nb_pos_out_max];
float y_out[nb_pos_out_max];
float z_out[nb_pos_out_max];


int16_t etho[nb_eth_out_max][4] = {0};
int16_t inc_eth = 0;
int16_t  etho_buf[3] = {0};
int inc_heading = 0;


//Slope variable calculation
int depth_slope;

// Variables algo speed 2
float fixed_speed = 0.45;
float p_lim = 0.3490; // in rad (20°)

const int nb_pts_max = 10;

uint16_t ind_min[1];

// MS5837 configuration
// Specify sensor full scale
uint8_t MS5837_OSR = ADC_8192;     // set pressure amd temperature oversample rate

uint16_t MS5837_Pcal[8];         // calibration constants from MS5837 PROM registers
unsigned char MS5837_nCRC;       // calculated check sum to ensure PROM integrity
uint32_t MS5837_D1 = 0, MS5837_D2 = 0;  // raw MS5837 pressure and temperature data
double MS5837_dT, MS5837_OFFSET, MS5837_SENS, MS5837_TT2, MS5837_OFFSET2, MS5837_SENS2;  // First order and second order corrections for raw MS5837 temperature and pressure data

double MS5837_Temperature, MS5837_Pressure; // stores MS5837 pressures sensor pressure and temperature
float fluidDensity = 1029.0f; // kg/m^3 for seawater

MS5837 MS5837(&i2c_0); // instantiate MS5837 class

// Variable and flag for ethogram
int flag_e[5][2] = {0};
int flag_s[5][2] = {0};

const int short_dive_limit = 30;
const int slope_up = -100;
const int slope_down = 100;
const int VeDBA_rest = 10;
const int surf_depth = 10;

// Timer to start a behavior (u = up, d = down, r = rest, s = swim, surf = surface)
const int t_d2u = 15;
const int t_d2r = 15;
const int t_r2u = 10;
const int t_r2d = 15;
const int t_u2surf = 15;
const int t_s2r = 15;
const int t_s2u = 15;
const int t_s2d = 15;

// Time to end a behavior to go in swim phase
const int t_d2s = 15;
const int t_u2s = 15;
const int t_r2s = 15;
const int t_surf2s = 15;

// Algorithm flag
bool start_dive_flag = 1;
bool end_dive_flag = 0;
bool flag_chg_behavior = 0;


// Function andrea
void GPS_first_fix( bool enable_serialPrint_data_GPS );
void read_update_GPS_data( bool enable_serialPrint_data_GPS );
void send_loRa_data( bool enable_serialPrint_loRaWan) ;
void init_loRa( void );
void read_battery_level( bool enable_serialPrint_battery );


// Timer
volatile bool flag_sampling_during_GPS = false; // used for data ready interrupt handling
TimerMillis samplingDuringGPS;  // instantiate low-frequency timer
volatile bool flag_new_depth_offset = false; // used for data ready interrupt handling
TimerMillis samplingDepthOffset;  // instantiate low-frequency timer


//************************************************************************************************************************************

void setup()
{


  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, LOW);  // start with blue led on (since active LOW)

  // Set the RTC time to firmware build time
  //SetDefaultRTC();

  Serial.begin(115200);
  delay(5000);
  Serial.println("\r\n--------------------------------------------------------------------");
  Serial.print("-----------------    TEST Consumption    07/2022    ----------------\r\n");
  Serial.println("--------------------------------------------------------------------\r\n");

  STM32L0.getUID(UID);
  Serial.print("STM32L0 MCU UID = 0x"); Serial.print(UID[0], HEX); Serial.print(UID[1], HEX); Serial.println(UID[2], HEX);

  // Batttery pin setup
  pinMode(myVBat_en, OUTPUT);
  digitalWrite(myVBat_en, LOW); // start with battery voltage monirtor off
  pinMode(myVBat, INPUT);
  analogReadResolution(12);

  VDDA = STM32L0.getVDDA();
  VBUS = STM32L0.getVBUS();
  STM32L0Temp = STM32L0.getTemperature();

  // Internal STM32L0 functions
  Serial.print("VDDA = "); Serial.print(VDDA, 2); Serial.println(" V");
  Serial.print("STM32L0 MCU Temperature = "); Serial.print(STM32L0Temp, 2); Serial.print(" C\r\n");
  Serial.println(" ");

  printAscale();
  printAODR();

  //init I2C
  I2C_BUS.begin();                                      // Set master mode
  delay(1000);
  I2C_BUS.setClock(400000);                             // I2C frequency at 400 kHz
  delay(1000);

  // set up Accelero
  pinMode(LSM303AGR_Acc_intPin1, INPUT);

  digitalWrite(myLed, HIGH);  // start with blue led on (since active LOW)
  delay(500); //stabilisation mesures
  digitalWrite(myLed, LOW);  // start with blue led on (since active LOW)
  delay(500); //stabilisation mesures


  aRes = LSM303AGR_Acc.getAres(Ascale); // get sensor resolution, only need to do this once
  LSM303AGR_Acc.selfTest();
  LSM303AGR_Acc.reset();
  LSM303AGR_Acc.init(Ascale, AODR);
  //LSM303AGR_Acc.offsetBias(param_accelBias);
  param_accelBias[0] = -0.01236;
  param_accelBias[1] = -0.09173;
  param_accelBias[2] = -0.01019;
  Serial.println("accel biases (mg)"); Serial.println(1000.0f * param_accelBias[0]); Serial.println(1000.0f * param_accelBias[1]); Serial.println(1000.0f * param_accelBias[2]);
  delay(1000);

  digitalWrite(myLed, HIGH);  // start with blue led on (since active LOW)
  delay(500); //stabilisation mesures
  digitalWrite(myLed, LOW);  // start with blue led on (since active LOW)
  delay(500); //stabilisation mesures
  digitalWrite(myLed, HIGH);  // start with blue led on (since active LOW)
  delay(500); //stabilisation mesures
  digitalWrite(myLed, LOW);  // start with blue led on (since active LOW)

  // set up Magnéto
  //pinMode(LSM303AGR_Mag_intPin, OUTPUT);
  LSM303AGR_Mag.selfTest();
  LSM303AGR_Mag.reset();
  LSM303AGR_Mag.init(MODR);
  //LSM303AGR_Mag.offsetBias(param_magBias, param_magScale);
  param_magBias[0] = 0.0005;
  param_magBias[1] = 0.022;
  param_magBias[2] = 0.0065;
  param_magScale[0] = 0.98;
  param_magScale[1] = 1;
  param_magScale[2] = 1.02;
  Serial.println("mag biases (mG)"); Serial.println(1000.0f * param_magBias[0]); Serial.println(1000.0f * param_magBias[1]); Serial.println(1000.0f * param_magBias[2]);
  Serial.println("mag scale (mG)"); Serial.println(param_magScale[0]); Serial.println(param_magScale[1]); Serial.println(param_magScale[2]);
  delay(2000); // add delay to see results before serial spew of data

  delay(100); //stabilisation mesures

  Serial.println("\r\nDataLog running...");


  //attachInterrupt(LSM303AGR_Acc_intPin1, myinthandler1, RISING);  // define data ready interrupt for intPin1 output of LSM303AGR_Acc
  //attachInterrupt(LSM303AGR_Mag_intPin, myinthandler2, RISING);  // define data ready interrupt for intPin1 output of LSM303AGR_Acc

  LSM303AGR_Acc.readAccData(accelData); // read data register to clear interrupt before main loop
  LSM303AGR_Mag.readMagData(magData); // read data register to clear status register

  getRTC(0); //timestamp

  // Init pressure
  // Reset the MS5837 pressure sensor
  MS5837.Reset();
  delay(100);
  Serial.println("MS5837 pressure sensor reset...");
  // Read PROM data from MS5837 pressure sensor
  MS5837.PromRead(MS5837_Pcal);
  Serial.println("PROM data read:");
  Serial.print("C0 = "); Serial.println(MS5837_Pcal[0]);
  unsigned char MS5837_refCRC = MS5837_Pcal[0] >> 12;
  Serial.print("C1 = "); Serial.println(MS5837_Pcal[1]);
  Serial.print("C2 = "); Serial.println(MS5837_Pcal[2]);
  Serial.print("C3 = "); Serial.println(MS5837_Pcal[3]);
  Serial.print("C4 = "); Serial.println(MS5837_Pcal[4]);
  Serial.print("C5 = "); Serial.println(MS5837_Pcal[5]);
  Serial.print("C6 = "); Serial.println(MS5837_Pcal[6]);

  MS5837_nCRC = MS5837.checkCRC(MS5837_Pcal);  //calculate checksum to ensure integrity of MS5837 calibration data
  Serial.print("Checksum = "); Serial.print(MS5837_nCRC); Serial.print(" , should be "); Serial.println(MS5837_refCRC);

  // First depth data to setup offset

  // Pressure MS5837 Data
  MS5837_D1 = MS5837.DataRead(ADC_D1, MS5837_OSR);  // get raw pressure value
  MS5837_D2 = MS5837.DataRead(ADC_D2, MS5837_OSR);  // get raw temperature value
  MS5837_dT = MS5837_D2 - MS5837_Pcal[5] * pow(2, 8); // calculate temperature difference from reference
  MS5837_OFFSET = MS5837_Pcal[2] * pow(2, 16) + MS5837_dT * MS5837_Pcal[4] / pow(2, 7);
  MS5837_SENS = MS5837_Pcal[1] * pow(2, 15) + MS5837_dT * MS5837_Pcal[3] / pow(2, 8);

  MS5837_Temperature = (2000 + (MS5837_dT * MS5837_Pcal[6]) / pow(2, 23)) / 100;     // First-order Temperature in degrees Centigrade

  // Second order corrections
  if (MS5837_Temperature > 20)
  {
    MS5837_TT2 = 2 * MS5837_dT * MS5837_dT / pow(2, 37); // correction for high temperatures
    MS5837_OFFSET2 = 1 * (100 * MS5837_Temperature - 2000) * (100 * MS5837_Temperature - 2000) / 16;
    MS5837_SENS2 = 0;
  }
  if (MS5837_Temperature < 20)                  // correction for low temperature
  {
    MS5837_TT2      = 3 * MS5837_dT * MS5837_dT / pow(2, 33);
    MS5837_OFFSET2 = 3 * (100 * MS5837_Temperature - 2000) * (100 * MS5837_Temperature - 2000) / 2;
    MS5837_SENS2   = 5 * (100 * MS5837_Temperature - 2000) * (100 * MS5837_Temperature - 2000) / 8;
  }
  if (MS5837_Temperature < -15)                     // correction for very low temperature
  {
    MS5837_OFFSET2 = MS5837_OFFSET2 + 7 * (100 * MS5837_Temperature + 1500) * (100 * MS5837_Temperature + 1500);
    MS5837_SENS2 = MS5837_SENS2 + 4 * (100 * MS5837_Temperature + 1500) * (100 * MS5837_Temperature + 1500);
  }
  // End of second order corrections

  MS5837_Temperature = MS5837_Temperature - MS5837_TT2 / 100;
  MS5837_OFFSET = MS5837_OFFSET - MS5837_OFFSET2;
  MS5837_SENS = MS5837_SENS - MS5837_SENS2;

  MS5837_Pressure = (((MS5837_D1 * MS5837_SENS) / pow(2, 21) - MS5837_OFFSET) / pow(2, 13)) / 10; // Pressure in mbar or hPa

  // Pressure in Pa, density in kg/mm^3,, gravity in m/s^2, i.e., SI units
  float MS5837_depth = (MS5837_Pressure * 100.0f - 101300.0f) / (fluidDensity * 9.80665f);
  offset_z = int16_t(MS5837_depth * 100);

  if (serial_print) {
    Serial.print("Depth :"); Serial.println(MS5837_depth);
    Serial.print("offset_z :"); Serial.println(offset_z);
  }

  pinMode(GNSS_backup, OUTPUT);       // Power for MAX M8Q RTC backup
  digitalWrite(GNSS_backup, HIGH);    // Setup GNSS

  // --- Configuration of GNSS --- //
  GNSS.begin(Serial1, GNSS.MODE_UBLOX, GNSS.RATE_1HZ);        // Start GNSS
  while (GNSS.busy()) { }                                     // Wait for begin to complete

  GNSS.setConstellation(GNSS.CONSTELLATION_GPS_AND_GLONASS) ; // Choose satellites
  while (GNSS.busy()) { }                                     // Wait for set to complete

  GNSS.setAntenna(GNSS.ANTENNA_EXTERNAL);                     // GNSS.ANTENNA_INTERNAL or GNSS.ANTENNA_EXTERNAL
  while (GNSS.busy()) { }                                     // Wait for set to complete

  GNSS.enableWakeup();                                        // Wake up
  while (GNSS.busy()) { }                                     // Wait for set to complete

  //Timer_GPS.start( Read_GPS , 0 , 20000 );
  if ( use_GPS == true ) {
    GPS_first_fix( true ) ;
  } else {
    Lat = -20.932876; //-21.094209; //Eperon, Lat = -20.932937; // Le Port  // planch al Lat=-21.094209
    Long = 55.290323; // EperonLong =  55.290304; // Le Port // planch al Long= 55.234457;
    lat_out[0] = Lat;
    lon_out[0] = Long;
  }

  GNSS.suspend() ;
  new_pos_gps = true;


  delay(1000);
  digitalWrite(myLed, HIGH); // turn off led when configuration successfully completed
  //pinMode(myLed, INPUT);

  buf_1[0] = 0;
  buf_2[0] = 0;

  //Timer sampling during GPS
  samplingDuringGPS.start(callbackSamplingDuringGPS, 0,   1000);    // high freq (one minute) timer
  //Timer offset pressure sensor
  samplingDepthOffset.start(callbackDepthOffset, 0,   time_depth_offset);    // high freq (one minute) timer

  STM32L0.stop();        // Enter STOP mode and wait for an interrupt


} /* end of setup */



void loop()
{



  /**************** START NEW DIVE ******************************/
  // Set behavior to SURFACE
  // Read GPS until depth > 0.2m

  if (start_dive_flag == 1) {

    digitalWrite(myLed, LOW); // Turn on led when New Dive or fixed successfully completed
    start_dive_flag = 0;
    buf_1[0] = 0;
    buf_2[0] = 0;
    inc = 1;
    d_t = 0;

    if (serial_print) {
      Serial.println("New Dive");
    }

    inc_eth = 0;
    etho[inc_eth][1] = 5;
    etho[inc_eth][2] = inc;
    etho[inc_eth][4] = z[inc];

    //GPS
    if ( use_GPS == true ) {
      read_update_GPS_data( true );
      inc++;
    }

    if (new_pos_gps == true) {
      lat_out[0] = Lat;
      lon_out[0] = Long;
      new_pos_gps = false;
    } else {
      lat_out[0] = lat_out[nb_pos_out - 1];
      lon_out[0] = lon_out[nb_pos_out - 1];
    }

    etho[inc_eth][3] = inc - 1;
    inc_eth = 1;
    etho[inc_eth][1] = 1;
    etho[inc_eth][2] = inc;
    etho[inc_eth][4] = z[inc];

    digitalWrite(myLed, HIGH); // turn off led when GPS Timeout or fixed successfully completed
  }


  /***************** COMPUTING AFTER CHANGE BEHAVIOR ********************/
  // Computing speed and trajectory

  if (flag_chg_behavior == 1) {

    flag_chg_behavior = 0;
    int first_data = etho[inc_eth - 1][2];
    int last_data =  etho[inc_eth - 1][3];
    speed_fct_F2(etho[inc_eth - 1][1]); // speed calculation with fct 1
    //htrack_fct(first_data, last_data, serial_debug);

    if (serial_print) {
      Serial.println("New Behavior");
      Serial.println("Start speed and traj calculation");
      Serial.print("first_data : "); Serial.println(first_data);
      Serial.print("last_data :"); Serial.println(last_data);
      Serial.print("Speed : ");  Serial.println(h_speed);
    }

    inc_heading = 0;

  }


  /***************** SAMPLING ********************/

  if (flag_sampling_during_GPS == true) // Samping flag every 1sec
  {

    flag_sampling_during_GPS = false;


    //  SAMPLING SWIM : ACC / MAG / DEPTH
    if (etho[inc_eth][1] == 1) {

      if (led_sampling) {
        digitalWrite(myLed, HIGH);
      }

      LSM303AGR_Acc.readAccData(accelData); // INT1 cleared on any read

      // Now we'll calculate the accleration value into actual g's
      ax_f = (float)accelData[0] * aRes - param_accelBias[0]; // get actual g value, this depends on scale being set
      ay_f = (float)accelData[1] * aRes - param_accelBias[1];
      az_f = (float)accelData[2] * aRes - param_accelBias[2];

      ax =  ax_f * 1000;
      ay =  ay_f * 1000;
      az =  az_f * 1000;

      Serial.print("ax : ");  Serial.println(ax); // Test timing
      Serial.print("ay : ");  Serial.println(ay); // Test timing
      Serial.print("az : ");  Serial.println(az); // Test timing

      byte c = 0;
      //Serial.print("mag 1  ");
      //delay(1);
      while (!((c >> 3) & 0x01))  // attente data mag
      {
        c = LSM303AGR_Mag.getStatus();
      }

      LSM303AGR_Mag.readMagData(magData);  // INT2 cleared on any read

      mx_f = (float)magData[0] * mRes - param_magBias[0]; // get actual G value
      my_f = (float)magData[1] * mRes - param_magBias[1];
      mz_f = (float)magData[2] * mRes - param_magBias[2];
      mx_f *= param_magScale[0];
      my_f *= param_magScale[1];
      mz_f *= param_magScale[2];

      mx =  mx_f * 1000;
      my =  my_f * 1000;
      mz =  mz_f * 1000;


      // Pressure MS5837 Data
      MS5837_D1 = MS5837.DataRead(ADC_D1, MS5837_OSR);  // get raw pressure value
      MS5837_D2 = MS5837.DataRead(ADC_D2, MS5837_OSR);  // get raw temperature value
      MS5837_dT = MS5837_D2 - MS5837_Pcal[5] * pow(2, 8); // calculate temperature difference from reference
      MS5837_OFFSET = MS5837_Pcal[2] * pow(2, 16) + MS5837_dT * MS5837_Pcal[4] / pow(2, 7);
      MS5837_SENS = MS5837_Pcal[1] * pow(2, 15) + MS5837_dT * MS5837_Pcal[3] / pow(2, 8);

      MS5837_Temperature = (2000 + (MS5837_dT * MS5837_Pcal[6]) / pow(2, 23)) / 100;     // First-order Temperature in degrees Centigrade

      // Second order corrections
      if (MS5837_Temperature > 20)
      {
        MS5837_TT2 = 2 * MS5837_dT * MS5837_dT / pow(2, 37); // correction for high temperatures
        MS5837_OFFSET2 = 1 * (100 * MS5837_Temperature - 2000) * (100 * MS5837_Temperature - 2000) / 16;
        MS5837_SENS2 = 0;
      }
      if (MS5837_Temperature < 20)                  // correction for low temperature
      {
        MS5837_TT2      = 3 * MS5837_dT * MS5837_dT / pow(2, 33);
        MS5837_OFFSET2 = 3 * (100 * MS5837_Temperature - 2000) * (100 * MS5837_Temperature - 2000) / 2;
        MS5837_SENS2   = 5 * (100 * MS5837_Temperature - 2000) * (100 * MS5837_Temperature - 2000) / 8;
      }
      if (MS5837_Temperature < -15)                     // correction for very low temperature
      {
        MS5837_OFFSET2 = MS5837_OFFSET2 + 7 * (100 * MS5837_Temperature + 1500) * (100 * MS5837_Temperature + 1500);
        MS5837_SENS2 = MS5837_SENS2 + 4 * (100 * MS5837_Temperature + 1500) * (100 * MS5837_Temperature + 1500);
      }
      // End of second order corrections

      MS5837_Temperature = MS5837_Temperature - MS5837_TT2 / 100;
      MS5837_OFFSET = MS5837_OFFSET - MS5837_OFFSET2;
      MS5837_SENS = MS5837_SENS - MS5837_SENS2;

      MS5837_Pressure = (((MS5837_D1 * MS5837_SENS) / pow(2, 21) - MS5837_OFFSET) / pow(2, 13)) / 10; // Pressure in mbar or hPa

      // Pressure in Pa, density in kg/mm^3,, gravity in m/s^2, i.e., SI units
      float MS5837_depth = (MS5837_Pressure * 100.0f - 101300.0f) / (fluidDensity * 9.80665f);

      z[inc] = int16_t(MS5837_depth * 100) - offset_z;

      if (serial_print) {
        Serial.print("Depth :"); Serial.println(MS5837_depth);
        Serial.print("Depth_z :"); Serial.println(z[inc]);
      }

      // LOW-PASS FILTER

      if (inc_filter <= len_buffer)
      {

        t_start = micros();     // Test timing

        sum_buf_x = sum_buf_x + ax;
        sum_buf_y = sum_buf_y + ay;
        sum_buf_z = sum_buf_z + az;

        sum_buf_mx = sum_buf_mx + mx;
        sum_buf_my = sum_buf_my + my;
        sum_buf_mz = sum_buf_mz + mz;

        buf_ax.unshift(ax);
        buf_ay.unshift(ay);
        buf_az.unshift(az);

        buf_mx.unshift(mx);
        buf_my.unshift(my);
        buf_mz.unshift(mz);

        // Filter raw acc to get gravitationnal acceleration
        agx_b = sum_buf_x / (inc_filter);
        agy_b = sum_buf_y / (inc_filter);
        agz_b = sum_buf_z / (inc_filter);

        // Filter raw acc to get gravitationnal mag
        mgx_b = sum_buf_mx / (inc_filter);
        mgy_b = sum_buf_my / (inc_filter);
        mgz_b = sum_buf_mz / (inc_filter);


      } else  //Length /2 of the moving average filter. Data is delayed of this number of sample
      {


        /**************************************************************************************************************/
        t_start = micros();     // Test timing


        sum_buf_x = sum_buf_x + ax - buf_ax[len_buffer - 1];
        sum_buf_y = sum_buf_y + ay - buf_ay[len_buffer - 1];
        sum_buf_z = sum_buf_z + az - buf_az[len_buffer - 1];

        sum_buf_mx = sum_buf_mx + mx - buf_mx[len_buffer - 1];
        sum_buf_my = sum_buf_my + my - buf_my[len_buffer - 1];
        sum_buf_mz = sum_buf_mz + mz - buf_mz[len_buffer - 1];

        // Low-pass filter : Moving average filter
        // Fill the buffer of raw accel data
        buf_ax.unshift(ax);
        buf_ay.unshift(ay);
        buf_az.unshift(az);

        buf_mx.unshift(mx);
        buf_my.unshift(my);
        buf_mz.unshift(mz);

        // Filter raw acc to get gravitationnal acceleration
        agx_b = sum_buf_x / (len_buffer);
        agy_b = sum_buf_y / (len_buffer);
        agz_b = sum_buf_z / (len_buffer);

        // Filter raw acc to get gravitationnal acceleration
        mgx_b = sum_buf_mx / (len_buffer);
        mgy_b = sum_buf_my / (len_buffer);
        mgz_b = sum_buf_mz / (len_buffer);
      }

      int16_t agx = agx_b;//float(100);
      int16_t agy = agy_b;//float(100);
      int16_t agz = agz_b;//float(100);

      int16_t adx = ax - agx;//float(100);
      int16_t ady = ay - agy;//float(100);
      int16_t adz = az - agz;//float(100);

      int16_t mgx = mgx_b;//float(100);
      int16_t mgy = mgy_b;//float(100);
      int16_t mgz = mgz_b;//float(100);

      t_end = micros();     // Test timing
      Serial.print("Time filter 1Hz: ");  // Test timing
      t_filter = t_filter + (t_end - t_start);
      inc_avg++;
      Serial.println(t_end - t_start);  // Test timing

      Serial.print("agx = "); Serial.print(int( agx));
      Serial.print("agy = "); Serial.print(int(agy));
      Serial.print("agz = "); Serial.print(int(agz)); Serial.println(" mg");
      Serial.print("adx = "); Serial.print(int(adx));
      Serial.print("ady = "); Serial.print(int(ady));
      Serial.print("adz = "); Serial.print(int(adz)); Serial.println(" mg");

      /**************************************************************************************************************/
      t_start = micros();     // Test timing

      roll_f = atan2(-agy, agz);
      pitch_f = atan(agx / (-agx * sin(roll) + agz * cos(roll)));
      int16_t magn_fy_fs = (mgz * sin(roll) + mgy * cos(roll)) * 10000;
      int16_t magn_fx_fs = (-mgx * cos(pitch) - mgy * sin(pitch) * sin(roll) + mgz * sin(pitch) * cos(roll)) * 10000;
      yaw_f = atan2(magn_fy_fs, magn_fx_fs);

      t_end = micros();     // Test timing
      Serial.print("Time orientation : ");  // Test timing
      t_o = t_o + (t_end - t_start);
      //inc_timing_filter++;
      Serial.println(t_end - t_start);  // Test timing

      /**************************************************************************************************************/
      /*t_start = micros();     // Test timing

        SAAM(agx, agy, agz, mgx, mgy, mgz);
        //q[0] = q_f[0];  q[1] = q_f[1]; q[2] = q_f[2]; q[3] = q_f[3];
        float yaw2_f   = atan2(2.0f * (q_f[1] * q_f[2] + q_f[0] * q_f[3]), q_f[0] * q_f[0] + q_f[1] * q_f[1] - q_f[2] * q_f[2] - q_f[3] * q_f[3]);
        pitch_f = -asin(2.0f * (q_f[1] * q_f[3] - q_f[0] * q_f[2]));
        roll_f  = atan2(2.0f * (q_f[0] * q_f[1] + q_f[2] * q_f[3]), q_f[0] * q_f[0] - q_f[1] * q_f[1] - q_f[2] * q_f[2] + q_f[3] * q_f[3]);

        t_end = micros();     // Test timing
        Serial.print("Time orientation 2 : ");  // Test timing
        //t_filter = t_filter + (t_end - t_start);
        //inc_timing_filter++;
        Serial.println(t_end - t_start);  // Test timing*/

      /**************************************************************************************************************/

      heading = int16_t(yaw_f * 10000) -  int16_t(decli * 10000); // - (90 / 180) * PI;

      if (heading < 0) {
        heading = heading + uint16_t(2 * PI * 10000);
      }

      /**************************************************************************************************************/
      /*Slope variable calculation */
      
      t_start = micros(); // Test timing
      int16_t slope = z[inc] - z[inc - 1];
      t_end = micros();     // Test timing
      Serial.print("Time diff depth : ");  // Test timing
      t_depth = t_depth + (t_end - t_start);
      Serial.println(t_end - t_start);  // Test timing

      /**************************************************************************************************************/
      /* VeDBA */
      t_start = micros();     // Test timing
      VeDBA = sqrt(adx * adx + ady * ady + adz * adz);
      t_end = micros();     // Test timing
      Serial.print("Time VeDBA : ");  // Test timing
      t_VeDBA = t_VeDBA + (t_end - t_start);
      Serial.println(t_end - t_start);  // Test timing

      /**************************************************************************************************************/
      /* Jerk */
      t_start = micros();     // Test timing
      int16_t jerk = sqrt(abs(adx - ady) + abs(ady - adz) + abs(- adz + adx));
      t_end = micros();     // Test timing
      Serial.print("Time Jerk : ");  // Test timing
      Serial.println(t_end - t_start);  // Test timing
      t_jerk = t_jerk + (t_end - t_start);

      /**************************************************************************************************************/

      if (serial_print) {
        Serial.print("yaw : ");    Serial.println(float(heading) * 180 / PI / 10000); // Test timing
        Serial.print("pitch : ");  Serial.println(pitch_f * 180 / PI); // Test timing// Test timing
        Serial.print("roll : ");  Serial.println(roll_f * 180 / PI); // Test timing// Test timing
        Serial.print("VeDBA : ");  Serial.println(VeDBA); // Test timing// Test timing
      }

      inc_filter++;

      heading_array[inc] = heading;


      /**************************************************************************************************************/
      /* Speed REG */

      t_start = micros();     // Test timing
      speed_fct_REG(adx);
      t_end = micros();     // Test timing
      Serial.print("Time reg: ");  // Test timing
      Serial.println(t_end - t_start);  // Test timing
      t_reg = t_reg + (t_end - t_start);

      /**************************************************************************************************************/
      /* Speed  */

      t_start = micros();     // Test timing
      speed_fct_OCDR(depth, depth + 1, pitch);;
      t_end = micros();     // Test timing
      Serial.print("Time ocdr: ");  // Test timing
      Serial.println(t_end - t_start);  // Test timing
      t_ocdr = t_ocdr + (t_end - t_start);



      /**************************************************************************************************************/
      /* Speed F2 */

      t_start = micros();     // Test timing
      speed_fct_F2(1);   // speed calculation with fct 2
      t_end = micros();     // Test timing
      Serial.print("Time F2: ");  // Test timing
      Serial.println(t_end - t_start);  // Test timing
      t_sF2 = t_sF2 + (t_end - t_start);

      /**************************************************************************************************************/
      /* Speed F3 */

      t_start = micros();     // Test timing
      speed_fct_F3(depth, depth + 1, pitch);
      t_end = micros();     // Test timing
      Serial.print("Time F3: ");  // Test timing
      Serial.println(t_end - t_start);  // Test timing
      t_sF3 = t_sF3 + (t_end - t_start);


      /**************************************************************************************************************/
      /* Speed F4 */

      t_start = micros();     // Test timing
      speed_fct_F4();
      t_end = micros();     // Test timing
      Serial.print("Time F4: ");  // Test timing
      Serial.println(t_end - t_start);  // Test timing
      t_sF4 = t_sF4 + (t_end - t_start);




      /**************************************************************************************************************/
      /* Distance */

      t_start = micros();     // Test timing
      float x_buf = h_speed * cos(float(heading_array[inc]) / 10000);
      float y_buf = h_speed * sin(float(heading_array[inc]) / 10000);

      float d = sqrt(  x_buf * x_buf + y_buf * y_buf);
      d_t = d_t + d; // Total distance travelled

      buf_1[inc] = buf_1[inc - 1] + int16_t(x_buf * 100);
      buf_2[inc] = buf_2[inc - 1] + int16_t(y_buf * 100);

      t_end = micros();     // Test timing
      Serial.print("Time Traj: ");  // Test timing
      Serial.println(t_end - t_start);  // Test timing
      t_traj = t_traj + (t_end - t_start);


    } // END SAMPLING SWIM : ACC / MAG / DEPTH




    // COMMON ALGO FOR ALL BEHAVIOR

    // Ethogram
    //etho_fct(); // Ethogram calculation

    inc++; // Increment number of dive sample

    if (serial_print) {
      Serial.println("**************************************************************************************");
      Serial.print("Inc : "); Serial.println(inc);  // Test timing
      Serial.println("**************************************************************************************");
    }

    if (led_sampling) {
      digitalWrite(myLed, HIGH); // turn off led when configuration successfully completed
    }
  }


  /*************** END DIVE *************/
  // Processing trajectory
  // Calculate nb of position available to send
  // Compression
  // Transmission

  if (inc >= len || end_dive_flag == 1) {

    end_dive_flag = 0;

    t_filter = t_filter / inc_avg;
    t_traj = t_traj / inc_avg;
    t_o = t_o / inc_avg;
    t_VeDBA = t_VeDBA / inc_avg;
    t_jerk = t_jerk / inc_avg;
    t_reg = t_reg / inc_avg;
    t_ocdr = t_ocdr / inc_avg;
    t_sF2 = t_sF2 / inc_avg;
    t_sF3 = t_sF3 / inc_avg;
    t_sF4 = t_sF4 / inc_avg;
    t_depth = t_depth / inc_avg;
    inc_avg = 0;


    Serial.print("Time avg filter : "); Serial.println(t_filter);  // Test timing
    Serial.print("Time avg traj: "); Serial.println(t_traj);  // Test timing
    Serial.print("Time avg orientation: "); Serial.println(t_o);  // Test timing
    Serial.print("Time avg VeDBA: "); Serial.println(t_VeDBA);  // Test timing
    Serial.print("Time avg jerk: "); Serial.println(t_jerk);  // Test timing
    Serial.print("Time avg depth: "); Serial.println(t_depth);  // Test timing

    Serial.print("Time avg reg : "); Serial.println(t_reg);  // Test timing
    Serial.print("Time avg ocdr: "); Serial.println(t_ocdr);  // Test timing
    Serial.print("Time avg sf2: "); Serial.println(t_sF2);  // Test timing
    Serial.print("Time avg sF3: "); Serial.println(t_sF3);  // Test timing
    Serial.print("Time avg sF4: "); Serial.println(t_sF4);  // Test timing



    // ADD in etho function
    start_dive_flag = 1;
    memset(flag_s, 0, sizeof(flag_s));
    memset(flag_e, 0, sizeof(flag_e));

    // FIN ALGO  PARTI 2/2

    int nb_eth = inc_eth + 1;
    int first_data = etho[inc_eth][2];
    int last_data =  inc;

    //speed_fct_1(etho[inc_eth][1]);   // speed calculation with fct 1

    //htrack_fct(first_data, last_data, serial_debug );

    // Calculate nb of position available to send
    //Compute message variable
    //Allocate ethogram payload size

    //Clear variable
    buf_mx.clear();
    buf_my.clear();
    buf_mz.clear();

    buf_ax.clear();
    buf_ay.clear();
    buf_az.clear();

    sum_buf_x = 0;
    sum_buf_y = 0;
    sum_buf_z = 0;

    sum_buf_mx = 0;
    sum_buf_my = 0;
    sum_buf_mz = 0;

    inc_dive++;
    inc_filter = 1;

    inc_e = 0;

    buf_1[0] = 0;
    buf_2[0] = 0;

  }

  STM32L0.stop();        // Enter STOP mode and wait for an interrupt



}  /* end of loop*/

//***************************************************************************************************************************

void myinthandler1()
{
  newLSM303AGR_AccData_1 = true;
  STM32L0.wakeup();
  //Serial.println("int wake up accel");
}

void callbackSamplingDuringGPS(void)
{
  flag_sampling_during_GPS = true;
  //Serial.println("callbackSamplingDuringGPS");
  STM32L0.wakeup();
}

void callbackDepthOffset(void) {
  flag_new_depth_offset = true;
  STM32L0.wakeup();
}


void SAAM(int16_t axb, int16_t ayb, int16_t azb, int16_t mxb, int16_t myb, int16_t mzb)
{
  float mD = axb * mxb + ayb * myb + azb * mzb;
  float mN = sqrt(1 - mD * mD);
  float norm_q;

  q_f[0] = - ayb * (mN + mxb) + axb * myb;
  q_f[1] =  (azb - 1) * (mN + mxb) + axb * (mD - mzb);
  q_f[2] =  (azb - 1) * myb + ayb * (mD - mzb);
  q_f[3] =  azb * mD - axb * mN - mzb;

  norm_q = sqrt(q_f[0] * q_f[0] + q_f[1] * q_f[1] + q_f[2] * q_f[2] + q_f[3] * q_f[3]);

  for (int i = 0; i < 4; i++) {
    q_f[i] = q_f[i] / norm_q;
  }

}

void ToEulerAngles(int16_t qw, int16_t qx, int16_t qy, int16_t qz) {


  // roll (x-axis rotation)
  int16_t sinr_cosp = (2 * (qw * qx + qy * qz)) * 1000;
  int16_t cosr_cosp = (1 - 2 * (qx * qx + qy * qy)) * 1000;
  roll = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  float sinp = 2 * (qw * qy - qz * qx);
  /*if (std::abs(sinp) >= 1)
      angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else*/
  pitch = asin(sinp);

  // yaw (z-axis rotation)
  int16_t siny_cosp = (2 * (qw * qz + qx * qy)) * 1000;
  int16_t cosy_cosp = (1 - 2 * (qy * qy + qz * qz)) * 1000;
  yaw = atan2(siny_cosp, cosy_cosp);

}


float pitch_fct(float xx, float yy, float zz)
{
  float pitch = atan2(-xx, sqrt(pow(yy, 2) + pow(zz, 2)));
  return pitch;
}

float roll_fct(float yy, float zz)
{
  float roll = atan2(yy, zz);
  return roll;
}

float yaw_fct(float f_pitch, float f_roll, float f_mx, float f_my, float f_mz)
{
  float by2 = f_mz * sin(f_roll) - f_my * cos(f_roll);
  float bz2 = f_my * sin(f_roll) + f_mz * cos(f_roll);
  float bx3 = f_mx * cos(f_pitch) + bz2 * sin(f_pitch);
  float yaw = atan2(by2, bx3);
  return yaw;
}


/* GPS */

void GPS_first_fix( bool enable_serialPrint_data_GPS ) {

  Serial.println("GPS_First_Fix");

  STM32L0.wakeup() ;

  delay(100);

  GNSS.resume();
  while (GNSS.busy()) { }                                     // Wait for set to complete

  int now = millis() ;

  EHPE = 999.99f ;
  //Serial.println(EHPE);

  // Serial.println( (String)"While loop - condition n°2 : " + (myLocation.ehpe() > 10) ) ;

  while ( (EHPE >= 10.0) ) { // Waiting to have a "good" EHPE

    //Serial.println(EHPE);
    // Serial.println( (String)"While loop - condition n°1 : " + (millis() - now <= 30000) ) ;
    // Serial.println( (String)"While loop - condition n°2 : " + (myLocation.ehpe() > 10) ) ;

    // Serial.println("Hello");

    EHPE = 999.99f ;

    // --- GNSS --- //
    if (GNSS.location(myLocation) ) {

      if ( enable_serialPrint_data_GPS == true ) {
        Serial.print("LOCATION: ");
        Serial.print(fixTypeString[myLocation.fixType()]);
        if ( fixTypeString[myLocation.fixType()] == "NONE") {
          Serial.println(".") ;
        }
      }

      if (myLocation.fixType() != GNSSLocation::TYPE_NONE) {

        GPS_Year   = myLocation.year()    ;
        GPS_Month  = myLocation.month()   ;
        GPS_Day    = myLocation.day()     ;
        GPS_Hour   = myLocation.hours()   ;
        GPS_Minute = myLocation.minutes() ;
        GPS_Second = myLocation.seconds() ;

        // Display date
        if ( enable_serialPrint_data_GPS == true ) {
          Serial.print(fixQualityString[myLocation.fixQuality()]);
          Serial.print(" ");
          Serial.print(myLocation.year());
          Serial.print("/");
          Serial.print(myLocation.month());
          Serial.print("/");
          Serial.print(myLocation.day());
          Serial.print(" ");

          // Display hours
          if (myLocation.hours() <= 9) {
            Serial.print("0");
          }
          Serial.print(myLocation.hours());
          Serial.print(":");

          if (myLocation.minutes() <= 9) {
            Serial.print("0");
          }
          Serial.print(myLocation.minutes());
          Serial.print(":");

          if (myLocation.seconds() <= 9) {
            Serial.print("0");
          }
          Serial.print(myLocation.seconds());


          if (myLocation.leapSeconds() != GNSSLocation::LEAP_SECONDS_UNDEFINED) {
            Serial.print(" ");
            Serial.print(myLocation.leapSeconds());
            if (!myLocation.fullyResolved()) {
              Serial.print("D");
            }
          }

        }

        if (myLocation.fixType() != GNSSLocation::TYPE_TIME) {

          Lat = myLocation.latitude();
          myLocation.latitude(latOut);
          Long = myLocation.longitude();
          myLocation.longitude(longOut);
          Alt = myLocation.altitude();
          EHPE = myLocation.ehpe(); // use this as accuracy figure of merit
          Nb_Satellite = mySatellites.count() ;

          if ( enable_serialPrint_data_GPS == true ) {
            Serial.print(" : ");
            Serial.print(Lat, 7); Serial.print(","); Serial.print(Long, 7);
            Serial.print(",");
            Serial.print(Alt, 3);
            Serial.print(" EHPE :");
            Serial.print(EHPE, 3);
            Serial.print(",");
            Serial.print(myLocation.evpe(), 3);
            Serial.print(" SAT. :");
            Serial.print(myLocation.satellites());
            Serial.print(" DOP :");
            Serial.print(myLocation.hdop(), 2);
            Serial.print(",");
            Serial.print(myLocation.vdop(), 2);
            Serial.print(" - SATELLITES: "); Serial.println(mySatellites.count()) ;

          }



          // if( (myLocation.fixType() != GNSSLocation::TYPE_2D) && (EPE < 10.0f) ) { // 10 is about as low as one should go, 50 is acceptable
          // if( (EPE < 50.0f) ) { // 10 is about as low as one should go, 50 is acceptable
          //   Serial.println("***GNSS go to sleep!***");
          //   GNSS.suspend(); // once we have a good 3D location fix put CAM M8Q to sleep
          //   //break ;
          // }

        } // if (myLocation.fixType() != GNSSLocation::TYPE_TIME)

      } // if (myLocation.fixType() != GNSSLocation::TYPE_NONE)

    } // if (GNSS.location(myLocation))


    if (GNSS.satellites(mySatellites))
    {

      Serial.print("SATELLITES: ");
      Serial.print(mySatellites.count());

      Serial.println();

      for (unsigned int index = 0; index < mySatellites.count(); index++)
      {
        unsigned int svid = mySatellites.svid(index);

        if ((svid >= 1) && (svid <= 32))
        {
          Serial.print("    ");

          if (svid <= 9)
          {
            Serial.print("  G");
            Serial.print(svid);
          }
          else
          {
            Serial.print(" G");
            Serial.print(svid);
          }
        }
        else if ((svid >= 65) && (svid <= 96))
        {
          Serial.print("    ");

          if ((svid - 64) <= 9)
          {
            Serial.print("  R");
            Serial.print(svid - 64);
          }
          else
          {
            Serial.print(" R");
            Serial.print(svid - 64);
          }
        }
        else if ((svid >= 120) && (svid <= 158))
        {
          Serial.print("    ");
          Serial.print("S");
          Serial.print(svid);
        }
        else if ((svid >= 173) && (svid <= 182))
        {
          Serial.print("    ");
          Serial.print("  I");
          Serial.print(svid - 172);
        }
        else if ((svid >= 193) && (svid <= 197))
        {
          Serial.print("    ");
          Serial.print("  Q");
          Serial.print(svid - 192);
        }
        else if ((svid >= 211) && (svid <= 246))
        {
          Serial.print("    ");

          if ((svid - 210) <= 9)
          {
            Serial.print("  E");
            Serial.print(svid - 210);
          }
          else
          {
            Serial.print(" E");
            Serial.print(svid - 210);
          }
        }
        else if (svid == 255)
        {
          Serial.print("    ");
          Serial.print("R???");
        }
        else
        {
          continue;
        }

        Serial.print(": SNR=");
        Serial.print(mySatellites.snr(index));
        Serial.print(", ELEVATION=");
        Serial.print(mySatellites.elevation(index));
        Serial.print(", AZIMUTH=");
        Serial.print(mySatellites.azimuth(index));

        if (mySatellites.unhealthy(index)) {
          Serial.print(", UNHEALTHY");
        }

        if (mySatellites.almanac(index)) {
          Serial.print(", ALMANAC");
        }

        if (mySatellites.ephemeris(index)) {
          Serial.print(", EPHEMERIS");
        }

        if (mySatellites.autonomous(index)) {
          Serial.print(", AUTONOMOUS");
        }

        if (mySatellites.correction(index)) {
          Serial.print(", CORRECTION");
        }

        if (mySatellites.acquired(index)) {
          Serial.print(", ACQUIRED");
        }

        if (mySatellites.locked(index)) {
          Serial.print(", LOCKED");
        }

        if (mySatellites.navigating(index)) {
          Serial.print(", NAVIGATING");
        }

        Serial.println();
      }

    } /* end of GNSS Satellites handling */

    // --- Information about satellites --- //
    // if (GNSS.satellites(mySatellites)){

    //     if( Enable_SerialPrint_Data_GPS == true ){
    //       Serial.print(" - SATELLITES: "); Serial.println(mySatellites.count()) ;
    //       Nb_Satellite = mySatellites.count() ;
    //     }

    // } // if (GNSS.satellites(mySatellites))


  } // while( (millis() - now <= 60000)   )

  GNSS.suspend() ;
  new_pos_gps = true;


  Serial.println("First Fix GPS done.") ;

}

void read_update_GPS_data( bool enable_serialPrint_data_GPS ) {

  Serial.println("Read_Update_GPS_Data");


  //STM32L0.wakeup() ;

  delay(10);

  GNSS.resume();

  delay(10);

  while (GNSS.busy()) { }                                     // Wait for set to complete
  Serial.println("Resume done");

  EHPE = 99.99f ;

  int now = millis() ;

  // Serial.println( (String)"While loop - condition n°2 : " + (myLocation.ehpe() > 10) ) ;

  while ( (millis() - now <= time_out_GPS)  ) {

    if (flag_sampling_during_GPS == true)
    {
      flag_sampling_during_GPS = false;
      Serial.println("Z, inc++ and X,Y during GPS search");
      inc++; // Increment number of dive sample

      // Sampling pressure
      // Pressure MS5837 Data
      MS5837_D1 = MS5837.DataRead(ADC_D1, MS5837_OSR);  // get raw pressure value
      MS5837_D2 = MS5837.DataRead(ADC_D2, MS5837_OSR);  // get raw temperature value
      MS5837_dT = MS5837_D2 - MS5837_Pcal[5] * pow(2, 8); // calculate temperature difference from reference
      MS5837_OFFSET = MS5837_Pcal[2] * pow(2, 16) + MS5837_dT * MS5837_Pcal[4] / pow(2, 7);
      MS5837_SENS = MS5837_Pcal[1] * pow(2, 15) + MS5837_dT * MS5837_Pcal[3] / pow(2, 8);

      MS5837_Temperature = (2000 + (MS5837_dT * MS5837_Pcal[6]) / pow(2, 23)) / 100;     // First-order Temperature in degrees Centigrade

      // Second order corrections
      if (MS5837_Temperature > 20)
      {
        MS5837_TT2 = 2 * MS5837_dT * MS5837_dT / pow(2, 37); // correction for high temperatures
        MS5837_OFFSET2 = 1 * (100 * MS5837_Temperature - 2000) * (100 * MS5837_Temperature - 2000) / 16;
        MS5837_SENS2 = 0;
      }
      if (MS5837_Temperature < 20)                  // correction for low temperature
      {
        MS5837_TT2      = 3 * MS5837_dT * MS5837_dT / pow(2, 33);
        MS5837_OFFSET2 = 3 * (100 * MS5837_Temperature - 2000) * (100 * MS5837_Temperature - 2000) / 2;
        MS5837_SENS2   = 5 * (100 * MS5837_Temperature - 2000) * (100 * MS5837_Temperature - 2000) / 8;
      }
      if (MS5837_Temperature < -15)                     // correction for very low temperature
      {
        MS5837_OFFSET2 = MS5837_OFFSET2 + 7 * (100 * MS5837_Temperature + 1500) * (100 * MS5837_Temperature + 1500);
        MS5837_SENS2 = MS5837_SENS2 + 4 * (100 * MS5837_Temperature + 1500) * (100 * MS5837_Temperature + 1500);
      }
      // End of second order corrections

      MS5837_Temperature = MS5837_Temperature - MS5837_TT2 / 100;
      MS5837_OFFSET = MS5837_OFFSET - MS5837_OFFSET2;
      MS5837_SENS = MS5837_SENS - MS5837_SENS2;

      MS5837_Pressure = (((MS5837_D1 * MS5837_SENS) / pow(2, 21) - MS5837_OFFSET) / pow(2, 13)) / 10; // Pressure in mbar or hPa

      // Pressure in Pa, density in kg/mm^3,, gravity in m/s^2, i.e., SI units
      float MS5837_depth = (MS5837_Pressure * 100.0f - 101300.0f) / (fluidDensity * 9.80665f);
      z[inc] = int16_t(MS5837_depth * 100) - offset_z;
      Serial.print("Depth :"); Serial.println(MS5837_depth);
      Serial.print("Depth_z :"); Serial.println(z[inc]);




    } // end of wake/sleep motion detection

    /*if (z[inc] < surf_depth) {
      Serial.print("end GPS, depth > threshold");
      break;
      }*/


    // --- GNSS --- //
    //if (GNSS.location(myLocation) && GNSS.satellites(mySatellites) ) {
    if (GNSS.location(myLocation)  ) {

      if ( enable_serialPrint_data_GPS == true ) {
        Serial.print("LOCATION: ");
        Serial.print(fixTypeString[myLocation.fixType()]);
        if ( fixTypeString[myLocation.fixType()] == "NONE") {
          Serial.println(".") ;
        }
      }

      if (myLocation.fixType() != GNSSLocation::TYPE_NONE) {

        GPS_Year   = myLocation.year()    ;
        GPS_Month  = myLocation.month()   ;
        GPS_Day    = myLocation.day()     ;
        GPS_Hour   = myLocation.hours()   ;
        GPS_Minute = myLocation.minutes() ;
        GPS_Second = myLocation.seconds() ;

        // Display date
        if ( enable_serialPrint_data_GPS == true ) {
          Serial.print(fixQualityString[myLocation.fixQuality()]);
          Serial.print(" ");
          Serial.print(myLocation.year());
          Serial.print("/");
          Serial.print(myLocation.month());
          Serial.print("/");
          Serial.print(myLocation.day());
          Serial.print(" ");

          // Display hours
          if (myLocation.hours() <= 9) {
            Serial.print("0");
          }
          Serial.print(myLocation.hours());
          Serial.print(":");

          if (myLocation.minutes() <= 9) {
            Serial.print("0");
          }
          Serial.print(myLocation.minutes());
          Serial.print(":");

          if (myLocation.seconds() <= 9) {
            Serial.print("0");
          }
          Serial.print(myLocation.seconds());


          if (myLocation.leapSeconds() != GNSSLocation::LEAP_SECONDS_UNDEFINED) {
            Serial.print(" ");
            Serial.print(myLocation.leapSeconds());
            if (!myLocation.fullyResolved()) {
              Serial.print("D");
            }
          }

        }

        if (myLocation.fixType() != GNSSLocation::TYPE_TIME) {

          Lat = myLocation.latitude();
          myLocation.latitude(latOut);
          Long = myLocation.longitude();
          myLocation.longitude(longOut);
          Alt = myLocation.altitude();
          EHPE = myLocation.ehpe(); // use this as accuracy figure of merit
          Nb_Satellite = mySatellites.count() ;

          if ( enable_serialPrint_data_GPS == true ) {
            Serial.print(" : ");
            Serial.print(Lat, 7); Serial.print(","); Serial.print(Long, 7);
            Serial.print(",");
            Serial.print(Alt, 3);
            Serial.print(" EHPE :");
            Serial.print(EHPE, 3);
            Serial.print(",");
            Serial.print(myLocation.evpe(), 3);
            Serial.print(" SAT. :");
            Serial.print(myLocation.satellites());
            Serial.print(" DOP :");
            Serial.print(myLocation.hdop(), 2);
            Serial.print(",");
            Serial.print(myLocation.vdop(), 2);
            Serial.print(" - SATELLITES: "); Serial.println(mySatellites.count()) ;

          }



          // if( (myLocation.fixType() != GNSSLocation::TYPE_2D) && (EPE < 10.0f) ) { // 10 is about as low as one should go, 50 is acceptable
          // if( (EPE < 50.0f) ) { // 10 is about as low as one should go, 50 is acceptable
          //   Serial.println("***GNSS go to sleep!***");
          //   GNSS.suspend(); // once we have a good 3D location fix put CAM M8Q to sleep
          //   //break ;
          // }

        } // if (myLocation.fixType() != GNSSLocation::TYPE_TIME)

      } // if (myLocation.fixType() != GNSSLocation::TYPE_NONE)

    } // if (GNSS.location(myLocation))

    if (GNSS.satellites(mySatellites))
    {

      Serial.print("SATELLITES: ");
      Serial.print(mySatellites.count());

      Serial.println();

      for (unsigned int index = 0; index < mySatellites.count(); index++)
      {
        unsigned int svid = mySatellites.svid(index);

        if ((svid >= 1) && (svid <= 32))
        {
          Serial.print("    ");

          if (svid <= 9)
          {
            Serial.print("  G");
            Serial.print(svid);
          }
          else
          {
            Serial.print(" G");
            Serial.print(svid);
          }
        }
        else if ((svid >= 65) && (svid <= 96))
        {
          Serial.print("    ");

          if ((svid - 64) <= 9)
          {
            Serial.print("  R");
            Serial.print(svid - 64);
          }
          else
          {
            Serial.print(" R");
            Serial.print(svid - 64);
          }
        }
        else if ((svid >= 120) && (svid <= 158))
        {
          Serial.print("    ");
          Serial.print("S");
          Serial.print(svid);
        }
        else if ((svid >= 173) && (svid <= 182))
        {
          Serial.print("    ");
          Serial.print("  I");
          Serial.print(svid - 172);
        }
        else if ((svid >= 193) && (svid <= 197))
        {
          Serial.print("    ");
          Serial.print("  Q");
          Serial.print(svid - 192);
        }
        else if ((svid >= 211) && (svid <= 246))
        {
          Serial.print("    ");

          if ((svid - 210) <= 9)
          {
            Serial.print("  E");
            Serial.print(svid - 210);
          }
          else
          {
            Serial.print(" E");
            Serial.print(svid - 210);
          }
        }
        else if (svid == 255)
        {
          Serial.print("    ");
          Serial.print("R???");
        }
        else
        {
          continue;
        }

        Serial.print(": SNR=");
        Serial.print(mySatellites.snr(index));
        Serial.print(", ELEVATION=");
        Serial.print(mySatellites.elevation(index));
        Serial.print(", AZIMUTH=");
        Serial.print(mySatellites.azimuth(index));

        if (mySatellites.unhealthy(index)) {
          Serial.print(", UNHEALTHY");
        }

        if (mySatellites.almanac(index)) {
          Serial.print(", ALMANAC");
        }

        if (mySatellites.ephemeris(index)) {
          Serial.print(", EPHEMERIS");
        }

        if (mySatellites.autonomous(index)) {
          Serial.print(", AUTONOMOUS");
        }

        if (mySatellites.correction(index)) {
          Serial.print(", CORRECTION");
        }

        if (mySatellites.acquired(index)) {
          Serial.print(", ACQUIRED");
        }

        if (mySatellites.locked(index)) {
          Serial.print(", LOCKED");
        }

        if (mySatellites.navigating(index)) {
          Serial.print(", NAVIGATING");
        }

        Serial.println();
      }

    } /* end of GNSS Satellites handling */

    // --- Information about satellites --- //
    // if (GNSS.satellites(mySatellites)){

    //     if( Enable_SerialPrint_Data_GPS == true ){
    //       Serial.print(" - SATELLITES: "); Serial.println(mySatellites.count()) ;
    //       Nb_Satellite = mySatellites.count() ;
    //     }

    // } // if (GNSS.satellites(mySatellites))

    // --- Break Loop --- //

    if ( (EHPE < 10.0f)  ) { // 10 is about as low as one should go, 50 is acceptable
      Serial.println( (String)"Break at " + EHPE);
      new_pos_gps = true;
      break ;
    }

    delay(100);

  } // while( (millis() - now <= 60000)   )

  //samplingDuringGPS.stop();
  GNSS.suspend() ;

  //STM32L0.stop() ;

}


void read_battery_level( bool enable_serialPrint_battery ) {

  VDDA = STM32L0.getVDDA();                                               // Positive Voltage for Analog

  digitalWrite(myVBat_en, HIGH);                                           // Allow check Battery level
  battery_level = 1.27f * VDDA * analogRead(myVBat) / 4096.0f ;  // Determine Battery level
  digitalWrite(myVBat_en, LOW);                                            // Close pin

  if ( enable_serialPrint_battery == true ) {
    Serial.print("Battery Level: ");
    Serial.print(battery_level, 2);
    Serial.println(" V");
  }

}


void printAODR()
{
  switch (AODR)
  {
    case 0x01:
      Serial.println("Data rate  = 1 Hz");
      break;
    case 0x02:
      Serial.println("Data rate  = 10 Hz");
      break;
    case 0x03:
      Serial.println("Data rate  = 25 Hz");
      break;
    case 0x04:
      Serial.println("Data rate  = 50 Hz");
      break;
    case 0x05:
      Serial.println("Data rate  = 100 Hz");
      break;
    default:
      Serial.println("Data rate Error");
      break;
  }
}


void printAscale()
{
  switch (Ascale)
  {
    case 0:
      Serial.println("Full scale = 2 g");
      break;
    case 1:
      Serial.println("Full scale = 4 g");
      break;
    case 2:
      Serial.println("Full scale = 8 g");
      break;
    case 3:
      Serial.println("Full scale = 16 g");
      break;
    default:
      Serial.println("Full scale Error");
      break;
  }
}


void getRTC (uint8_t index)
{
  RTC.getDate(day, month, year);
  RTC.getTime(hours[index], minutes[index], seconds[index], subSeconds[index]);
}

void printRTC (uint8_t index)
{
  //Serial.print(day); Serial.print(":"); Serial.print(month); Serial.print(":20"); Serial.print(year);
  //Serial.print(" ");

  milliseconds = ((subSeconds[index] >> 17) * 1000 + 16384) / 32768;

  if (hours[index] < 10)
  {
    Serial.print("0"); Serial.print(hours[index]);
  }
  else
    Serial.print(hours[index]);

  Serial.print(":");
  if (minutes[index] < 10)
  {
    Serial.print("0"); Serial.print(minutes[index]);
  }
  else
    Serial.print(minutes[index]);

  Serial.print(":");
  if (seconds[index] < 10)
  {
    Serial.print("0"); Serial.print(seconds[index]);
  }
  else
    Serial.print(seconds[index]);

  Serial.print(".");
  if (milliseconds <= 9)
  {
    Serial.print("0");
  }
  if (milliseconds <= 99)
  {
    Serial.print("0");
  }
  Serial.print(milliseconds);
  Serial.println(" ");
}


void SetDefaultRTC()   // Function sets the RTC to the FW build date-time...
{
  char Build_mo[3];
  String build_mo = "";

  Build_mo[0] = build_date[0];    // Convert month string to integer
  Build_mo[1] = build_date[1];
  Build_mo[2] = build_date[2];
  for (uint8_t i = 0; i < 3; i++)
  {
    build_mo += Build_mo[i];
  }
  if (build_mo == "Jan")
  {
    month = 1;
  } else if (build_mo == "Feb")
  {
    month = 2;
  } else if (build_mo == "Mar")
  {
    month = 3;
  } else if (build_mo == "Apr")
  {
    month = 4;
  } else if (build_mo == "May")
  {
    month = 5;
  } else if (build_mo == "Jun")
  {
    month = 6;
  } else if (build_mo == "Jul")
  {
    month = 7;
  } else if (build_mo == "Aug")
  {
    month = 8;
  } else if (build_mo == "Sep")
  {
    month = 9;
  } else if (build_mo == "Oct")
  {
    month = 10;
  } else if (build_mo == "Nov")
  {
    month = 11;
  } else if (build_mo == "Dec")
  {
    month = 12;
  } else
  {
    month = 1;                                                                                       // Default to January if something goes wrong...
  }
  if (build_date[4] != 32)                                                                           // If the first digit of the date string is not a space
  {
    day   = (build_date[4] - 48) * 10 + build_date[5]  - 48;                                         // Convert ASCII strings to integers; ASCII "0" = 48
  } else
  {
    day   = build_date[5]  - 48;
  }
  year    = (build_date[9] - 48) * 10 + build_date[10] - 48;
  hours[0]   = (build_time[0] - 48) * 10 + build_time[1]  - 48;
  minutes[0] = (build_time[3] - 48) * 10 + build_time[4]  - 48;
  seconds[0] = (build_time[6] - 48) * 10 + build_time[7]  - 48;
  RTC.setDay(day);                                                                                   // Set the date/time
  RTC.setMonth(month);
  RTC.setYear(year);
  RTC.setHours(hours[0]);
  RTC.setMinutes(minutes[0]);
  RTC.setSeconds(seconds[0]);
}





void fct_end_dive()
{
  if (start_dive_flag == 1 && z[inc] < 25) {
    etho[inc_eth][3] = inc;
    start_dive_flag = 0;
    end_dive_flag = 1;
  }
}


void etho_fct()
{
  int nb_sample_end;

  if (inc > 30)
  {
    start_dive_flag = 1;
  }


  if (inc_eth == 0) // Starting behavior of a dive
  {
    if (z[inc] < 1)
    {
      etho[inc_eth][1] = 3;  // Down phase
      etho[inc_eth][2] = 1;
      inc_eth++;
    } else {
      etho[inc_eth][1] = 5;  // sub_surface phase
      etho[inc_eth][2] = 1;
      inc_eth++;
    }
  }

  //Starting behavior
  //UP : Test if the dive slope is inforior to a fixed value, we use flag_e4
  if (depth_slope < slope_up)
  {
    flag_s[3][1]++;
    flag_s[3][2] = inc;

  } else {
    flag_s[3][1] = 0;
  }


  //DOWN :Test if the dive slope is inforior to a fixed value, we use flag_e4
  if (depth_slope > slope_down)
  {
    flag_s[4][1]++;
    flag_s[4][2] = inc;
  } else {
    flag_s[4][1] = 0;
  }

  //REST : Test if ODBA inferior to a threshold. We can slope also
  if (VeDBA < VeDBA_rest)
  {
    flag_s[2][1]++;
    flag_s[2][2] = inc;
  } else {
    flag_s[2][1] = 0;
  }

  //SURFACE : Test if depth is inferior to a threshold
  if (z[inc] < surf_depth)
  {
    flag_s[5][1]++;
    flag_s[5][2] = inc;
  } else {
    flag_s[5][1] = 0;
  }

  //Ending behavior
  //DOWN :Test if the dive slope is inforior to a fixed value, we use flag_e4
  if (depth_slope < slope_down  && etho[inc_eth][1] == 4)
  {
    flag_e[4][1]++;
    flag_e[4][2] = inc;
  } else {
    flag_e[4][1] = 0;
  }

  //UP : Test if the dive slope is inforior to a fixed value, we use flag_e4
  if (depth_slope < slope_down  && etho[inc_eth][1] == 3)
  {
    flag_e[3][1]++;
    flag_e[3][2] = inc;
  } else {
    flag_e[3][1] = 0;
  }

  //REST : Test if ODBA inferior to a threshold. We can slope also
  if (VeDBA > VeDBA_rest && etho[inc_eth][1] == 2)
  {
    flag_e[2][1]++;
    flag_e[2][2] = inc;
  } else {
    flag_e[2][1] = 0;
  }

  // Change behavior section

  //DOWN phase -> UP, REST
  if (etho[inc_eth][1] == 4)
  {
    if (flag_s[3][1] >= t_d2u) //UP : Compare flag number and sampling (in nb of sample)
    {
      nb_sample_end = flag_s[3][2] - flag_s[3][1];
      etho[inc_eth][3] = nb_sample_end - 1; // End of the behavior
      inc_eth++;
      etho[inc_eth][1] = 3;                 // Start UP behavior
      etho[inc_eth][2] = nb_sample_end;     // Start time of the new behavior
      memset(flag_s, 0, sizeof(flag_s));
      memset(flag_e, 0, sizeof(flag_e));
    } else if ( flag_s[2][1] >= t_d2r)
    {
      nb_sample_end = flag_s[2][2] - flag_s[2][1];
      etho[inc_eth][3] = nb_sample_end - 1; // End of the behavior
      inc_eth++;
      etho[inc_eth][1] = 2;                 // Start UP behavior
      etho[inc_eth][2] = nb_sample_end;     // Start time of the new behavior
      memset(flag_s, 0, sizeof(flag_s));
      memset(flag_e, 0, sizeof(flag_e));

    }
  }

  //Rest phase -> UP, DOWN
  if (etho[inc_eth][1] == 2)
  {
    if (flag_s[3][1] >= t_r2u) //UP : Compare flag number and sampling (in nb of sample)
    {
      nb_sample_end = flag_s[3][2] - flag_s[3][1];
      etho[inc_eth][3] = nb_sample_end - 1; // End of the behavior
      inc_eth++;
      etho[inc_eth][1] = 3;                 // Start UP behavior
      etho[inc_eth][2] = nb_sample_end;     // Start time of the new behavior
      memset(flag_s, 0, sizeof(flag_s));
      memset(flag_e, 0, sizeof(flag_e));
    } else if ( flag_s[4][1] >= t_r2d)
    {
      nb_sample_end = flag_s[4][2] - flag_s[4][1];
      etho[inc_eth][3] = nb_sample_end - 1; // End of the behavior
      inc_eth++;
      etho[inc_eth][1] = 4;                 // Start DOWN behavior
      etho[inc_eth][2] = nb_sample_end;     // Start time of the new behavior
      memset(flag_s, 0, sizeof(flag_s));
      memset(flag_e, 0, sizeof(flag_e));

    }
  }

  //UP phase -> SURFACE
  if (etho[inc_eth][1] == 3)
  {
    if (flag_s[5][1] >= t_u2s) //UP : Compare flag number and sampling (in nb of sample)
    {
      nb_sample_end = flag_s[5][2] - flag_s[5][1];
      etho[inc_eth][3] = nb_sample_end - 1; // End of the behavior
      inc_eth++;
      etho[inc_eth][1] = 5;                 // Start Surface behavior
      etho[inc_eth][2] = nb_sample_end;     // Start time of the new behavior
      memset(flag_s, 0, sizeof(flag_s));
      memset(flag_e, 0, sizeof(flag_e));
    }
  }

  //Surface phase -> DOWN (If the dive crop is well done nothing after surface)
  if (etho[inc_eth][1] == 5)
  {
    if (flag_s[4][1] >= t_s2d) //UP : Compare flag number and sampling (in nb of sample)
    {
      nb_sample_end = flag_s[4][2] - flag_s[4][1];
      etho[inc_eth][3] = nb_sample_end - 1; // End of the behavior
      inc_eth++;
      etho[inc_eth][1] = 4;                 // Start Down behavior
      etho[inc_eth][2] = nb_sample_end;     // Start time of the new behavior
      memset(flag_s, 0, sizeof(flag_s));
      memset(flag_e, 0, sizeof(flag_e));
    }
  }

  //SWIM phase -> DOWN, UP, REST
  if (etho[inc_eth][1] == 1)
  {
    if (flag_s[2][1] >= t_s2r) // REST: Compare flag number and sampling (in nb of sample)
    {
      nb_sample_end = flag_s[2][2] - flag_s[2][1];
      etho[inc_eth][3] = nb_sample_end - 1; // End of the behavior
      inc_eth++;
      etho[inc_eth][1] = 2;                 // Start REST behavior
      etho[inc_eth][2] = nb_sample_end;     // Start time of the new behavior
      memset(flag_s, 0, sizeof(flag_s));
      memset(flag_e, 0, sizeof(flag_e));
    } else if ( flag_s[3][1] >= t_s2u) //UP : compare flag number and sampling (in nb of sample)
    {
      nb_sample_end = flag_s[3][2] - flag_s[3][1];
      etho[inc_eth][3] = nb_sample_end - 1; // End of the behavior
      inc_eth++;
      etho[inc_eth][1] = 3;                 // Start UP behavior
      etho[inc_eth][2] = nb_sample_end;     // Start time of the new behavior
      memset(flag_s, 0, sizeof(flag_s));
      memset(flag_e, 0, sizeof(flag_e));

    }
    else if ( flag_s[4][1] >= t_s2d) //DOWN : compare flag number and sampling (in nb of sample)
    {
      nb_sample_end = flag_s[4][2] - flag_s[4][1];
      etho[inc_eth][3] = nb_sample_end - 1; // End of the behavior
      inc_eth++;
      etho[inc_eth][1] = 4;                 // Start DOWN behavior
      etho[inc_eth][2] = nb_sample_end;     // Start time of the new behavior
      memset(flag_s, 0, sizeof(flag_s));
      memset(flag_e, 0, sizeof(flag_e));
    }
  }

  //SWIM : Swimming phase is hard to describe so we use the end time of the other phase
  //We use end flag of other behavior to go in SWIM phase
  if (flag_e[4][1] >= t_d2s) //If the time of end flag for up phase is superior to the threshold
  {
    nb_sample_end = flag_e[4][2] - flag_e[4][2];
    etho[inc_eth][3] = nb_sample_end - 1; // End of the behavior
    inc_eth++;
    etho[inc_eth][1] = 1;                 // Start SWIM behavior
    etho[inc_eth][2] = nb_sample_end;     // Start time of the new behavior
    memset(flag_s, 0, sizeof(flag_s));
    memset(flag_e, 0, sizeof(flag_e));
  } else if (flag_e[5][1] >= t_surf2s) //If the time of end flag for up phase is superior to the threshold
  {
    nb_sample_end = flag_e[5][2] - flag_e[5][2];
    etho[inc_eth][3] = nb_sample_end - 1; // End of the behavior
    inc_eth++;
    etho[inc_eth][1] = 1;                 // Start SWIM behavior
    etho[inc_eth][2] = nb_sample_end;     // Start time of the new behavior
    memset(flag_s, 0, sizeof(flag_s));
    memset(flag_e, 0, sizeof(flag_e));
  } else if (flag_e[3][1] >= t_u2s) //If the time of end flag for up phase is superior to the threshold
  {
    nb_sample_end = flag_e[3][2] - flag_e[3][2];
    etho[inc_eth][3] = nb_sample_end - 1; // End of the behavior
    inc_eth++;
    etho[inc_eth][1] = 1;                 // Start SWIM behavior
    etho[inc_eth][2] = nb_sample_end;     // Start time of the new behavior
    memset(flag_s, 0, sizeof(flag_s));
    memset(flag_e, 0, sizeof(flag_e));
  } else if (flag_e[2][1] >= t_r2s) //If the time of end flag for up phase is superior to the threshold
  {
    nb_sample_end = flag_e[2][2] - flag_e[2][2];
    etho[inc_eth][3] = nb_sample_end - 1; // End of the behavior
    inc_eth++;
    etho[inc_eth][1] = 1;                 // Start SWIM behavior
    etho[inc_eth][2] = nb_sample_end;     // Start time of the new behavior
    memset(flag_s, 0, sizeof(flag_s));
    memset(flag_e, 0, sizeof(flag_e));
  }
}



void speed_fct_F4()
{
  h_speed = (0.0829 + 8.6612 * VeDBA_f) * 100;
}


void speed_fct_F2(int16_t behavior) {
  // speed calculation with fct 1

  if (behavior == 1) {
    h_speed = 83; //0.69; //1.11; //0.69;
    //speed_h[inc] = 145;

    if (behavior == 5) {
      h_speed = 69; //0.69; //1.11; //0.69;
      //speed_h[inc] = 145;
    } else
    {
      h_speed = 0; //0;
      //speed_h[inc] = 0;
    }
  }
}


void speed_fct_REG(int16_t adx) {

  //sign = last_sign;

  h_speed = (0.3026 + 4.1525 * adx + 0.0040 * adx + adx * adx) / 1000;

  /*
    if (buf_accx > stroke_max){
    stroke_max = buf_accx;
    }

    if (buf_accx < stroke_min){
    stroke_min = buf_accx;
    }

    if (buf_accx >= 0){
    sign = 1;
    } else {
    sign = -1;
    }


    if (sign == 1 && last_sign != sign){
    etho_2_reg[inc_eth_2][1] = inc;
    }

    if (sign == -1 && last_sign != sign){
    etho_2_reg[inc_eth_2][2] = inc;
    etho_2_reg[inc_eth_2][3] = stroke_min;
    etho_2_reg[inc_eth_2][4] = stroke_max;
    etho_2_reg[inc_eth_2][5] = stroke_max-stroke_min;
    stroke_min = 0;
    stroke_max = 0;

    inc_eth_2++;
    flag_zero_crossing = 1;
    }


    if (flag_zero_crossing == 1){
    flag_zero_crossing = 0;

    if (etho_2_reg[inc_eth_2][4] < 0.05 && etho_2_reg[inc_eth_2][4] > 0.005 && etho_2_reg[inc_eth_2][3] < -0.005 && etho_2_reg[inc_eth_2][4] > 0.05)
    {
    inc_regular++;
    max_inc_regular = inc_regular;
    } else {
    inc_regular = 0;
    inc_eth_2 = 0;
    }

    if (inc_regular >= 5){
    flag_start_regular = 1;
    }

    if (flag_start_regular && inc_regular == 0){ // When we were in regular phase and stop the phase we calculate the speed

    flag_start_regular = 0;

    for (int ii =  0; ii < max_inc_regular; ii++){

    h_speed = 0.3026 + 4.1525 * etho_2_reg[ii-1][5] + 0.0040 * etho_2_reg[ii-1][4] + etho_2_reg[inc_eth_2-1][4]*etho_2_reg[inc_eth_2-1][5]

    for(int jj =  etho_2_reg[ii][1]; jj < etho_2_reg[ii][2]; jj++){

       buf_2[jj] = h_speed;

    }

    }

    max_inc_regular = 0;

    }

    }*/

}

void speed_fct_OCDR(int16_t f_depth, int16_t f_last_depth, float pitch) {

  int16_t diff_d;

  diff_d = abs(f_last_depth - f_depth) * 1; // With fs = 1;
  h_speed = (diff_d / tan(pitch)) * 100;

}


void speed_fct_F3(float f_depth, float f_last_depth, float pitch) {

  // speed calculation with fct 1
  float v_speed = 0;
  float diff_d;

  diff_d = abs(f_last_depth - f_depth) * 1; // With fs = 1;
  v_speed = diff_d / sin(pitch);

  h_speed = (sqrt(max(h_speed * h_speed - v_speed * v_speed, 0))) * 100;

}


void htrack_fct(int first_data_b, int last_data_b, bool debug)
{
  float x_buf;
  float y_buf;

  for (int i = first_data_b; i <= last_data_b; i++)
  {

    x_buf = /*float(x[INC - 1]) +*/ h_speed * cos(float(heading_array[i]) / 10000);
    y_buf = /*float(y[INC - 1]) +*/ h_speed * sin(float(heading_array[i]) / 10000);

    float d = sqrt(  pow(x_buf, 2) + pow( y_buf, 2));
    d_t = d_t + d; // Total distance travelled

    buf_1[i] = buf_1[i - 1] + int16_t(x_buf * 100);
    buf_2[i] = buf_2[i - 1] + int16_t(y_buf * 100);

    if (debug) {
      Serial.print(" yaw : ");  Serial.println(float(heading_array[i]) / 10000);
      Serial.print("sin yaw : ");  Serial.println(sin(float(heading_array[i]) / 10000));
      Serial.print("x_buf : ");  Serial.println(x_buf);
      Serial.print("y_buf : ");  Serial.println(y_buf);
      Serial.print("buf_1 : ");  Serial.print(buf_1[i]);
      Serial.print("buf_2 : ");  Serial.println(buf_2[i]);
    }
  }
}
