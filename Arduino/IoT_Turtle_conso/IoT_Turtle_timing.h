/* IOT Turtle / Accéléromètre Magnétomètre datalogger
   CMWX1ZZABZ (STM32L082 and SX1276)
   LSM303AGR
   MX25R6435FZAI 8 MByte SPI NOR flash memory

   LIRMM / LDK 11/2020 V3.0 (IOT)
*/

#include <STM32L0.h>
#include <RTC.h>
#include "LSM303AGR_Acc.h"
#include "LSM303AGR_Mag.h"
#include "SPIFlash.h"
#include "I2CDev.h"

//---------------------------- DATALOG PARAMETERS ----------------------------------------------------------------------------------------------------------------
#define LogDelay 0      //temporisation avant début d'enregistrement en minutes. Zéro -> pas de tempo.
#define LogTime 1150    //durée de log en minutes (inactif si > capacité mémoire. Maxi environ 2h à 100Hz, 4h à 50 Hz, 20h à 10 Hz,). 1150 -> full memory à 10, 50 et 100 Hz.
//----------------------------------------------------------------------------------------------------------------------------------------------------------------

// SPI Flash: 64 MBit (8 MByte) SPI Flash 32,768, 256-byte pages
//page 0 reserved, accelero values start at page 1
uint16_t  first_log_page = 0x01;   // higher it if you want to reserve some pages for other use...
uint16_t  last_log_page = 0x7FFF;  // lower it if you want to reserve some pages for other use...
uint32_t  force_last_page;
uint16_t  page_number;
uint16_t  last_page;
uint16_t  invalid_page;
uint8_t   flashPage[256];          // array to hold the data for flash page write
uint8_t   AccValue_number;         // compteur de triplets Acc dans une page (max 21 = 126 octets)
uint8_t   MagValue_number;         // compteur de triplets Mag dans une page (max 21 = 126 octets)
uint16_t  i;

#define csPin 25 // SPI Flash chip select pin
SPIFlash SPIFlash(csPin);

#define I2C_BUS    Wire               // Define the I2C bus (Wire instance) you wish to use
I2Cdev             i2c_0(&I2C_BUS);   // Instantiate the I2Cdev object and point to the desired I2C bus

const char        *build_date = __DATE__;   // 11 characters MMM DD YYYY
const char        *build_time = __TIME__;   // 8 characters HH:MM:SS

// pin assignments
#define myLed     10 // blue led 
#define myVBat_en  2 // enable VBat read
#define myVBat    A1 // VBat analog read pin

uint32_t UID[3] = {0, 0, 0};

bool SerialDebug = true;

uint8_t hours[4] = {12, 12, 12, 12}, minutes[4] = {0, 0, 0, 0}, seconds[4] = {0, 0, 0, 0}, year = 1, month = 1, day = 1;
uint32_t subSeconds[4], milliseconds;

// battery voltage monitor definitions
float VDDA, VBAT, VBUS, STM32L0Temp;

//LSM303AGR_Acc definitions
#define LSM303AGR_Acc_intPin1 A4  // interrupt1 pin definitions, data ready

union {             // scale resolutions per LSB for the accel sensor
  float fval;
  byte bval[4];
} aRes;

union {           // offset biases
  float fval;
  byte bval[4];
} accelBias[3];

float buf_acc[1200] = {0};
float param_accelBias[3] = {0.0f, 0.0f, 0.0f}; // offset biases copy for fonction call
int16_t accelData[3], accTempData;  // Stores the 10-bit signed accel output if normal mode
float ax, ay, az;                   // variables to hold latest accel data values
uint8_t Ascale, AODR, AODR_rec;
volatile bool newLSM303AGR_AccData = false; // used for data ready interrupt handling

LSM303AGR_Acc LSM303AGR_Acc(&i2c_0); // instantiate LSM303AGR_Acc class

//LSM303AGR_Mag definitions
int16_t magData[3];
float bx, by, bz;                   // variables to hold latest accel data values

union {           // offset biases
  float fval;
  byte bval[4];
} magBias[3];

float param_magBias[3] = {0.0f, 0.0f, 0.0f};
LSM303AGR_Mag LSM303AGR_Mag(&i2c_0); // instantiate LSM303AGR_Mag class

//**************************************************************************************************************************

void setup()
{
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, LOW);  // start with blue led on (since active LOW)

  // Set the RTC time to firmware build time
  //SetDefaultRTC();

  Serial.begin(115200);
  delay(2000);
  Serial.println("\r\n--------------------------------------------------------------------");
  Serial.print("-----------------    CONFINED LIRMM      11/2020    ----------------\r\n");
  Serial.println("--------------------------------------------------------------------\r\n");

  STM32L0.getUID(UID);
  Serial.print("STM32L0 MCU UID = 0x"); Serial.print(UID[0], HEX); Serial.print(UID[1], HEX); Serial.println(UID[2], HEX);

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

  //SPI FLASH
  pinMode(csPin, OUTPUT); // set SPI chip select as L082 output
  digitalWrite(csPin, HIGH);

  // check SPI Flash ID
  SPIFlash.init();      // start SPI (include SPI.begin)
  SPIFlash.powerUp();   // MX25R6435FZAI defaults to power down state
  SPIFlash.getChipID(); // Verify SPI flash communication

  // recherche dernière page de flash utilisée
  findLastPage();

  // AODR, Ascale in SpiFlash page 0
  SPIFlash.flash_read_pages(flashPage, 0, 1);
  AODR   = flashPage[18];
  Ascale = flashPage[19];

  switch (AODR)
  {
    case 0x02:
      AODR_rec = 10;
      break;
    case 0x04:
      AODR_rec = 50;
      break;
    case 0x05:
      AODR_rec = 100;
      break;
  }

  for (i = 1; i <= 3600; i++) {
    buf_acc[i] = 0;
  }

  //si usb connecté dump SpiFlash, sinon log.
  if (VBUS ==  1)
    //if (0)
  {
    delay(2000);
    dumpFlash();
    Serial.println("\r\nYou can now safely poweroff the board.\r\n");
    // Pour le dump on s'arrête là. Si USB + batterie, charge ok.
    digitalWrite(myLed, HIGH);
    SPIFlash.powerDown();  // Put SPI flash into power down mode
    SPI.end();             // End SPI peripheral to save power in STOP mode
    STM32L0.stop();        // Enter STOP mode and wait for an interrupt
    while (1);         // par sécurité
  }

  //tempo avant début d'enregistrement
  if (LogDelay > 0)
  {
    Serial.print("\r\nWaiting "); Serial.print(LogDelay); Serial.print(" mn before starting DataLog...\r\n\n");
    digitalWrite(myLed, HIGH);
    delay(200);
    digitalWrite(myLed, LOW);
    delay(500);
    digitalWrite(myLed, HIGH);
    delay(200);
    digitalWrite(myLed, LOW);
    delay(500);
    digitalWrite(myLed, HIGH);
    delay(200);
    digitalWrite(myLed, LOW);
    delay(500);
    digitalWrite(myLed, HIGH);
    delay(LogDelay * 60 * 1000);
  }

  // calcul last page si limite demandée
  force_last_page = ((uint32_t)AODR_rec * (uint32_t)LogTime * 60) / 21 + first_log_page;
  if (force_last_page > first_log_page)
  {
    if (force_last_page > last_log_page)
    {
      force_last_page = last_log_page;
    }
  }

  if (last_page >= (uint16_t)force_last_page) // la limite de flash est atteinte
  {
    Serial.print("\r\nMEMORY IS FULL !!!\r\n");

    digitalWrite(myLed, HIGH); // turn off led
    SPIFlash.powerDown();  // Put SPI flash into power down mode
    SPI.end();             // End SPI peripheral to save power in STOP mode
    STM32L0.stop();        // Enter STOP mode and wait for an interrupt
    while (1);  // par sécurité
  }

  //setup for start of log
  if (last_page >= first_log_page) // reprise après poweroff, on force une page à 0 sauf si 1er run
  {
    for (i = 0; i < 256; i++)
    {
      flashPage[i] = 0x00;
    }
    SPIFlash.flash_page_program(flashPage, ++last_page);  // Temps d'écriture mesuré : 1 ms
  }

  page_number = ++last_page;      // set first value page for main loop
  AccValue_number = 0;            // reset first value number for main loop
  MagValue_number = 0;            // reset first value number for main loop

  // set last allowed page for log
  if (force_last_page < last_log_page)  // limite par nombre de minutes de log
  {
    last_page = force_last_page;
  }
  else
  {
    last_page = last_log_page;
  }


  printAscale();
  printAODR();

  //init I2C
  I2C_BUS.begin();                                      // Set master mode
  delay(1000);
  I2C_BUS.setClock(400000);                             // I2C frequency at 400 kHz
  delay(1000);

  // set up Accelero
  pinMode(LSM303AGR_Acc_intPin1, INPUT);
  LSM303AGR_Acc.reset();
  LSM303AGR_Acc.init(Ascale, AODR);

  // set up Magnéto
  LSM303AGR_Mag.reset();
  LSM303AGR_Mag.init(AODR);

  delay(100); //stabilisation mesures

  Serial.print("\r\nDataLog running...");

  digitalWrite(myLed, HIGH); // turn off led when configuration successfully completed

  attachInterrupt(LSM303AGR_Acc_intPin1, myinthandler1, RISING);  // define data ready interrupt for intPin1 output of LSM303AGR_Acc
  LSM303AGR_Acc.readAccData(accelData); // read data register to clear interrupt before main loop

  LSM303AGR_Mag.readMagData(magData); // read data register to clear status register

  getRTC(0); //timestamp
} /* end of setup */

//******************************************************************************************************************************************
/*

   Everything in the main loop is based on interrupts, so that
   if there has not been an interrupt event the STM32L082 should be in STOP mode
*/

// 32,768 256-byte pages in a 8 MByte flash
// 21 mesures accelero + magnéto -> 252 octets -> ~ 1 page
//store RAW accelData (3 x 2 octets) + RAW magData (3 x 2 octets)
//mise en forme + bias au dump

void loop()
{
  // When exiting STOP mode, re-enable the SPI peripheral
  SPIFlash.init();      // start SPI (include SPI.begin)
  SPIFlash.powerUp();   // MX25R6435FZAI defaults to power down state

  if (newLSM303AGR_AccData == true) // on interrupt, read data
  {
    digitalWrite(myLed, HIGH);    // turn off led
    newLSM303AGR_AccData = false;     // reset newData flag

    LSM303AGR_Acc.readAccData(accelData); // INT1 cleared on any read

    flashPage[ AccValue_number * 12 + 0]  = (accelData[0] & 0xFF00) >> 8;
    flashPage[ AccValue_number * 12 + 1]  = (accelData[0] & 0x00FF);
    flashPage[ AccValue_number * 12 + 2]  = (accelData[1] & 0xFF00) >> 8;
    flashPage[ AccValue_number * 12 + 3]  = (accelData[1] & 0x00FF);
    flashPage[ AccValue_number * 12 + 4]  = (accelData[2] & 0xFF00) >> 8;
    flashPage[ AccValue_number * 12 + 5]  = (accelData[2] & 0x00FF);

    AccValue_number++;

    //Serial.print(" AV"); Serial.print(AccValue_number);

    byte c = 0;
    while (!((c >> 3) & 0x01))  // attente data mag
    {
      c = LSM303AGR_Mag.getStatus();
    }

    LSM303AGR_Mag.readMagData(magData);

    flashPage[ MagValue_number * 12 + 6]  = (magData[0] & 0xFF00) >> 8;
    flashPage[ MagValue_number * 12 + 7]  = (magData[0] & 0x00FF);
    flashPage[ MagValue_number * 12 + 8]  = (magData[1] & 0xFF00) >> 8;
    flashPage[ MagValue_number * 12 + 9]  = (magData[1] & 0x00FF);
    flashPage[ MagValue_number * 12 + 10]  = (magData[2] & 0xFF00) >> 8;
    flashPage[ MagValue_number * 12 + 11]  = (magData[2] & 0x00FF);

    MagValue_number++;

    //Serial.print(" MV"); Serial.print(MagValue_number);
  }

  if (AccValue_number == 21) // on écrit la page
  {
    flashPage[255] = 0x00;  // on force le dernier octet de la page à zéro (flag page utilisée).

    digitalWrite(myLed, LOW); //led on
    SPIFlash.flash_page_program(flashPage, page_number);  // Temps d'écriture mesuré : 1 ms
    AccValue_number = 0;
    MagValue_number = 0;
    page_number++;
  }

  if (page_number > last_page) // fin de log
  {
    getRTC(1); //timestamp

    detachInterrupt(LSM303AGR_Acc_intPin1);  // detach interrupt to stay in stop mode
    LSM303AGR_Acc.reset();
    LSM303AGR_Mag.reset();

    Serial.print("... and stopped.\r\n");

    Serial.print("\r\nLast sequence start : ");
    printRTC(0); //timestamp
    Serial.print("Last sequence stop  : ");
    printRTC(1); //timestamp

    digitalWrite(myLed, HIGH); // turn off led
  }

  SPIFlash.powerDown();  // Put SPI flash into power down mode
  SPI.end();             // End SPI peripheral to save power in STOP mode
  STM32L0.stop();        // Enter STOP mode and wait for an interrupt
}  /* end of loop*/

//***************************************************************************************************************************

void myinthandler1()
{
  newLSM303AGR_AccData = true;
  STM32L0.wakeup();
}

void findLastPage()
{
  uint16_t low, high, test;

  low = first_log_page - 1; //première page de log - 1
  high = last_log_page;

  while ((high - low) != 1) // pas 2 pages consécutives free/used
  {
    test = (high - low) / 2 + low;

    SPIFlash.flash_read_pages(flashPage, test, 1);

    if (flashPage[255] == 0x00) //used page
    {
      low = test;
    }
    else  // on test la page suivante (if faut 2 pages libres consécutives)
    {
      SPIFlash.flash_read_pages(flashPage, test + 1, 1);

      if (flashPage[255] == 0x00) //used page
      {
        low = test + 1;
      }
      else
      {
        high = test;
      }
    }
  }

  last_page = low;

  //cas particulier dernière page (non testé dans la boucle)
  if (last_page == (last_log_page - 1))
  {
    SPIFlash.flash_read_pages(flashPage, last_log_page, 1);

    if (flashPage[255] == 0x00) //used page
    {
      last_page++;
    }
  }
}

void dumpFlash()
{
  // Read aRes and biases in SpiFlash page 0
  SPIFlash.flash_read_pages(flashPage, 0, 1);

  aRes.bval[0] = flashPage[2];
  aRes.bval[1] = flashPage[3];
  aRes.bval[2] = flashPage[4];
  aRes.bval[3] = flashPage[5];

  accelBias[0].bval[0] = flashPage[6];
  accelBias[0].bval[1] = flashPage[7];
  accelBias[0].bval[2] = flashPage[8];
  accelBias[0].bval[3] = flashPage[9];

  accelBias[1].bval[0] = flashPage[10];
  accelBias[1].bval[1] = flashPage[11];
  accelBias[1].bval[2] = flashPage[12];
  accelBias[1].bval[3] = flashPage[13];

  accelBias[2].bval[0] = flashPage[14];
  accelBias[2].bval[1] = flashPage[15];
  accelBias[2].bval[2] = flashPage[16];
  accelBias[2].bval[3] = flashPage[17];

  AODR   = flashPage[18];
  Ascale = flashPage[19];

  magBias[0].bval[0] = flashPage[20];
  magBias[0].bval[1] = flashPage[21];
  magBias[0].bval[2] = flashPage[22];
  magBias[0].bval[3] = flashPage[23];

  magBias[1].bval[0] = flashPage[24];
  magBias[1].bval[1] = flashPage[25];
  magBias[1].bval[2] = flashPage[26];
  magBias[1].bval[3] = flashPage[27];

  magBias[2].bval[0] = flashPage[28];
  magBias[2].bval[1] = flashPage[29];
  magBias[2].bval[2] = flashPage[30];
  magBias[2].bval[3] = flashPage[31];

  Serial.println("\r\nLSM303AGR setup : ");

  printAscale();
  printAODR();

  if (last_page < first_log_page)
  {
    Serial.println("\r\nNO DATA AVAILABLE !!!\r\n");
  }
  else
  {
    Serial.println("\r\nMemory dump start in 5s...\r\n");
    delay(5000);

    invalid_page = 0;

    Serial.print("\r\n AccX ; AccY ; AccZ ; MagX ; MagY ; MagZ    (Acc = mg, Mag = mG)\r\n\n");

    getRTC(0); //timestamp

    for (page_number = first_log_page; page_number < last_page; page_number++)
    {
      digitalWrite(myLed, LOW); delay(1); digitalWrite(myLed, HIGH);
      SPIFlash.flash_read_pages(flashPage, page_number, 1);

      // si triplets nuls (impossible) ou page invalide
      if (((flashPage[0] == 0x00) \
           && (flashPage[1] == 0x00) \
           && (flashPage[2] == 0x00) \
           && (flashPage[3] == 0x00) \
           && (flashPage[4] == 0x00) \
           && (flashPage[5] == 0x00) \
           && (flashPage[6] == 0x00) \
           && (flashPage[7] == 0x00) \
           && (flashPage[8] == 0x00) \
           && (flashPage[9] == 0x00) \
           && (flashPage[10] == 0x00) \
           && (flashPage[11] == 0x00)) \
          || (flashPage[255] == 0xFF))
      {
        Serial.print("0;0;0;0;0;0\r\n");
        invalid_page ++;
      }
      else
      {
        for (AccValue_number = 0; AccValue_number < 21; AccValue_number++)
        {
          // reconstruct accelero values
          accelData[0] = (int16_t)((flashPage[AccValue_number * 12 + 0]) << 8);
          accelData[0] |= (int16_t)(flashPage[AccValue_number * 12 + 1]);
          accelData[1] = (int16_t)((flashPage[AccValue_number * 12 + 2]) << 8);
          accelData[1] |= (int16_t)(flashPage[AccValue_number * 12 + 3]);
          accelData[2] = (int16_t)((flashPage[AccValue_number * 12 + 4]) << 8);
          accelData[2] |= (int16_t)(flashPage[AccValue_number * 12 + 5]);

          ax = (float)accelData[0] * aRes.fval - accelBias[0].fval; // get actual g value, this depends on scale being set
          ay = (float)accelData[1] * aRes.fval - accelBias[1].fval;
          az = (float)accelData[2] * aRes.fval - accelBias[2].fval;

          // reconstruct magnéto values
          magData[0] = (int16_t)((flashPage[AccValue_number * 12 + 6]) << 8);
          magData[0] |= (int16_t)(flashPage[AccValue_number * 12 + 7]);
          magData[1] = (int16_t)((flashPage[AccValue_number * 12 + 8]) << 8);
          magData[1] |= (int16_t)(flashPage[AccValue_number * 12 + 9]);
          magData[2] = (int16_t)((flashPage[AccValue_number * 12 + 10]) << 8);
          magData[2] |= (int16_t)(flashPage[AccValue_number * 12 + 11]);

          bx = (float)magData[0] * 1.5f - magBias[0].fval;  // get actual mG (milligauss) value.
          by = (float)magData[1] * 1.5f - magBias[1].fval;
          bz = (float)magData[2] * 1.5f - magBias[2].fval;

          Serial.print((int)1000 * ax); Serial.print(";");
          Serial.print((int)1000 * ay); Serial.print(";");
          Serial.print((int)1000 * az); Serial.print(";");
          Serial.print(bx); Serial.print(";");
          Serial.print(by); Serial.print(";");
          Serial.print(bz);
          Serial.print("\r\n");
        }
      }
    }

    getRTC(1); //timestamp

    Serial.print("\r\nTotal : "); Serial.print(((uint32_t)page_number - first_log_page - invalid_page) * 21); Serial.print(" Measurements");
    if (invalid_page > 0)
    {
      Serial.print(" in "); Serial.print(invalid_page + 1); Serial.print(" sequences");
    }

    Serial.print("\r\n\r\nDump start : ");
    printRTC(0); //timestamp
    Serial.print("Dump end   : ");
    printRTC(1); //timestamp
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
