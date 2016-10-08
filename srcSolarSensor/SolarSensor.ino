/*
Kod do senzoru na baterkku co se budi a posila svoje napeti baterky a vlhkost a dalsi pres NRF24 do bazove stanice
*/

/*
  Moisture Circuit:
  To connect two nails and a 10 KOhms resistor as shown:

        digital 2---*
                  |
                  \
                  /
                  \ R1
                  /
                  |
                  |
        analog 0----*
                  |
                  |
                  *----> nail 1

                  *----> nail 2
                  |
                  |
                  |
        digital 3---*
 */
#include <printf.h>
#include <dht.h>
#include <Enerlib.h>
#include <DS3232RTC.h>    //http://github.com/JChristensen/DS3232RTC
#include <RF24.h>

#define DEBUG

#ifdef DEBUG
#define DEBUG_SERIAL(x) Serial.begin(x)
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_SERIAL(x)
#define DEBUG_PRINT(x) 
#define DEBUG_PRINTLN(x) 
#endif


#define DHT11_PIN       3
#define DHT11_POWER_PIN 4
dht DHT;

#define moisture_input 1
#define divider_top    8
#define divider_bottom 4

#define RTC_POWER_PIN        3
// Set up nRF24L01 radio on SPI bus plus pins 9 & 10
// Pro mini SPI: 10 (SS), 11 (MOSI), 12 (MISO), 13 (SCK).
RF24 radio(9, 10);

Energy energy;

const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

void setup()
{
  DEBUG_SERIAL(57600);
  DEBUG_PRINTLN("Setup start");

  printf_begin();
  //InitializeDHT();
  InitializeNRF24();
  InitializeRTC();     // Initialize RTC last so we are sure we will be sleeping once wake up arrive

  DEBUG_PRINTLN("Setup end");
}

void InitializeDHT()
{
  pinMode(DHT11_POWER_PIN, OUTPUT);
  digitalWrite(DHT11_POWER_PIN, HIGH);

  DEBUG_PRINT("DHT LIBRARY VERSION: ");
  DEBUG_PRINTLN(DHT_LIB_VERSION);
  DEBUG_PRINTLN();
}

void InitializeRTC()
{
  DEBUG_PRINTLN("Initialize RTC start");
  pinMode(RTC_POWER_PIN, OUTPUT);
  digitalWrite(RTC_POWER_PIN, HIGH);

  /*
  Pin 2 will be the "wake button". Due to uC limitations,
  it needs to be a level interrupt.
  For experienced programmers:
    ATMega's datasheet contains information about the rest of
    wake up sources. The Extended Standby is not implemented.
  */
  attachInterrupt(0, INT0_ISR, HIGH);

  digitalWrite(2, HIGH);  //pull up the interrupt pin
  RTC.squareWave(SQWAVE_NONE);    //no square wave
  RTC.setAlarm(ALM1_EVERY_SECOND   , 0, 0, 0, 0);  // ALM1_MATCH_SECONDS, ALM1_EVERY_SECOND
  RTC.alarmInterrupt(ALARM_1, true);      //assert the INT pin when Alarm1 occurs.
  RTC.alarm(ALARM_1);

  DEBUG_PRINTLN("Initialize RTC end");
}

void InitializeNRF24()
{
  DEBUG_PRINTLN("Initialize NRF24 start");
  radio.begin();
  radio.setPayloadSize(12);
  radio.setPALevel( RF24_PA_MAX ) ;     // Max power
  radio.setDataRate( RF24_250KBPS ) ;   // Min speed (for better range I presume)
  radio.setCRCLength( RF24_CRC_16 ) ;   // 8 bits CRC
  radio.setRetries(15, 15);              // increase the delay between retries & # of retries
  radio.openWritingPipe(pipes[0]);
  radio.openReadingPipe(1, pipes[1]);
  radio.printDetails();
  radio.stopListening();
  DEBUG_PRINTLN("Initialize NRF24 end");
}

void loop()
{

  DEBUG_PRINTLN("PowerDown");
  RTC.alarm(ALARM_1);
  energy.PowerDown(); //Most power saving
  if (RTC.alarm(ALARM_1))
  {
    DEBUG_PRINTLN("ALARM_1");
  }

  sendMessage();
}

bool IsPowerLow = false;
unsigned long VccLowPower = 2900; // mV

void sendMessage()
{
  radio.powerUp();                                // Power up the radio after sleeping

  unsigned long vcc = readVcc();
  ChechNextWakeUpBasedOnPower(vcc);
  printf("Vcc = %lu\t", vcc);
  //digitalWrite(DHT11_POWER_PIN, HIGH);        // Power on DHT Temperature sensor in advance so it can boot up
  unsigned long soilMoisture = SoilMoisture(vcc);
  printf("Soil Moisture = %lu\t", soilMoisture);
  //ReadTemperatureAndHumidity();
  //digitalWrite(DHT11_POWER_PIN, LOW);
  //unsigned int temperatureCelsius = (unsigned int)(DHT.temperature * 10);
  //unsigned int humidity = (unsigned int)(DHT.humidity * 10);
  //printf("Temperature = %lu\tHumidity = %lu\t", temperatureCelsius, humidity);
  unsigned long temperature = RTC.temperature() * 2.5;
  printf("Temperature = %lu\t", temperature);

  const unsigned int messageLength = 3;
  unsigned long message[messageLength];
  message[0] = vcc;
  message[1] = soilMoisture;
  message[2] = temperature;
  //message[2] = temperatureCelsius;
  //message[3] = humidity;
  //printf("Now sending [\"%lu\", \"%lu\", \"%lu\", \"%lu\"]\r\n", message[0], message[1], message[2], message[3]);
  printf("Now sending [\"%lu\", \"%lu\", \"%lu\"]\r\n", message[0], message[1], message[2]);
  bool ok = radio.write( message, messageLength * sizeof(unsigned long) );
  if (ok)
  {
    printf("ok\n\r");
  }
  else
  {
    printf("failed\n\r");
  }

  radio.powerDown();              // NOTE: The radio MUST be powered back up again manually

  
}

void ChechNextWakeUpBasedOnPower(unsigned long vcc)
{
  bool newIsPowerLow = (vcc < VccLowPower);
  
  if (!IsPowerLow && newIsPowerLow)  // Entering Low Power Mode
  {
    DEBUG_PRINTLN("Entering Low Power Mode");
    IsPowerLow = true;
    RTC.setAlarm(ALM1_MATCH_MINUTES   , 0, 0, 0, 0); // Wake up every hour
  }

    if (IsPowerLow && !newIsPowerLow)  // Entering Normal Mode
  {
    DEBUG_PRINTLN("Entering Normal Mode");
    IsPowerLow = false;
    RTC.setAlarm(ALM1_MATCH_SECONDS   , 0, 0, 0, 0); // Wake up every minute
  }
}


void INT0_ISR(void)
{
  /*
  The WasSleeping function will return true if Arduino
  was sleeping before the IRQ. Subsequent calls to
  WasSleeping will return false until Arduino reenters
  in a low power state. The WasSleeping function should
  only be called in the ISR.
  */
  if (energy.WasSleeping())
  {
    DEBUG_PRINTLN("Wake up");

    /*
    Arduino was waked up by IRQ.

    If you shut down external peripherals before sleeping, you
    can reinitialize them here. Look on ATMega's datasheet for
    hardware limitations in the ISR when microcontroller just
    leave any low power state.
    */
  }
  else
  {
    //DEBUG_PRINTLN("WRONG interrupt");
    /*
    The IRQ happened in awake state.

    This code is for the "normal" ISR.
    */
  }
}



unsigned long SoilMoisture(unsigned int vcc)
{
  unsigned long reading;
  // set driver pins to outputs
  pinMode(divider_top, OUTPUT);
  pinMode(divider_bottom, OUTPUT);

  // drive a current through the divider in one direction
  digitalWrite(divider_top, LOW);
  digitalWrite(divider_bottom, HIGH);

  // wait a moment for capacitance effects to settle
  delay(100);

  // take a reading
  reading = analogRead(moisture_input);

  // reverse the current
  digitalWrite(divider_top, HIGH);
  digitalWrite(divider_bottom, LOW);

  // give as much time in 'reverse' as in 'forward'
  delay(100);

  // stop the current
  digitalWrite(divider_top, LOW);

  //reading = reading * vcc / (420 * 6);   // Correction for vcc and normalization into 0..1000
  return reading;
}

void ReadTemperatureAndHumidity()
{
  int chk = DHT.read11(DHT11_PIN);
  digitalWrite(DHT11_POWER_PIN, LOW);
  switch (chk)
  {
    case DHTLIB_OK:
      Serial.print("OK,\t");
      break;
    case DHTLIB_ERROR_CHECKSUM:
      Serial.print("Checksum error,\t");
      break;
    case DHTLIB_ERROR_TIMEOUT:
      Serial.print("Time out error,\t");
      break;
    case DHTLIB_ERROR_CONNECT:
      Serial.print("Connect error,\t");
      break;
    case DHTLIB_ERROR_ACK_L:
      Serial.print("Ack Low error,\t");
      break;
    case DHTLIB_ERROR_ACK_H:
      Serial.print("Ack High error,\t");
      break;
    default:
      Serial.print("Unknown error,\t");
      break;
  }
  // DISPLAY DATA
  //Serial.print(DHT.humidity, 1);
  //Serial.print(",\t");
  //DEBUG_PRINTLN(DHT.temperature, 1);
}

unsigned long readVcc()
{
  unsigned long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA, ADSC));
  result = ADCL;
  result |= ADCH << 8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}
