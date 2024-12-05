/*
 * This code is and adobtion from the example code provided by Heltec Automation
 * An easy waht to achive our goal, but I would code in a different style.
*/

#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "deviceConfig.h"
#include <Adafruit_Si7021.h>
#include <tinycbor.h>

/*
 * set LoraWan_RGB to Active,the RGB active in loraWan
 * RGB red means sending;
 * RGB purple means joined done;
 * RGB blue means RxWindow1;
 * RGB yellow means RxWindow2;
 * RGB green means received done;
 */

/* OTAA para*/
uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xC8, 0x16 };
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appKey[] = { 0xC6, 0x7A, 0xB9, 0x48, 0x5D, 0x57, 0x15, 0x68, 0xA6, 0xD2, 0xB3, 0x85, 0x4A, 0x37, 0x17, 0xEC };

/* ABP para */
uint8_t nwkSKey[] = NWKSKEY;
uint8_t appSKey[] = APPSKEY;
uint32_t devAddr =  ( uint32_t ) DEVADDR; 
 
/*LoraWan channelsmask, default channels 0-7*/ 
uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t  loraWanClass = LORAWAN_CLASS;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 60*60*1000; // 30 minutes

uint32_t cyclesToSend = 1; //60*30; // 30 minutes

uint32_t count = 0;

/*OTAA or ABP*/
bool overTheAirActivation = LORAWAN_NETMODE;

/*ADR enable*/
bool loraWanAdr = LORAWAN_ADR;

/* set LORAWAN_Net_Reserve ON, the node could save the network info to flash, when node reset not need to join again */
bool keepNet = true; //LORAWAN_NET_RESERVE;

/* Indicates if the node is sending confirmed or unconfirmed messages */
//#define LORAWAN_UPLINKMODE false
bool isTxConfirmed = false; //LORAWAN_UPLINKMODE;

/* Application port */
uint8_t appPort = 2;

/*!
* Number of trials to transmit the frame, if the LoRaMAC layer did not
* receive an acknowledgment. The MAC performs a datarate adaptation,
* according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
* to the following table:
*
* Transmission nb | Data Rate
* ----------------|-----------
* 1 (first)       | DR
* 2               | DR
* 3               | max(DR-1,0)
* 4               | max(DR-1,0)
* 5               | max(DR-2,0)
* 6               | max(DR-2,0)
* 7               | max(DR-3,0)
* 8               | max(DR-3,0)
*
* Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
* the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 3;

/* Si7021 Sensor*/
Adafruit_Si7021 envSensor = Adafruit_Si7021();

/*  
 * get the BatteryVoltage in mV. 
 * Source: http://community.heltec.cn/t/measure-battery-level-htcc-ab01/9511
 */

uint32_t BoardGetBatteryVoltage(void)
{
    float temp = 0;
    uint16_t volt;
    uint8_t pin;
    pin = ADC;
    #if defined(CubeCell_Board) || defined(CubeCell_Capsule) || defined(CubeCell_BoardPlus) || defined(CubeCell_BoardPRO) || defined(CubeCell_GPS) || defined(CubeCell_HalfAA)
        /*
        * have external 10K VDD pullup resistor
        * connected to VBAT_ADC_CTL pin
        */
        pinMode(VBAT_ADC_CTL, OUTPUT);
        digitalWrite(VBAT_ADC_CTL, LOW);
    #endif
    for (int i = 0; i < 50; i++) // read 50 times and get average
        temp += analogReadmV(pin);
    volt = temp / 50;
    #if defined(CubeCell_Board) || defined(CubeCell_Capsule) || defined(CubeCell_BoardPlus) || defined(CubeCell_BoardPRO) || defined(CubeCell_GPS) || defined(CubeCell_HalfAA)
        pinMode(VBAT_ADC_CTL, INPUT);
    #endif
    volt = volt * 2;
    return volt;
}

/* Prepares the payload of the frame */
static void prepareTxFrame( uint8_t port )
{
	/*appData size is LORAWAN_APP_DATA_MAX_SIZE which is defined in "commissioning.h".
	*appDataSize max value is LORAWAN_APP_DATA_MAX_SIZE.
	*if enabled AT, don't modify LORAWAN_APP_DATA_MAX_SIZE, it may cause system hanging or failure.
	*if disabled AT, LORAWAN_APP_DATA_MAX_SIZE can be modified, the max value is reference to lorawan region and SF.
	*for example, if use REGION_CN470, 
	*the max value for different DR can be found in MaxPayloadOfDatarateCN470 refer to DataratesCN470 and BandwidthsCN470 in "RegionCN470.h".
	*/

    float voltage = BoardGetBatteryVoltage()/1000.0;    
    uint8_t encode_buffer[242];
    TinyCBOR.init();

    // Assign buffer to encoder.
    TinyCBOR.Encoder.init(encode_buffer, sizeof(encode_buffer));
    TinyCBOR.Encoder.create_map(3);
    {
      TinyCBOR.Encoder.encode_text_string("temp", 4);
      TinyCBOR.Encoder.encode_float(envSensor.readTemperature());  
      TinyCBOR.Encoder.encode_text_string("hum", 3);
      TinyCBOR.Encoder.encode_float(envSensor.readHumidity());   
      TinyCBOR.Encoder.encode_text_string("vbat", 4);
      TinyCBOR.Encoder.encode_float(voltage);
    }        
    TinyCBOR.Encoder.close_container();

    Serial.println("Temperature: " + String(envSensor.readTemperature()) + " C");
    Serial.println("Humidity: " + String(envSensor.readHumidity()) + " %");
    Serial.println("Battery Voltage: " + String(voltage) + " mV");

    uint8_t* buffer = TinyCBOR.Encoder.get_buffer(); 
    size_t buffer_size = TinyCBOR.Encoder.get_buffer_size();

    // appData and appDataSzie are a global variables, used by the LoRaWAN library.
    // Ugly programming, but it is what it is.
    memcpy(appData, buffer, buffer_size);
    appDataSize = buffer_size;
}


void setup() {
	Serial.begin(9600);
      // wait for serial port to open
    while (!Serial) {
        delay(10);
    }
    boardInitMcu();
    #if(AT_SUPPORT)
	    enableAt();
    #endif
    if (!envSensor.begin()) {
        Serial.println("Did not find Si7021 sensor!");
        while (true);
    }
	deviceState = DEVICE_STATE_INIT;
    LoRaWAN.ifskipjoin();

}

void loop()
{
	switch( deviceState )
	{
		case DEVICE_STATE_INIT:
		{
            #if(AT_SUPPORT)
			    getDevParam();
            #endif
			printDevParam();
			LoRaWAN.init(loraWanClass,loraWanRegion);
			deviceState = DEVICE_STATE_JOIN;
			break;
		}
		case DEVICE_STATE_JOIN:
		{
			LoRaWAN.join();
			break;
		}
		case DEVICE_STATE_SEND:
		{
            prepareTxFrame( appPort );
            LoRaWAN.send();
			deviceState = DEVICE_STATE_CYCLE;
			break;
		}
		case DEVICE_STATE_CYCLE:
		{
			// Schedule next packet transmission
			txDutyCycleTime = appTxDutyCycle + randr( 0, APP_TX_DUTYCYCLE_RND );
			LoRaWAN.cycle(txDutyCycleTime);
            deviceState = DEVICE_STATE_SLEEP;
			break;
		}
		case DEVICE_STATE_SLEEP:
		{
			LoRaWAN.sleep();
			break;
		}
		default:
		{
			deviceState = DEVICE_STATE_INIT;
			break;
		}
	}
}