


#include <MySensor.h>  
#include <SPI.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <SimpleTimer.h>
//#include <DigitalIO.h>

#include <Bounce2.h>
#include <avr/wdt.h>


 //#define NDEBUG                        // enable local debugging information


#define NODE_ID 190 //На даче поменять на 100

/******************************************************************************************************/
/*                               				Сенсоры												  */
/******************************************************************************************************/

#define CHILD_ID_POWEOFFRELAY				  30  //Реле 1

#define CHILD_ID_POWEOFFRELAY_STATUS	  20  //Фоторезистор на светодиоде статуса реле 1
#define CHILD_ID_AMBIENTLIGHT           21
#define CHILD_ID_MODEBUTTON             40

#define REBOOT_CHILD_ID                       100
#define RECHECK_SENSOR_VALUES                 101 
#define DISABLE_SWITCHOFFPOWER_CHILD_ID  	    102
#define NIGHTMODE_CHILD_ID                    105


/******************************************************************************************************/
/*                               				IO												                                    */
/******************************************************************************************************/

#define ONE_WIRE_BUS              3      // Pin where dallase sensor is connected 
#define POWEOFFRELAY_PIN		      4
#define LIGHT_SENSOR_POWEOFFRELAY A4 
#define LIGHT_SENSOR_AMBIENT      A5        
#define MODELED_PIN               A3
#define MODEBUTTON_PIN            5

/*****************************************************************************************************/
/*                               				Common settings									      */
/******************************************************************************************************/

#define RELAY_ON 1  // GPIO value to write to turn on attached relay
#define RELAY_OFF 0 // GPIO value to write to turn off attached relay

#define COMPARE_TEMP              1      // Send temperature only if changed? 1 = Yes 0 = No
#define MAX_ATTACHED_DS18B20      14
#define EEPROM_DEVICE_ADDR_START  64     // start byte in eeprom for remembering our sensors
#define EEPROM_DEVICE_ADDR_END    EEPROM_DEVICE_ADDR_START+MAX_ATTACHED_DS18B20*2

#define RADIO_RESET_DELAY_TIME 40 //Задержка между сообщениями
#define MESSAGE_ACK_RETRY_COUNT 5  //количество попыток отсылки сообщения с запросом подтверждения

//#define NUM_OF_PRESENTED_TEMP_SENSORS 14

OneWire oneWire(ONE_WIRE_BUS);        // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire);  // Pass the oneWire reference to Dallas Temperature. 

float lastTemperature[MAX_ATTACHED_DS18B20];
uint8_t numSensors = 0;
uint8_t currentTsensor = 0;
DeviceAddress dsaddr[MAX_ATTACHED_DS18B20];
bool ts_spot[MAX_ATTACHED_DS18B20]; // used spot array

boolean gotAck=false; //подтверждение от гейта о получении сообщения 
int iCount = MESSAGE_ACK_RETRY_COUNT;

boolean boolRecheckSensorValues = false;
boolean boolRecheckTempValues = false;
boolean boolSwitchOffPowerDisabled = false;

boolean boolReportPowerOffDisabledState = false;

int lastRelayPowerStatusLightLevel;             // Holds last light level
boolean boolHardwareSwitchOffPowerDisabled = false;

float highestSensorsTemperature = 0;
float lasthighestSensorsTemperature = 0;
int lastAmbientLightLevel = 0;

boolean boolNightMode = false;


Bounce debouncer = Bounce(); 


// Initialize temperature message
MyMessage TempMsg(0, V_TEMP);  //CHILD_ID_TEMPERATURE

// Initialize temperature message
MyMessage SwitchOffPowerCheckStateMsg(DISABLE_SWITCHOFFPOWER_CHILD_ID, V_TRIPPED);


MyMessage RelayPowerStatusLightMsg(CHILD_ID_POWEOFFRELAY_STATUS, V_LIGHT_LEVEL);

MyMessage ButtonStateMsg(CHILD_ID_MODEBUTTON, V_TRIPPED);

MyMessage AmbientLightLevelMsg(CHILD_ID_AMBIENTLIGHT, V_LIGHT_LEVEL);

//boolean receivedConfig = false;
boolean metric = true; 

SimpleTimer timer;
SimpleTimer SwitchOffPowerCheckStateReporttimer;
//SimpleTimer checkRelayPowerStatus;
//SimpleTimer checkHardwareButton;



MySensor gw;




void setup()  
{ 

//set GRB LED pin modes
pinMode(A0, OUTPUT);
pinMode(A1, OUTPUT);
pinMode(A2, OUTPUT);

// begin setup
setLEDColor(false,false,true);



  // Startup up the OneWire library
  sensors.begin();

  
  // requestTemperatures() will not block current thread
  sensors.setWaitForConversion(false);

  // Startup and initialize MySensors library. Set callback for incoming messages. 
  gw.begin(incomingMessage, NODE_ID, false);
  gw.wait(RADIO_RESET_DELAY_TIME); 
  // Send the sketch version information to the gateway and Controller
  


  gw.sendSketchInfo("PowerBoxTsensor", "1.1", true);
  gw.wait(RADIO_RESET_DELAY_TIME); 


  // Fetch the number of attached temperature sensors  
  numSensors = sensors.getDeviceCount();

        #ifdef NDEBUG
          Serial.print(F("Number of sensors # "));
          Serial.println(numSensors);
        #endif

  for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {
     // reset used spot array
     ts_spot[i] = false;
  }
  for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {
     sensors.getAddress(dsaddr[i],i);
     // check if we know this sensor
     int8_t sidx = getSensorIndex(dsaddr[i]);
     if (sidx >= 0) {
        // we know this sensor
        ts_spot[sidx] = true;
     }
  }
  // Present all sensors to controller
  for (int i=0; i<numSensors && i<MAX_ATTACHED_DS18B20; i++) {
     int8_t sidx = getSensorIndex(dsaddr[i]);
     if (sidx < 0) {
        // we have unknown sensor (not present in EEPROM)
        // let's find a first free spot and put the addr hash there
        uint8_t spot = 0;
        while (ts_spot[spot] && spot < MAX_ATTACHED_DS18B20) {
          spot++;
        }
        ts_spot[spot] = true;
        storeSensorAddr(dsaddr[i],spot);
        sidx = spot;
        #ifdef NDEBUG
          Serial.print(F("Added new sensor to spot # "));
          Serial.println(spot);
        #endif
     }
     
     gw.present(sidx, S_TEMP);
  //   #ifdef NDEBUG
  //     Serial.println();
  //     Serial.print(i);
  //     Serial.print(F(" index: "));
  //     Serial.print(sidx);
  //     Serial.print(F(" address: "));
  //     printAddress(dsaddr[i]);
  //     Serial.println();
  //   #endif
  }

    //reboot sensor command
    gw.wait(RADIO_RESET_DELAY_TIME);
    gw.present(REBOOT_CHILD_ID, S_BINARY); //, "Reboot node sensor", true); 

    //reget sensor values
    gw.wait(RADIO_RESET_DELAY_TIME);
  	gw.present(RECHECK_SENSOR_VALUES, S_LIGHT); 

	//disable temp difference check
     gw.wait(RADIO_RESET_DELAY_TIME);
  	gw.present(DISABLE_SWITCHOFFPOWER_CHILD_ID, S_LIGHT); 


  //ambient light level
     gw.wait(RADIO_RESET_DELAY_TIME);
    gw.present(CHILD_ID_AMBIENTLIGHT, S_LIGHT_LEVEL); 

    // Relays
    pinMode(POWEOFFRELAY_PIN, OUTPUT); 
    gw.wait(RADIO_RESET_DELAY_TIME); 
    gw.present(CHILD_ID_POWEOFFRELAY, S_LIGHT);    
 
    //Relays status light sensors
    pinMode(LIGHT_SENSOR_POWEOFFRELAY, INPUT);
    gw.wait(RADIO_RESET_DELAY_TIME);
    gw.present(CHILD_ID_POWEOFFRELAY_STATUS, S_LIGHT_LEVEL);

    //Button
    gw.wait(RADIO_RESET_DELAY_TIME);
    gw.present(CHILD_ID_MODEBUTTON, V_TRIPPED); 

    //Night mode
    gw.wait(RADIO_RESET_DELAY_TIME);
    gw.present(NIGHTMODE_CHILD_ID, S_DOOR); 

     //set led pin mode
      pinMode(MODELED_PIN, OUTPUT);
      digitalWrite(MODELED_PIN, LOW);
      // Setup the button
      pinMode(MODEBUTTON_PIN,INPUT);
      // Activate internal pull-up
      digitalWrite(MODEBUTTON_PIN,HIGH);
  
      // After setting up the button, setup debouncer
      debouncer.attach(MODEBUTTON_PIN);
      debouncer.interval(5);

    gw.wait(RADIO_RESET_DELAY_TIME); 
    gw.request(DISABLE_SWITCHOFFPOWER_CHILD_ID, V_LIGHT);

    gw.wait(RADIO_RESET_DELAY_TIME); 
    gw.request(NIGHTMODE_CHILD_ID, V_TRIPPED); 

 
      startupChecks(numSensors);

      //boolRecheckSensorValues = true;

  // start our periodic jobs
  // many other periodic jobs can be added here
  timer.setInterval(60000, checkTemperature);
  SwitchOffPowerCheckStateReporttimer.setInterval(120000, reportSwitchOffPowerCheckState);
  //checkRelayPowerStatus.se  tInterval(120000, checkRelayStatus);
  //checkHardwareButton.setInterval(5000, checkButtonState);


  //Enable watchdog timer
  wdt_enable(WDTO_8S);


// end setup
setLEDColor(false,false,false);

//        #ifdef NDEBUG
//          Serial.print(F("End setup "));

//        #endif

}


void loop() {
  timer.run();
  SwitchOffPowerCheckStateReporttimer.run();
  //checkRelayPowerStatus.run();
  //checkHardwareButton.run();

  gw.process();


if (boolRecheckSensorValues)
{
  checkRelayStatus();
  checkButtonState();
  checkAmbientLight();
  boolRecheckSensorValues = false;
}

if (boolReportPowerOffDisabledState)
{
   reportSwitchOffPowerCheckState();
   boolReportPowerOffDisabledState = false;
}
    //reset watchdog timer
    wdt_reset();    
}



void incomingMessage(const MyMessage &message) {

  if (message.isAck())
  {
    gotAck = true;
    return;
  }

    if ( message.sensor == REBOOT_CHILD_ID && message.getBool() == true && strlen(message.getString())>0 ) {
             wdt_enable(WDTO_30MS);
              while(1) {};

     }
     
     if (message.type==V_LIGHT && strlen(message.getString())>0 ) 
     {
         if ( message.sensor == CHILD_ID_POWEOFFRELAY ) 
         {
               
               if (message.getBool())
               {
               		switchPowerOFF();

               }
         }
     
     }

    if ( message.sensor == DISABLE_SWITCHOFFPOWER_CHILD_ID && !boolHardwareSwitchOffPowerDisabled && strlen(message.getString())>0) {
         
         if (message.getBool() == true)
         {
            boolSwitchOffPowerDisabled = true;           
         }
         else
         {
            boolSwitchOffPowerDisabled = false;
         }

                if (!boolNightMode)
                {           
                    digitalWrite(MODELED_PIN, boolSwitchOffPowerDisabled ? 0 : 1); 
                }
            boolReportPowerOffDisabledState = true;
     }

    if ( message.sensor == NIGHTMODE_CHILD_ID  && strlen(message.getString())>0 ) {
         
         if (message.getBool() == true)
         {
            setLEDColor(false,false,false);
            digitalWrite(MODELED_PIN, LOW);
            boolNightMode = true;
         }
         else
         {
            boolNightMode = false;
            digitalWrite(MODELED_PIN, boolSwitchOffPowerDisabled ? 0 : 1);
            visualizeCurrentTempState ( lasthighestSensorsTemperature );
            
         }

     }


    if ( message.sensor == RECHECK_SENSOR_VALUES && strlen(message.getString())>0 ) {
         
         if (message.getBool() == true)
         {
            boolRecheckSensorValues = true;
            boolRecheckTempValues = true;

         }

     }

        return;      
} 


// a simple hash function for 1wire address to reduce id to two bytes
uint16_t simpleAddrHash(DeviceAddress a){
  return ((a[1] ^ a[2] ^ a[3] ^ a[4] ^ a[5] ^ a[6]) << 8) + a[7];
}

// search for device address hash in eeprom
// return -1 if not found
int8_t getSensorIndex(DeviceAddress a) {
  uint16_t hash = simpleAddrHash(a);
  int8_t idx = -1;
  uint8_t aidx = 0;
  uint8_t ptr = EEPROM_DEVICE_ADDR_START;
  while (ptr < EEPROM_DEVICE_ADDR_END && idx == -1) {
    uint8_t hash1 = gw.loadState(ptr);
    uint8_t hash2 = gw.loadState(ptr+1);
    if ( hash1 == (uint8_t)(hash >> 8) && hash2 == (uint8_t)(hash & 0x00FF)) {
      // found device index
      idx = aidx;
    }
    aidx++;
    ptr+=2;
  }
  return idx;
}

// save address hash in EEPROM under index
void storeSensorAddr(DeviceAddress a, uint8_t index) {
    uint16_t hash = simpleAddrHash(a);
    uint8_t ptr = EEPROM_DEVICE_ADDR_START + index * 2;
    if (ptr < EEPROM_DEVICE_ADDR_END) {
      gw.saveState(ptr,   hash >> 8);
      gw.saveState(ptr+1, hash & 0x00FF);
    }
    #ifdef NDEBUG
      Serial.print(F("storeSensorAddr under index: "));
      Serial.println(index);
    #endif
}



void checkTemperature(){
  // Check temperature
  // Fetch temperatures from Dallas sensors
  sensors.requestTemperatures();

  // query conversion time and sleep until conversion completed
  int16_t conversionTime = sensors.millisToWaitForConversion(sensors.getResolution());
  
  if (numSensors > 0) {
    currentTsensor = 0;
    timer.setTimeout(conversionTime, readTemperature);
  }

  checkButtonState();
}

void readTemperature(){

  #ifdef NDEBUG
  unsigned long start = millis();
  #endif

  // Fetch and round temperature to one decimal
  //float temperature = static_cast<float>(static_cast<int>(sensors.getTempC(dsaddr[currentTsensor]) * 10.)) / 10.;
  // Fetch and round temperature to one decimal
  //float temperature = static_cast<float>(static_cast<int>((gw.getConfig().isMetric?sensors.getTempC(dsaddr[currentTsensor]):sensors.getTempF(dsaddr[currentTsensor])) * 10.)) / 10.;
  float temperature = static_cast<float>(static_cast<int>(sensors.getTempC(dsaddr[currentTsensor]) * 10.)) / 10.;


  // Only send data if temperature has changed and no error
  #if COMPARE_TEMP == 1
  if ( ((lastTemperature[currentTsensor] != temperature) || boolRecheckTempValues ) && temperature != -127.00 && temperature != 85.00) {
  #else
  if (temperature != -127.00 && temperature != 85.00) {
  #endif

      if ( temperature > 59 && !boolSwitchOffPowerDisabled )
      {
        //TODO: отослать номер сенсора
        switchPowerOFF(); //Выключить электричество, если какой-то из автоматов нагрелся более чем на 59 градусов
      }

       // if ( (abs(lastTemperature[currentTsensor] - temperature ) >= 0.2) || boolRecheckTempValues )
       // {
            //Отсылаем состояние сенсора с подтверждением получения
            iCount = MESSAGE_ACK_RETRY_COUNT;

              while( !gotAck && iCount > 0 )
                {
    	             // Send in the new temperature                  
    	             gw.send(TempMsg.setSensor(getSensorIndex(dsaddr[currentTsensor])).set(temperature,1), true);
                    gw.wait(RADIO_RESET_DELAY_TIME);
                  iCount--;
                 }

                gotAck = false;
        //  }
    // Save new temperatures for next compare
    lastTemperature[currentTsensor] = temperature;

  }
 
  //Serial.print(F(" -> "));
  //Serial.println(temperature);

 
  #ifdef NDEBUG
  Serial.print(F("Temperature "));
  Serial.print(currentTsensor,DEC);
  Serial.print(F(" index: "));
  Serial.print(getSensorIndex(dsaddr[currentTsensor]));
  Serial.print(F(" -> "));
  Serial.print(temperature);
  unsigned long etime = millis() - start;
  Serial.print(F(" time elapsed: "));
  Serial.println(etime);
  #endif
  
  if ( temperature > highestSensorsTemperature )
  {
    highestSensorsTemperature = temperature;
  }

  currentTsensor++;
  if (currentTsensor < numSensors && currentTsensor < MAX_ATTACHED_DS18B20) {
    // postpone next sensor reading
    timer.setTimeout(25, readTemperature);    
  } else
  {

      visualizeCurrentTempState ( highestSensorsTemperature );

      lasthighestSensorsTemperature = highestSensorsTemperature;              
      highestSensorsTemperature = 0;

      if (boolRecheckTempValues)
      {
        boolRecheckTempValues=false;
      }
  }

}


void visualizeCurrentTempState ( float fTemp )
{

        if (fTemp <= 45 )
      {
        setLEDColor(false,true,false);

      } else if (fTemp > 45 && fTemp <= 50)
            {
                  setLEDColor(false,false,true);

            } else if ( fTemp > 50 )
                    {
                      setLEDColor(true,false,false);
                    }
}


void reportSwitchOffPowerCheckState()
{


      //Отсылаем состояние сенсора софт блокировки выключения питания по температуре с подтверждением получения
     iCount = MESSAGE_ACK_RETRY_COUNT;

       while( !gotAck && iCount > 0 )
        {
         gw.send(SwitchOffPowerCheckStateMsg.set(boolSwitchOffPowerDisabled ? "1" : "0" ), true);    // Send motion value to gw
         gw.wait(RADIO_RESET_DELAY_TIME);
          iCount--;
       }

       gotAck = false;

    //Отсылаем состояние сенсора аппаратной блокировки выключения питания с подтверждением получения
    iCount = MESSAGE_ACK_RETRY_COUNT;

    while( !gotAck && iCount > 0 )
    {
//      gw.send(ButtonStateMsg.set(value==HIGH ? 1 : 0), true); 
      gw.send(ButtonStateMsg.set(boolHardwareSwitchOffPowerDisabled), true);       
      gw.wait(RADIO_RESET_DELAY_TIME);
      iCount--;
    }
    gotAck = false;       

      checkRelayStatus();

      checkAmbientLight();

}


void checkRelayStatus()
{
  

  int  lightLevel=0;
  

     lightLevel = (1023-analogRead(LIGHT_SENSOR_POWEOFFRELAY))/10.23; 

 //       #ifdef NDEBUG
 //         Serial.print(F("Relay power light level # "));
 //         Serial.println(lightLevel);
 //       #endif
            
      if (lightLevel > 0)
      {    

          if ( (lightLevel != lastRelayPowerStatusLightLevel) || boolRecheckSensorValues) {


      //Отсылаем состояние сенсора с подтверждением получения
     iCount = MESSAGE_ACK_RETRY_COUNT;

       while( !gotAck && iCount > 0 )
        {
         gw.send(RelayPowerStatusLightMsg.set(lightLevel), true);   // Send motion value to gw
         gw.wait(RADIO_RESET_DELAY_TIME);
          iCount--;
       }

       gotAck = false;

            
              
            lastRelayPowerStatusLightLevel = lightLevel;
          }
    
    
          lightLevel=0;

  }  
  
}


void checkButtonState()
{
    debouncer.update();
  // Get the update value
  int value = debouncer.read();
 
  value=value ? 0 : 1;

  if (value != boolHardwareSwitchOffPowerDisabled || boolRecheckSensorValues) {


    if (!boolNightMode)
    {
      digitalWrite(MODELED_PIN, value ? 0 : 1 );
    }

    //Отсылаем состояние сенсора с подтверждением получения
    iCount = MESSAGE_ACK_RETRY_COUNT;

    while( !gotAck && iCount > 0 )
    {
//      gw.send(ButtonStateMsg.set(value==HIGH ? 1 : 0), true); 
      gw.send(ButtonStateMsg.set(value), true);       
      gw.wait(RADIO_RESET_DELAY_TIME);
      iCount--;
    }
    gotAck = false;

     boolHardwareSwitchOffPowerDisabled = boolSwitchOffPowerDisabled = value;


    //reportSwitchOffPowerCheckState();

       //Отсылаем состояние сенсора с подтверждением получения
     iCount = MESSAGE_ACK_RETRY_COUNT;

       while( !gotAck && iCount > 0 )
        {
         gw.send(SwitchOffPowerCheckStateMsg.set(boolSwitchOffPowerDisabled ? "1" : "0" ), true);    // Send motion value to gw
         gw.wait(RADIO_RESET_DELAY_TIME);
          iCount--;
       }

       gotAck = false;


  }


}


void ledShowTempState()
{

highestSensorsTemperature = 0;


 for (int currentTsensor = 0; currentTsensor < numSensors; currentTsensor++) 
 {
       if ( lastTemperature[currentTsensor] > highestSensorsTemperature )
      {
        highestSensorsTemperature = lastTemperature[currentTsensor];
      }
 }


  
      if (highestSensorsTemperature < 35 )
      {
        setLEDColor(false,true,false);

      } else if (highestSensorsTemperature >= 35 && highestSensorsTemperature < 50)
            {
                  setLEDColor(true,true,false);

            } else if ( highestSensorsTemperature >= 50 )
                    {
                      setLEDColor(true,false,false);
                    }

}


void checkAmbientLight()
{
  

  int  lightLevel=0;
  

     //lightLevel = (1023-analogRead(LIGHT_SENSOR_AMBIENT))/10.23; 

     lightLevel = analogRead(LIGHT_SENSOR_AMBIENT); 

     double Vout=lightLevel*0.0048828125;
     lightLevel=(2500/Vout-500)/100;

        #ifdef NDEBUG
          Serial.print(F("Ambient light level # "));
          Serial.println(lightLevel);
        #endif
            
      if (lightLevel > 0)
      {    

          if ( (lightLevel != lastAmbientLightLevel) || boolRecheckSensorValues) {


      //Отсылаем состояние сенсора с подтверждением получения
     iCount = MESSAGE_ACK_RETRY_COUNT;

       while( !gotAck && iCount > 0 )
        {
         gw.send(AmbientLightLevelMsg.set(lightLevel), true);   // Send motion value to gw
         gw.wait(RADIO_RESET_DELAY_TIME);
          iCount--;
       }

       gotAck = false;

            
              
            lastAmbientLightLevel = lightLevel;
          }
    
    
          lightLevel=0;

  }  
  
}


void switchPowerOFF()
{

  	digitalWrite(POWEOFFRELAY_PIN, RELAY_ON);
   	gw.wait(200);
  	digitalWrite(POWEOFFRELAY_PIN, RELAY_OFF);

}


void setLEDColor(boolean bRed, boolean bGreen, boolean bBlue)
{
  if (!boolNightMode)
  {
    digitalWrite(A0, bBlue ? 0 : 1);
    digitalWrite(A1, bGreen ? 0 : 1);    
    digitalWrite(A2, bRed ? 0 : 1);   
  }     
}

/* Проверка при загрузке                                                            */
/* При ошибке мигает светодиодом:                                                    */
/* RED-GREEN - нет питания на реле                                                  */
/* RED-BLUE - не найдены один или несколько температурных сенсоров                  */

void startupChecks(int numOfSensors)
{
  int  lightLevel=0;


      debouncer.update();

      // Get the update value
      int value = debouncer.read();
 
      value=value ? 0 : 1;
  

     lightLevel = (1023-analogRead(LIGHT_SENSOR_POWEOFFRELAY))/10.23; 

            
      if (lightLevel < 80 && !value)
      { 
          //error. Blink RED+BLUE

          while (!value)
          {
            setLEDColor(true, false, false);

            gw.wait(500);
          
            setLEDColor(false, true, false);

            gw.wait(500);

            debouncer.update();
            value = debouncer.read();
            value=value ? 0 : 1;
        };

      }

      if (numOfSensors < MAX_ATTACHED_DS18B20 && !value)
      { 
          while (!value)
          {
            setLEDColor(true, false, false);

            gw.wait(500);
          
            setLEDColor(false, false, true);

            gw.wait(500);

            debouncer.update();
            value = debouncer.read();
            value=value ? 0 : 1;
        };
      }

}



/*
#ifdef NDEBUG
// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}
#endif
*/