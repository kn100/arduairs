// kn100.me - Arduairs
// Available on Github: https://github.com/kn100/arduairs
// Note: This code is provided as an example only - it does not implement authentication, TLS or anything even remotely close to security. It's probably fine if all your infrastructure lives on your local network, but you might want to consider looking into security.

#include <PubSubClient.h>

#include "bsec.h"
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <EEPROM.h>

// Controls which pins are the I2C ones.
#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22

// Controls how often the code persists the BSEC Calibration data
#define STATE_SAVE_PERIOD  UINT32_C(60 * 60 * 1000) // every 60 minutes

const char* mqtt_server = "<ip>";
const char* ssid = "<your-ssid>";
const char* passphrase = "<your-password>";

// Controls the offset for the temperature sensor. My BME680 was reading 4 degrees higher than another thermometer I trusted more, so my offset is 4.0.
const float tempOffset = 4.0;

// Configuration data for the BSEC library - telling it it is running on a 3.3v supply, that it is read from every 3 seconds, and that the sensor should take into account the last 4 days worth of data for calibration purposes.
const uint8_t bsec_config_iaq[] = {
#include "config/generic_33v_3s_4d/bsec_iaq.txt"
};

// Create an object of the class Bsec
Bsec iaqSensor;

String output;

// This stores a question mark and is eventually replaced with a space character once the BSEC library has decided it has enough calibration data to persist. It is displayed in the bottom right of the LCD as a kind of debug symbol. 
String sensorPersisted = "?";

uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE] = {0};
uint16_t stateUpdateCounter = 0;

WiFiClient espClient;
PubSubClient broker(espClient);

// The I2C address of your LCD, it will likely either be 0x27 or 0x3F
const uint8_t LCD_ADDR = 0x27;

// 20 characters across, 4 lines deep
LiquidCrystal_I2C lcd(LCD_ADDR, 20, 4);

void setup(void)
{
  Serial.begin(115200);
  broker.setServer(mqtt_server, 1883);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  // Your module MAY use the primary address, which is available as BME680_I2C_ADDR_PRIMARY
  iaqSensor.begin(BME680_I2C_ADDR_SECONDARY, Wire);
  output = "\nBSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix);
  Serial.println(output);
  checkIaqSensorStatus();
  iaqSensor.setConfig(bsec_config_iaq);

  //loadState here refers to the BSEC Calibration state.
  loadState();

  // List all the sensors we want the bsec library to give us data for
  bsec_virtual_sensor_t sensorList[10] = {
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
  };

  iaqSensor.setTemperatureOffset(tempOffset);
  
  // Receive data from sensor list above at BSEC_SAMPLE_RATE_LP rate (every 3 seconds). There is also BSEC_SAMPLE_RATE_ULP - which requires a configuration change above.
  iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);
  checkIaqSensorStatus();

  lcd.init();
  lcd.backlight();
  // I wanted some iconography, so this function creates some icons in the LCDs memory. They were created using https://maxpromer.github.io/LCD-Character-Creator/
  createLCDSymbols();
  connectToNetwork();
}

// Function that is looped forever
void loop(void)
{
  if (!broker.connected()) {
    reconnectToBroker();
  }
  broker.loop();
  // iaqSensor.run() will return true once new data becomes available
  if (iaqSensor.run()) {
    lcd.clear();
    displayIAQ(String(iaqSensor.staticIaq));
    displayTemp(String(iaqSensor.temperature));
    displayHumidity(String(iaqSensor.humidity));
    displayPressure(String(iaqSensor.pressure/100));
    displayCO2(String(iaqSensor.co2Equivalent));
    displayVOC(String(iaqSensor.breathVocEquivalent));
    displaySensorPersisted();
    updateState();
  } else {
    checkIaqSensorStatus();
  }
}

// checks to make sure the BME680 Sensor is working correctly.
void checkIaqSensorStatus(void)
{
  if (iaqSensor.status != BSEC_OK) {
    lcd.setCursor(0, 0);
    lcd.print("Sensor error");
    if (iaqSensor.status < BSEC_OK) {
      output = "BSEC error code : " + String(iaqSensor.status);
      Serial.println(output);
    } else {
      output = "BSEC warning code : " + String(iaqSensor.status);
      Serial.println(output);
    }
    delay(5000);
    lcd.clear();
    checkIaqSensorStatus();
  }

  if (iaqSensor.bme680Status != BME680_OK) {
    lcd.setCursor(0, 0);
    lcd.print("Sensor error");
    if (iaqSensor.bme680Status < BME680_OK) {
      output = "BME680 error code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
    } else {
      output = "BME680 warning code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
    }
    delay(5000);
    lcd.clear();
  }
}

// The below display functions display data as well as publishing it to the MQTT broker. They expect the area that they are rendering to to be free of characters.
void displayIAQ(String iaq)
{
  lcd.setCursor(0, 0);
  lcd.write(0);
  lcd.print(iaq);
  char carr[iaq.length()];
  iaq.toCharArray(carr, iaq.length());
  broker.publish("bme680/iaq", carr);
}

void displayTemp(String tmp)
{
  lcd.setCursor(14, 0);
  lcd.print(tmp);
  lcd.write(1);
  char carr[tmp.length()];
  tmp.toCharArray(carr, tmp.length());
  broker.publish("bme680/temperature", carr);
}

void displayHumidity(String humidity)
{
  lcd.setCursor(0, 1);
  lcd.write(2);
  lcd.print(humidity + "%");
  char carr[humidity.length()];
  humidity.toCharArray(carr, humidity.length());
  broker.publish("bme680/humidity", carr);
}

void displayPressure(String pressure)
{
  lcd.setCursor(12, 1);
  lcd.print(pressure);
  lcd.write(3);
  char carr[pressure.length()];
  pressure.toCharArray(carr, pressure.length());
  broker.publish("bme680/pressure", carr);
}

void displayCO2(String co)
{
  lcd.setCursor(0, 3);
  lcd.print("CO2 " + co + "ppm");
  char carr[co.length()];
  co.toCharArray(carr, co.length());
  broker.publish("bme680/co", carr);
}

void displayVOC(String voc)
{
  lcd.setCursor(0, 2);
  lcd.print("VOC " + voc + "ppm");
  char carr[voc.length()];
  voc.toCharArray(carr, voc.length());
  broker.publish("bme680/voc", carr);
}

void displaySensorPersisted()
{
  lcd.setCursor(19,3);
  lcd.print(sensorPersisted);
}

void connectToNetwork() {
  WiFi.disconnect();
  lcd.setCursor(0, 0);
  lcd.print("Connecting to");
  lcd.setCursor(0, 1);
  lcd.print(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, passphrase);
  int wait = 0;
  while (WiFi.status() != WL_CONNECTED && wait < 30) {
    delay(1000);
    Serial.println("Establishing connection to WiFi..");
    wait++;
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Abandoning attempt, retrying");
    connectToNetwork();
  }
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Connected");
  Serial.println("Connected to network");
  delay(1000);
  lcd.clear();
}

void reconnectToBroker() {
  // Loop until we're reconnected
  if (WiFi.status() != WL_CONNECTED) {
    lcd.clear();
    connectToNetwork();
  }
  while (!broker.connected()) {
    lcd.setCursor(0, 0);
    lcd.print("Connecting MQTT");
    lcd.setCursor(0, 1);
    lcd.print(mqtt_server);
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "aqm-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (broker.connect(clientId.c_str())) {
      Serial.println("connected to broker");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Connected");
      delay(1000);
    } else {
      lcd.setCursor(0,2);
      lcd.print("Broker failed");
      Serial.print("failed connecting to broker, rc=");
      Serial.print(broker.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
    lcd.clear();
  }
}

void createLCDSymbols() {
    byte iaqsymbol[] = {
    0x04,
    0x0A,
    0x1F,
    0x11,
    0x0E,
    0x0A,
    0x0E,
    0x02
  };
  lcd.createChar(0, iaqsymbol);
  byte tempSymbol[] = {
    0x18,
    0x18,
    0x07,
    0x04,
    0x04,
    0x04,
    0x04,
    0x07
  };
  lcd.createChar(1, tempSymbol);
  byte humiditySymbol[] = {
    0x04,
    0x04,
    0x0A,
    0x0A,
    0x11,
    0x11,
    0x0A,
    0x04
  };
  lcd.createChar(2, humiditySymbol);
  byte airPressureSymbol[] = {
    0x04,
    0x15,
    0x0E,
    0x04,
    0x01,
    0x1E,
    0x01,
    0x1E
  };
  lcd.createChar(3, airPressureSymbol);
}

// loadState attempts to read the BSEC state from the EEPROM. If the state isn't there yet - it wipes that area of the EEPROM ready to be written to in the future. It'll also set the global variable 'sensorPersisted' to a space, so that the question mark disappears forever from the LCD.
void loadState(void)
{
  if (EEPROM.read(0) == BSEC_MAX_STATE_BLOB_SIZE) {
    // Existing state in EEPROM
    Serial.println("Reading state from EEPROM");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++) {
      bsecState[i] = EEPROM.read(i + 1);
      Serial.println(bsecState[i], HEX);
    }
    sensorPersisted = " ";
    iaqSensor.setState(bsecState);
    checkIaqSensorStatus();
  } else {
    // Erase the EEPROM with zeroes
    Serial.println("Erasing EEPROM");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE + 1; i++)
      EEPROM.write(i, 0);

    EEPROM.commit();
  }
}

// updateState waits for the in air quality accuracy to hit '3' - and will then write the state to the EEPROM. Then on every STATE_SAVE_PERIOD, it'll update the state.
void updateState(void)
{
  bool update = false;
  /* Set a trigger to save the state. Here, the state is saved every STATE_SAVE_PERIOD with the first state being saved once the algorithm achieves full calibration, i.e. iaqAccuracy = 3 */
  if (stateUpdateCounter == 0) {
    if (iaqSensor.iaqAccuracy >= 3) {
      update = true;
      stateUpdateCounter++;
    }
  } else {
    /* Update every STATE_SAVE_PERIOD milliseconds */
    if ((stateUpdateCounter * STATE_SAVE_PERIOD) < millis()) {
      update = true;
      stateUpdateCounter++;
    }
  }

  if (update) {
    iaqSensor.getState(bsecState);
    checkIaqSensorStatus();

    Serial.println("Writing state to EEPROM");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE ; i++) {
      EEPROM.write(i + 1, bsecState[i]);
      Serial.println(bsecState[i], HEX);
    }

    EEPROM.write(0, BSEC_MAX_STATE_BLOB_SIZE);
    EEPROM.commit();
  }
}
