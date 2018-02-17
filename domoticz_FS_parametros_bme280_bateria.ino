#include <FS.h>                   //this needs to be first, or it all crashes and burns...
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <WiFiUdp.h>
#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <ArduinoJson.h>

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define SEALEVELPRESSURE_HPA (1013.25)
#define SLEEP_TIME 300
//https://github.com/bblanchon/ArduinoJson
ADC_MODE(ADC_VCC);
unsigned int localPort = 2390;      // local port to listen on

char packetBuffer[255]; //buffer to hold incoming packet
char  ReplyBuffer[] = "acknowledged";       // a string to send back

Adafruit_BME280 bme; // I2C

WiFiUDP Udp;
DNSServer dnsServer;
const byte DNS_PORT = 53;
//define your default values here, if there are different values in config.json, they are overwritten.
//length should be max size + 1
char domoticz_server[40];
char domoticz_port[6] = "8080";
char domoticz_id[4] = "";
//default custom static IP
char static_ip[16] = "10.0.1.56";
char static_gw[16] = "10.0.1.1";
char static_sn[16] = "255.255.255.0";
int port = 8080;
int pwrPin = 12;
//flag for saving data
bool shouldSaveConfig = false;
WiFiClient client;
//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void setup() {
  // put your setup code here, to run once:
  pinMode(pwrPin, OUTPUT);

  pinMode(0, INPUT);
  Serial.begin(115200);
  Serial.println();
  Wire.begin(5, 4);
  bool status;

  // default settings
  status = bme.begin(0x76);
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  //clean FS, for testing
  //SPIFFS.format();

  //read configuration from FS json
  Serial.println("mounting FS...");

  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nparsed json");

          strcpy(domoticz_server, json["domoticz_server"]);
          strcpy(domoticz_port, json["domoticz_port"]);
          strcpy(domoticz_id, json["domoticz_id"]);

          if (json["ip"]) {
            Serial.println("setting custom ip from config");
            //static_ip = json["ip"];
            strcpy(static_ip, json["ip"]);
            strcpy(static_gw, json["gateway"]);
            strcpy(static_sn, json["subnet"]);
            //strcat(static_ip, json["ip"]);
            //static_gw = json["gateway"];
            //static_sn = json["subnet"];
            Serial.println(static_ip);
            /*            Serial.println("converting ip");
                        IPAddress ip = ipFromCharArray(static_ip);
                        Serial.println(ip);*/
          } else {
            Serial.println("no custom ip in config");
          }
        } else {
          Serial.println("failed to load json config");
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
  //end read
  Serial.println(static_ip);
  Serial.println(domoticz_server);
  Serial.println(domoticz_id);


  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter custom_domoticz_server("server", "Servidor Domoticz", domoticz_server, 40);
  WiFiManagerParameter custom_domoticz_port("port", "Puerto Domoticz", domoticz_port, 5);
  WiFiManagerParameter custom_domoticz_id("id", "Id de dispositivo", domoticz_id, 34);

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //set static ip
  IPAddress _ip, _gw, _sn;
  _ip.fromString(static_ip);
  _gw.fromString(static_gw);
  _sn.fromString(static_sn);

  wifiManager.setSTAStaticIPConfig(_ip, _gw, _sn);

  //add all your parameters here
  wifiManager.addParameter(&custom_domoticz_server);
  wifiManager.addParameter(&custom_domoticz_port);
  wifiManager.addParameter(&custom_domoticz_id);

  //reset settings - for testing
  //wifiManager.resetSettings();

  //set minimu quality of signal so it ignores AP's under that quality
  //defaults to 8%
  wifiManager.setMinimumSignalQuality();

  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  //wifiManager.setTimeout(120);

  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect("AutoConnectAP", "password")) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");
  dnsServer.start(DNS_PORT, WiFi.hostname(), WiFi.localIP());

  //read updated parameters
  strcpy(domoticz_server, custom_domoticz_server.getValue());
  strcpy(domoticz_port, custom_domoticz_port.getValue());
  strcpy(domoticz_id, custom_domoticz_id.getValue());

  //save the custom parameters to FS
  if (shouldSaveConfig) {
    Serial.println("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["domoticz_server"] = domoticz_server;
    json["domoticz_port"] = domoticz_port;
    json["domoticz_id"] = domoticz_id;

    json["ip"] = WiFi.localIP().toString();
    json["gateway"] = WiFi.gatewayIP().toString();
    json["subnet"] = WiFi.subnetMask().toString();

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }

    json.prettyPrintTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save
  }

  Serial.println("local ip");
  Serial.println(WiFi.localIP());
  Serial.println(WiFi.gatewayIP());
  Serial.println(WiFi.subnetMask());
  Serial.println(WiFi.hostname());
  Udp.begin(localPort);
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1, // temperature
                  Adafruit_BME280::SAMPLING_X1, // pressure
                  Adafruit_BME280::SAMPLING_X1, // humidity
                  Adafruit_BME280::FILTER_OFF   );

}

void loop() {
  //---------------------------------------------------------------------------------------------------------------------------------
  //---------------------------------------------------------------------------------------------------------------------------------
  if ( digitalRead(0) == LOW ) {
    //WiFiManager
    //Local intialization. Once its business is done, there is no need to keep it around
    WiFiManager wifiManager;

    //reset settings - for testing
    Serial.println("Reseteando a valores de fabrica");
    wifiManager.resetSettings();
    delay(1000);
    ESP.reset();
    //sets timeout until configuration portal gets turned off
    //useful to make it all retry or go to sleep
    //in seconds
    //wifiManager.setTimeout(120);

    //it starts an access point with the specified name
    //here  "AutoConnectAP"
    //and goes into a blocking loop awaiting configuration

    //WITHOUT THIS THE AP DOES NOT SEEM TO WORK PROPERLY WITH SDK 1.5 , update to at least 1.5.1
    //WiFi.mode(WIFI_STA);

    if (!wifiManager.startConfigPortal("OnDemandAP")) {
      Serial.println("failed to connect and hit timeout");
      delay(3000);
      //reset and try again, or maybe put it to deep sleep
      ESP.reset();
      delay(5000);
    }

    //if you get here you have connected to the WiFi
    Serial.println("connected...yeey :)");
  }
  //---------------------------------------------------------------------------------------------------------------------------------
  digitalWrite(pwrPin, HIGH);
  bme.takeForcedMeasurement();
  dnsServer.processNextRequest();
  Serial.print("Intentando conectar a ");
  Serial.print (domoticz_server);
  //Serial.println (int(domoticz_port));
  if (client.connect(domoticz_server, 8080) ) {
    long rssi = WiFi.RSSI();

    String json1 = "GET /json.htm?type=command&param=udevice&idx=";
    json1 += String(domoticz_id);
    json1 += "&nvalue=0&svalue=";
    json1 += String(bme.readTemperature());
    json1 += ";";
    json1 += String(bme.readHumidity());
    json1 += ";0;";
    json1 += String(bme.readPressure() / 100.0F);
    json1 += ";0";
    json1 += " HTTP/1.0";
    client.println(json1);
    client.println();
    Serial.println (json1);
    while (client.available()) {
      String line = client.readStringUntil('\r');
      Serial.print(line);

    } client.stop();
  }




  IPAddress BcastIP = WiFi.localIP();
  BcastIP[3] = 255;
  Udp.beginPacket(BcastIP, 2391);
  Udp.write("Bme 280 Domoticz Bateria+control avanzado del sensor.");
  Udp.endPacket();
  //


  uint32_t getVcc = ESP.getVcc();
  if (client.connect(domoticz_server, 8080) ) {
    String url = "/json.htm?type=command&param=udevice&idx=11&nvalue=0&svalue=";
    url += (getVcc / 1000.000F);
    Serial.println(url);
    client.println(String("GET ") + url + " HTTP/1.0");
    client.println();
    while (client.available()) {
      String line = client.readStringUntil('\r');
      Serial.print(line);

    } client.stop();
  }
  // put your main code here, to run repeatedly:
  ESP.deepSleep(SLEEP_TIME * 1000000, WAKE_RF_DEFAULT);
  delay(2000);


}
