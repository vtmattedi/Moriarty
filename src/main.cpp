#include <lvgl.h>
#include <TFT_eSPI.h>
#include <../lib/ui/ui.h>
#include <WiFi.h>
#include <Wire.h>
#include <ArduinoOTA.h>
#include <../lib/boardstuff/boardstuff.h>
#include <../lib/pubsubclient-2.8/src/PubSubClient.h>
#include <../include/Creds/WifiCred.h>
#include <../include/Creds/HiveMQCred.h>
#include <../lib/Time-master/TimeLib.h>
#include <NightMareTCP.h>
#include <NightMareNetwork.h>
#include <ESPmDNS.h>
#include <HTTPClient.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <UserDefines.h>
#include <Services.h>
StaticJsonDocument<512> DocJson;
SemaphoreHandle_t xSemaphore_tcp = NULL;
StaticSemaphore_t xSemaphore_tcp_Buffer;

int blink = false;
// #define COMPILE_SERIAL
bool done = false;
bool done2 = false;
bool sleeping = false;
bool deepsleeping = false;
uint32_t colorOverridenMode = false;
uint16_t seconds_to_sleep = 60;
uint32_t _lastSetCalled = 0;
#define DEVICE_NAME "Moriarty"
#define SERVER_HOST "Feymann"
NightMareTCPClient tcpClient(DEVICE_NAME, true);
void handle_gateway(const char *topic, String incommingMessage);
/*Don't forget to set Sketchbook location in File/Preferencesto the path of your UI project (the parent foder of this INO file)*/
WiFiClientSecure hive_client;
PubSubClient HiveMQ(hive_client);
/*Change to your screen resolution*/
static const uint16_t screenWidth = 240;
static const uint16_t screenHeight = 320;
void handleSetModeColors();
void setTemp();

struct Internals
{
  bool sleeping = false;
  bool deepsleeping = false;
  bool updating = false;
  uint32_t seconds_to_sleep = 60;
  bool tcp_state = false;
  bool mqtt_state = false;
  uint32_t wake_up_millis = 0;
  bool enable_touch = false;
  uint32_t boot = 0;
  bool fs_mount = false;
  double _lastTemperature = 0;
  int32_t angle = 0;
  bool angle_backwards = false;
} control_variables;

struct Configuration
{
  bool use_led = false;
  byte brigtness = 0xff;
  bool turnoff_on_sleep = false;

  void Save()
  {
    if (!control_variables.fs_mount)
    {
      Serial.printf("...FS is not mounted...\n");
      return;
    }
  }

  void Load()
  {
    if (!control_variables.fs_mount)
    {
      Serial.printf("...FS is not mounted...\n");
      return;
    }
  }
} Configs;

struct TBlynker
{
  bool _state = false;
  uint32_t interval = 500;

} Blynker;

void Set_led(uint32_t color, bool force = false)
{
  if (Configs.use_led || force)
  {
    set_led(color);
  }
}

/// @brief Gets the MQTT topic in the proper format ie. with 'DEVICE_NAME/' before the topic.
/// @param topic The topic without the owner prefix.
/// @return The topic with the owner prefix.
String MqttTopic(String topic)
{
#ifndef DEVICE_NAME
#error Please define device name
#endif
  String topicWithOwner = "";
  topicWithOwner += DEVICE_NAME;
  if (topic != "" || topic != NULL)
  {
    if (topic[0] != '/')
      topicWithOwner += "/";
    topicWithOwner += topic;
  }
  return topicWithOwner;
}

/// @brief Sends an message to the Mqtt Server
/// @param topic The topic to the message
/// @param message The message
/// @param insertOwner Use the MqttTopic function to insert device's name before the topic.
/// @param retained Retained message or normal
void MqttSend(String topic, String message, bool insertOwner = true, bool retained = false)
{

  if (insertOwner)
    topic = MqttTopic(topic);
  Serial.printf("[TCP %d][MQTT %d] -- %s, %s\n", control_variables.tcp_state, control_variables.mqtt_state, topic.c_str(), message.c_str());
  if (control_variables.tcp_state)
    tcpClient.send("topic::" + topic + "payload::" + message);
  else if (control_variables.mqtt_state)
  {
    HiveMQ.publish(topic.c_str(), message.c_str(), retained);
  }
}

void setAcColors()
{
  lv_color_t color = lv_color_hex(COLOR_DARK_RED); // Dark RED
  if (Ac.SetTemp.stale)
  {
    lv_obj_set_style_text_color(ui_lbTemp, lv_color_hex(COLOR_DARK_RED), LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_lbTempSecondary, lv_color_hex(COLOR_WHITE), LV_STATE_DEFAULT);
    lv_label_set_text(ui_lbConstAcPower, "Power");
    color = lv_color_hex(COLOR_GREY); // grey
    lv_obj_set_style_bg_color(ui_btAcPower, color, LV_STATE_DEFAULT);
  }
  else if (Ac.SetTemp.value > 0)
  {
    lv_label_set_text(ui_lbConstAcPower, "Stop");
    lv_obj_set_style_bg_color(ui_btAcPower, color, LV_STATE_DEFAULT);
    color = lv_color_hex(COLOR_LIGHT_BLUE); // light blue
    lv_obj_set_style_text_color(ui_lbTemp, color, LV_STATE_DEFAULT);
  }
  else
  {
    lv_label_set_text(ui_lbConstAcPower, "Power");
    color = lv_color_hex(COLOR_DEFAULT_BLUE); // default blue
    lv_obj_set_style_bg_color(ui_btAcPower, color, LV_STATE_DEFAULT);
    lv_color_t color = lv_color_hex(COLOR_WHITE); // white
    lv_obj_set_style_text_color(ui_lbTemp, color, LV_STATE_DEFAULT);
  }

  setTemp();
}

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[screenWidth * screenHeight / 10];

TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight); /* TFT instance */

struct Timers_Last_Values
{
  uint32_t last_touch = 0;
  uint32_t sync_time = 0;
  uint32_t mqtt_publish = 0;
  uint32_t mqtt_reconnect = 0;
  uint32_t panel_update = 0;
  uint32_t Tasks = 0;
  uint32_t soft_ldr = 0;
};
Timers_Last_Values oldTimers;

/// @brief Structure with name, value and timestamp of sensor (info from MQTT).
struct SensorData
{
  String name = "unused";
  double value = -1;
  uint timestamp = 0;
  /// @brief Updates the value of the sensor and the timestamp.
  /// @param newValue the new value of the sensor.
  void update(double newValue)
  {
    value = newValue;
    timestamp = now();
#ifdef COMPILE_SERIAL
    Serial.printf("[%s] updated [%2.2f]\n", name.c_str(), value);
#endif
  }

  String fname()
  {
    int _index = name.indexOf("/");
    int _last = name.lastIndexOf("/");
    if (_index < 0 || _last < 0)
      return name;
    else
    {
      return "(" + name.substring(0, _index) + ") " + name.substring(_last);
    }
  }

  /// @brief Generate a String representing the sensor data holding name value and timestamp values.
  /// @return A String that can be parsed somewhere else or by fromString().
  String toString()
  {
    String msg = "name: ";
    msg += name;
    msg += "\nvalue: ";
    msg += String(value, 1);
    msg += "\ntimestamp: ";
    msg += timestamp;
    return msg;
  }
  /// @brief restore a sensor from a String generated by toString().
  /// @param str the String with the sensor data.
  void fromString(const String &str)
  {
    int nameIndex = str.indexOf("name: ") + 6;
    int valueIndex = str.indexOf("value: ") + 7;
    int newlineIndex = str.indexOf('\n', valueIndex);
    name = str.substring(nameIndex, valueIndex - 8);
    value = str.substring(valueIndex, newlineIndex).toDouble();
    int timestampIndex = str.indexOf("timestamp: ") + 11;
    timestamp = strtoul(str.substring(timestampIndex).c_str(), NULL, 10);
  }
  /// @brief resets the Sensor object
  void reset()
  {
    name = "unused";
    value = -1;
    timestamp = 0;
  }
};

/// @brief Sensor struct to hold sensor a num of sensors defined in NUM_OF_SENSORS.
struct sensorsStruct
{
#define NUM_OF_SENSORS 20
  /// @brief Array to hold the sensors data
  SensorData _sensors[NUM_OF_SENSORS];

  /// @brief Creates or gets the index in the sensor array.
  /// @param name The name to be found on the array or to be created.
  /// @return -1 if could not be created nor was found or the index of the created or found sensor.
  int CreateOrGetIndex(const String &name)
  {
    for (size_t i = 0; i < NUM_OF_SENSORS; i++)
    {
      if (_sensors[i].name == name)
        return i;
    }
    for (size_t i = 0; i < NUM_OF_SENSORS; i++)
    {
      if (_sensors[i].name == "unused")
      {
        _sensors[i].name = name;
#ifdef COMPILE_SERIAL
        Serial.printf("sensor '%s' created at '%d'\n", _sensors[i].name.c_str(), i);
#endif
        return i;
      }
    }
    return -1;
  }

  /// @brief Checks if there is a sensor with that name on the array.
  /// @param name The name of the sensor
  /// @return The index of the sensor or -1 if it does not exist.
  int Exists(const String &name)
  {
    for (size_t i = 0; i < NUM_OF_SENSORS; i++)
    {
      if (_sensors[i].name == name)
        return i;
    }
    return -1;
  }

  /// @brief Checks if there is a valid sensor in the index on the array.
  /// @param index The index of the array sensor.
  /// @return True if there is a sensor at index that is been used.
  bool Exists(const int index)
  {
    if (index < 0 || index >= NUM_OF_SENSORS)
      return false;
    if (_sensors[index].name.equals("unused"))
      return false;
    else
      return true;
  }

  /// @brief Deletes a sensor by reseting.
  /// @param name The name of the sensor to be reseted.
  /// @return False if name was not found true if it was and sensor was reset.
  bool DeleteByName(const String &name)
  {
    for (size_t i = 0; i < NUM_OF_SENSORS; i++)
    {
      if (_sensors[i].name == name)
      {
        DeleteAt(i);
        return true;
      }
    }
    return false;
  }

  /// @brief Deletes a sensor by resetingit.
  /// @param index The index of the sensor to be reseted.
  /// @return False if index is invalid true otherwise.
  bool DeleteAt(int index)
  {
    if (index < 0 || index >= NUM_OF_SENSORS)
      return false;
    _sensors[index].reset();
    return true;
  }

  /// @brief Resets all sensors in the array.
  void Reset()
  {
    for (size_t i = 0; i < NUM_OF_SENSORS; i++)
    {
      _sensors[i].reset();
    }
  }

  /// @brief Gets all the used sensors on the array.
  /// @return All the sensors toString() separeted by a ';' char.
  String ToString()
  {
    String message = "";
    for (size_t i = 0; i < NUM_OF_SENSORS; i++)
    {
      if (_sensors[i].name != "unused")
      {
        message += _sensors[i].toString();
        message += ";";
      }
    }
    return message;
  }
};

/// @brief The sensor object that holds the sensor array.
sensorsStruct Sensors;

void sleep(bool value, bool lowpower = false)
{
  if (control_variables.sleeping && !value)
  {
    // wakeup command
    control_variables.wake_up_millis = millis() + 200;
    if (lowpower)
    {
      tft.writecommand(0x11);
      delay(175);
    }
    set_backlight(255);
    control_variables.sleeping = false;
  }
  if (value && !control_variables.sleeping)
  {
    // sleep command
    set_backlight(32);
    lv_obj_add_flag(ui_pnConfig, LV_OBJ_FLAG_HIDDEN);
    _ui_screen_change(&ui_lockScreen, LV_SCR_LOAD_ANIM_NONE, 500, 0, &ui_lockScreen_screen_init);

    if (lowpower)
    {
      tft.writecommand(0x10);
    }
    control_variables.wake_up_millis = UINT32_MAX;
    control_variables.enable_touch = false;
    control_variables.sleeping = true;
    //  Set_led(0);
  }
}

#pragma region OTA
void startOTA()
{
  String type;
  // is_updating = true;
  //  caso a atualização esteja sendo gravada na memória flash externa, então informa "flash"
  if (ArduinoOTA.getCommand() == 0)
    type = "flash";
  else                   // caso a atualização seja feita pela memória interna (file system), então informa "filesystem"
    type = "filesystem"; // U_SPIFFS
  // exibe mensagem junto ao tipo de gravação
  Serial.println("Start updating " + type);
  control_variables.updating = true;
  lv_obj_set_style_bg_color(ui_arcTemp, lv_color_hex(0x00FF00), LV_STATE_DEFAULT);
  lv_label_set_text(ui_lbTempSecondary, "UPT");
  lv_obj_add_flag(ui_btAcPower, LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_flag(ui_btColorAuto, LV_OBJ_FLAG_HIDDEN);
}
// exibe mensagem
void endOTA()
{
  lv_label_set_text(ui_lbTempSecondary, "DONE");
  Serial.println("\nEnd");
  control_variables.updating = false;
}
// exibe progresso em porcentagem
void progressOTA(unsigned int progress, unsigned int total)
{
  byte update_progress = round((float)progress / total * 100);
  lv_arc_set_value(ui_arcTemp, update_progress);
  lv_label_set_text(ui_lbTemp, String(update_progress + "%").c_str());
  Serial.printf("Progress: %u%%\r\n", (progress / (total / 100)));
}

void errorOTA(ota_error_t error)
{
  lv_label_set_text(ui_lbTempSecondary, "FAIL");
  Serial.printf("Error[%u]: ", error);
  if (error == OTA_AUTH_ERROR)
    Serial.println("Auth Failed");
  else if (error == OTA_BEGIN_ERROR)
    Serial.println("Begin Failed");
  else if (error == OTA_CONNECT_ERROR)
    Serial.println("Connect Failed");
  else if (error == OTA_RECEIVE_ERROR)
    Serial.println("Receive Failed");
  else if (error == OTA_END_ERROR)
    Serial.println("End Failed");
  control_variables.updating = false;
  lv_obj_clear_flag(ui_btAcPower, LV_OBJ_FLAG_HIDDEN);
  lv_obj_clear_flag(ui_btColorAuto, LV_OBJ_FLAG_HIDDEN);
}
#pragma endregion

enum TimeStampFormat
{
  DateAndTime,
  OnlyDate,
  OnlyTime,
  DowDate
};

// @brief Converts a timestamp to a date and time String.
// @param timestamp The unix timestamp to be parsed.
// @param _format Whether you want Date, Time or Both.
// @return a String with the date on the timestamp formatted as HH:mm DD/MM/YY, attending to the TimeStampFormat passed.
String timestampToDateString(uint32_t timestamp, const TimeStampFormat _format = DateAndTime)
{
  // Convert the timestamp to a time_t object.
  time_t timeObject = timestamp;

  // Extract the date components from the time_t object.
  int _year = year(timeObject) % 100;
  int _month = month(timeObject);
  int _day = day(timeObject);

  int _hour = hour(timeObject);
  int _minute = minute(timeObject);
  uint8_t dow = dayOfWeek(timeObject);

  String dateString = "";
  // Adds Time
  if (_format == OnlyTime || _format == DateAndTime)
  {
    dateString = String(_hour, DEC).length() == 1 ? "0" + String(_hour, DEC) : String(_hour, DEC);
    if (now() % 2 == 0)
      dateString += ":";
    else
      dateString += " ";
    dateString += String(_minute, DEC).length() == 1 ? "0" + String(_minute, DEC) : String(_minute, DEC);
  }
  if (_format == DowDate)
  {
    dateString += dayShortStr(dow);

    // // switch (dow)
    // {
    // case 1:
    //   dateString += "Sun";
    //   break;
    // case 2:
    //   dateString += "Mon";
    //   break;
    // case 3:
    //   dateString += "Tue";
    //   break;
    // case 4:
    //   dateString += "Wed";
    //   break;
    // case 5:
    //   dateString += "Thu";
    //   break;
    // case 6:
    //   dateString += "Fri";
    //   break;
    // case 7:
    //   dateString += "Sat";
    //   break;
    // default:
    //   dateString += "---";
    // }
  }

  // Adds a Space between Time and Date
  if (_format == DateAndTime || _format == DowDate)
    dateString += " ";
  // Adds Date
  if (_format == OnlyDate || _format == DateAndTime || _format == DowDate)
  {
    dateString += String(_day, DEC).length() == 1 ? "0" + String(_day, DEC) : String(_day, DEC);
    dateString += "/";
    dateString += String(_month, DEC).length() == 1 ? "0" + String(_month, DEC) : String(_month, DEC);
    if (_format != DowDate)
    {
      dateString += "/";
      dateString += String(_year, DEC);
    }
  }
  return dateString;
}

String sizeLimit(String msg, uint8_t size)
{
  int diff = msg.length() - size;
  if (diff == 0)
    return msg;

  if (diff < 0)
  {
    while (diff < 0)
    {
      msg = " " + msg;
    }
    return msg;
  }
  else
  {
    return msg.substring(0, size);
  }
}

float mapFloat(float x, float inMin, float inMax, float outMin, float outMax)
{
  // Ensure x is within the input range
  x = constrain(x, inMin, inMax);

  // Map x from the input range to the output range
  float mappedValue = (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;

  return mappedValue;
}

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, w, h);
  tft.pushColors((uint16_t *)&color_p->full, w * h, true);
  tft.endWrite();

  lv_disp_flush_ready(disp);
}

/*Read the touchpad*/
void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
  uint16_t touchX = 0, touchY = 0;

  bool touched = tft.getTouch(&touchX, &touchY, 600);
#if ROTATION == ROTATION_PORTRAIT_INV
  touchY = 320 - touchY;
#endif
  if (!touched)
  {
    data->state = LV_INDEV_STATE_REL;
  }
  else
  {
    oldTimers.last_touch = millis();
    sleep(false);
    data->state = LV_INDEV_STATE_PR;

    /*Set the coordinates*/
    data->point.x = touchX;
    data->point.y = touchY;

    Serial.print("Data x ");
    Serial.println(touchX);

    Serial.print("Data y ");
    Serial.println(touchY);
  }
}

/// @brief Attempts to connect, reconnect to the MQTT broker .
/// @return True if successful or false if not.
bool MQTT_Reconnect()
{
  String clientID = DEVICE_NAME;
  clientID += String(random(), HEX);
  if (HiveMQ.connect(clientID.c_str(), MQTT_USER, MQTT_PASSWD))
  {
    bool sub = HiveMQ.subscribe("#");
#ifdef COMPILE_SERIAL
    Serial.printf("MQTT Connected subscribe was: %s\n", sub ? "success" : "failed");
#endif
    return true;
  }
  else
  {
#ifdef COMPILE_SERIAL
    Serial.printf("Can't Connect to MQTT Error Code : %d\n", HiveMQ.state());
#endif
    return false;
  }
}

void assertServer(bool result)
{
  Serial.printf("assert was %d\n", result);
}

// Sherlock/Lights
void setTemp()
{
  // Stale
  if (Ac.SetTemp.stale)
  {
    lv_label_set_text(ui_lbTemp, "--");
    lv_arc_set_value(ui_arcTemp, mapFloat(0, 15, 38, 0, 100));
    lv_label_set_text(ui_lbTempSecondary, "--");
    return;
  }

  auto temp_str = String(control_variables._lastTemperature);
  bool amIint = Ac.SetTemp.value - floor(Ac.SetTemp.value) == 0;
  auto set_str = String(Ac.SetTemp.value).substring(0, amIint ? 2 : 4);

  if (temp_str.length() > 4)
  {
    temp_str = temp_str.substring(0, 4);
  }
  // Ac is SET
  if (Ac.SetTemp.value > 0)
  {
    lv_label_set_text(ui_lbTemp, set_str.c_str());
    lv_arc_set_value(ui_arcTemp, mapFloat(Ac.SetTemp.value, 15, 38, 0, 100));
    temp_str += "°C";
    lv_label_set_text(ui_lbTempSecondary, temp_str.c_str());
    lv_obj_set_style_text_color(ui_lbTempSecondary, lv_color_hex(COLOR_WHITE), LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_lbTemp, lv_color_hex(COLOR_LIGHT_BLUE), LV_STATE_DEFAULT);
  }
  else // AC is normal
  {
    lv_label_set_text(ui_lbTemp, temp_str.c_str());
    lv_arc_set_value(ui_arcTemp, mapFloat(control_variables._lastTemperature, 15, 38, 0, 100));
    lv_label_set_text(ui_lbTempSecondary, String(Ac.Temperature.value > 0 ? Ac.Temperature.value : -1 * Ac.Temperature.value).c_str());
    lv_obj_set_style_text_color(ui_lbTemp, lv_color_hex(COLOR_WHITE), LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_lbTempSecondary, lv_color_hex(Ac.Temperature.value > 0 ? COLOR_LIGHT_BLUE : COLOR_WHITE), LV_STATE_DEFAULT);
  }
}

void handle_color_auto2(lv_event_t *e)
{
  int state = 2;
  if (BedLights.AutomationRestore.value != 0)
  {
    state = BedLights.LightState.value;
  }

  if (state == 0)
  {
    BedLights.AutomationRestore.change(now() + 3600);
    BedLights.LightState.change(true);
    MqttSend("Sherlock/Lights", "u1", false);
  }
  else if (state == 1)
  {
    BedLights.AutomationRestore.change(0);
    MqttSend("Sherlock/Lights", "auto", false);
  }
  else if (state == 2)
  {
    BedLights.AutomationRestore.change(now() + 3600);
    BedLights.LightState.change(false);
    MqttSend("Sherlock/Lights", "u0", false);
  }
}

void handle_ac(lv_event_t *e)
{
  byte *type = (byte *)lv_event_get_user_data(e);
  if (!type)
    return;
  if (*type == AC_BUTTON_POWER)
  {
    if (Ac.SetTemp.value < 0)
    {
      MqttSend("Adler/console/in", "send POWER", false);
      Ac.Temperature.change(Ac.Temperature.value * -1);
    }
    else
    {
      MqttSend("Adler/console/in", "target 0", false);
      Ac.SetTemp.change(Ac.SetTemp.value * -1);
    }
  }
  else if (*type == AC_BUTTON_PLUS || *type == AC_BUTTON_MINUS)
  {
    bool plus = *type == AC_BUTTON_PLUS;
    if (Ac.SetTemp.value > 0)
    {
      Ac.SetTemp.change(Ac.SetTemp.value += plus ? 0.5 : -0.5);
    }
    else
      MqttSend("Adler/console/in", "send " + String(plus ? "PLUS" : "MINUS"), false);
  }
  else if (*type == AC_BUTTON_SET)
  {
    MqttSend("Adler/console/in", "target " + String(Ac.SetTemp.value > 0 ? Ac.SetTemp.value : 0), false);
    Ac.SetTemp.change(Ac.SetTemp.value * -1);
  }
}

/// @brief Gets if the topic have been publish as a sensor a.k.a. have '/sensors/' on the topic.
/// @param topic The topic name.
/// @return True if topic has '/sensors/', false otherwise.
bool isMqttSensor(String topic)
{
#ifdef COMPILE_SERIAL
  Serial.printf("Is topic '%s' a sensor?? - ", topic.c_str());
#endif
  if (topic.indexOf(String(DEVICE_NAME)) >= 0)
  {
#ifdef COMPILE_SERIAL
    Serial.println("own device");
#endif
    return false;
  }
  if (topic.indexOf("/sensors/") >= 0)
  {
#ifdef COMPILE_SERIAL
    Serial.println(" IT is!");
#endif
    return true;
  }
#ifdef COMPILE_SERIAL
  Serial.println(" none of the above");
#endif
  return false;
}

void HandleMqtt(char *topic, byte *payload, unsigned int length)
{
  Set_led(0x0000FF);
  String incommingMessage = "";
  for (int i = 0; i < length; i++)
    incommingMessage += (char)payload[i];
  handle_gateway(topic, incommingMessage);
  Set_led(0);
}

String HandleTcp(String msg)
{
  Set_led(0xFF0000);
  int index = msg.indexOf(",");
  if (index > 0)
  {
    handle_gateway(msg.substring(0, index).c_str(), msg.substring(index + 1));
  }
  else
    Serial.printf("{%s}\n", msg.c_str());
  Set_led(0);
  return "";
}

void handle_gateway(const char *topic, String incommingMessage)
{
  if (strcmp("Adler/sensors/Temperature", topic) == 0)
  {
    control_variables._lastTemperature = incommingMessage.toDouble();
    setTemp();
  }
  if (strcmp("Adler/sensors/LDR", topic) == 0 && !sleeping)
  {
    int _val = incommingMessage.toInt();
    // //set_backlight(map(_val, 2000, 4095, 127, 255));
  }
  if (strcmp("Sherlock/state", topic) == 0)
  {
    deserializeJson(DocJson, incommingMessage);
    BedLights.AutomationRestore.handleServer(DocJson["Automation"]);
    BedLights.LightState.handleServer(DocJson["State"]);
    BedLights.Hue.handleServer(DocJson["Hue"]);
  }
  if (strcmp("Adler/state", topic) == 0)
  {
    deserializeJson(DocJson, incommingMessage);
    Ac.Temperature.handleServer(DocJson["Temp"]);
    Ac.Hsleep.handleServer(DocJson["Hsleep"]);
    Ac.Ssleep.handleServer(DocJson["Ssleep"]);
    Ac.SetTemp.handleServer(DocJson["Settemp"]);
  }

  if (isMqttSensor(topic))
  {
    int sensorIndex = Sensors.CreateOrGetIndex(topic);
    // Serial.printf("%s: %d\n", incommingMessage.c_str(), sensorIndex);
    if (sensorIndex >= 0)
    {
      Sensors._sensors[sensorIndex].update(incommingMessage.toDouble());
    }
  }
  String msg = "topic:'";
  msg += topic;
  msg += "',payload:'";
  msg += incommingMessage;
  msg += "'.";
  Serial.println(msg);
  // tcpServer.Broadcast(msg);
}

bool getTime()
{
  bool result = false;
#ifdef COMPILE_SERIAL
  Serial.println("Syncing Time Online");
#endif
  WiFiClient client;
  HTTPClient http;
  http.begin(client, "http://worldtimeapi.org/api/timezone/America/Bahia.txt"); // HTTP
  int httpCode = http.GET();
  // httpCode will be negative on error
  if (httpCode > 0)
  {
    // HTTP header has been send and Server response header has been handled
    // file found at server
    if (httpCode == HTTP_CODE_OK)
    {
#ifdef COMPILE_SERIAL
      Serial.printf("[HTTP] OK... code: %d\n", httpCode);
#endif
      String payload = http.getString();
      char str[payload.length() + 1];
      strcpy(str, payload.c_str());
      char *pch;
      pch = strtok(str, ":\n");
      int i = 0;
      int raw_offset = 0;
      while (pch != NULL)
      {
        i++;
        if (i == 23)
        {
          raw_offset = atoi(pch);
        }
        if (i == 27)
        {
          setTime(atoi(pch) + raw_offset);
        }
        // printf("%d: %s\n", i, pch);
        pch = strtok(NULL, ":\n");
      }
#ifdef COMPILE_SERIAL
      String msg = "Time Synced ";
      msg += millis();
      msg += "ms from boot.";
      Serial.println(msg);
#endif
      result = true;
    }
    else
    {
#ifdef COMPILE_SERIAL
      Serial.printf("[HTTP] Error code: %d\n", httpCode);
#endif
    }
  }
  else
  {
#ifdef COMPILE_SERIAL
    Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
#endif
  }
  http.end();
  return result;
}

void mqtt_reconect_task()
{
  if (HiveMQ.connected())
    return;
  control_variables.mqtt_state = MQTT_Reconnect();
}

void blink_led()
{
  blink++;
  if (blink >= 3)
    blink == 0;

  // if (blink == 0)
  //   Set_led(0xff, 0, 0);
  // if (blink == 1)
  //   Set_led(0, 0xff, 0);
  // if (blink == 2)
  //   Set_led(0, 0, 0xff);

  Set_led(0xff, true);
}

void set_time()
{
  lv_label_set_text(ui_lbTime, timestampToDateString(now() - 1, OnlyTime).c_str());

  String state_string = "Offline";
  int color = 0x088004;
  if (control_variables.tcp_state)
    state_string = "TCP";
  else if (control_variables.mqtt_state)
    state_string = "MQTT";
  else
    color = 0x591706;

  lv_obj_set_style_text_color(ui_lbInfo, lv_color_hex(color), LV_STATE_DEFAULT);
  lv_label_set_text(ui_lbInfo, state_string.c_str());

  auto boottime = now() - control_variables.boot;
  Configs.use_led ? lv_obj_add_state(ui_swLed, LV_STATE_CHECKED) : lv_obj_clear_state(ui_swLed, LV_STATE_CHECKED);
  String time = "";
  if (boottime < 90)
  {
    time += boottime;
    time += " sec.";
  }
  else if (boottime < 90 * 60)
  {
    time += boottime / 60;
    time += " min.";
  }
  else
  {
    time += boottime / 3600;
    time += " hours.";
  }
  lv_label_set_text(ui_lbBoot, time.c_str());
}

void tcp_reconnect()
{
  int a = millis();
  // Serial.printf("TCP: (%d) || MQTT: (%d) || UPDT: (%d) \n", control_variables.tcp_state, control_variables.mqtt_state, control_variables.updating);
  if (control_variables.updating)
    return;
  // Serial.printf("[RAW] TCP: (%d) || MQTT: (%d) \n", tcpClient.connected(), HiveMQ.connected());

  if (!tcpClient.connected())
  {
    IPAddress server_ip = MDNS.queryHost(SERVER_HOST);
    if (server_ip != IPAddress())
    {

      if (tcpClient.connect(server_ip))
      {
        tcpClient.setMode(TransmissionMode::SizeColon);
        tcpClient.send("REQ_UPDATES");
        if (HiveMQ.connected())
        {
          HiveMQ.disconnect();
          control_variables.mqtt_state = false;
        }
        control_variables.tcp_state = true;
      }
      else
      {
        control_variables.tcp_state = false;
      }
    }
    else
    {

      control_variables.tcp_state = false;
    }
  }
  if (!control_variables.tcp_state)
    mqtt_reconect_task();

  // Serial.printf("[RAW] TCP: (%d) || MQTT: (%d) \n", tcpClient.connected(), HiveMQ.connected());
}

void sync_server_variables()
{
  BedLights.sync();
  Ac.sync();
}

void set_angle()
{
#define max_angle 30
  lv_img_set_angle(ui_imgLogo, control_variables.angle += control_variables.angle_backwards ? -30 : 30);
  if (control_variables.angle >= max_angle * 10 || control_variables.angle <= max_angle * -10)
  {
    control_variables.angle_backwards = !control_variables.angle_backwards;
  }

  // control_variables.angle += 64;
  // if(control_variables.angle>256)
  // control_variables.angle = 128;
  // lv_img_set_zoom(ui_imgLogo,control_variables.angle);
}

void temperatureSend(double target)
{
  MqttSend("Adler/console/in", "target " + String(target), false);
}

void handle_color_auto()
{
  lv_color_t color = lv_color_hex(COLOR_DEFAULT_BLUE);
  if (BedLights.AutomationRestore.value > now())
  {
    lv_obj_clear_flag(ui_Bar1, LV_OBJ_FLAG_HIDDEN);
    lv_bar_set_value(ui_Bar1, map(BedLights.AutomationRestore.value - now(), 0, 3600, 0, 100), LV_ANIM_OFF);
    lv_label_set_text(ui_lbConstColorAuto, BedLights.LightState.value ? "On" : "Off");
  }
  else
  {
    lv_obj_add_flag(ui_Bar1, LV_OBJ_FLAG_HIDDEN);
    color = lv_color_hex(COLOR_LIGHT_GREEN);
    lv_label_set_text(ui_lbConstColorAuto, "Auto");
  }
  lv_obj_set_style_bg_color(ui_btColorAuto, color, LV_STATE_DEFAULT);
}

void ConnectivityHandle(void *args)
{
  for (;;)
  {
    if (xSemaphoreTake(xSemaphore_tcp, 1) == pdTRUE)
    {
      tcp_reconnect();
      xSemaphoreGive(xSemaphore_tcp);
    }
    // Serial.print(" tas: ");
    // auto res = uxTaskGetStackHighWaterMark(NULL);
    // Serial.println(res);
    yield();
    vTaskDelay(1000);
  }
}

void handle_config(lv_event_t *e)
{
  byte config = *(byte *)lv_event_get_user_data(e);
  if (config == CONFIG_LED_ENABLE)
  {
    Configs.use_led = lv_obj_get_state(ui_swLed) & LV_STATE_CHECKED;
  }
  else if (config == CONFIG_RESET)
  {
    ESP.restart();
  }
}

void set_boot_time()
{
}

void set_config()
{
  lv_label_set_text(ui_lbIP, WiFi.localIP().toString().c_str());
  int used_fs = round((float)(LittleFS.usedBytes()) / LittleFS.totalBytes() * 100);
  int used_heap = round((float)(ESP.getFreeHeap()) / ESP.getHeapSize() * 100);
  lv_label_set_text(ui_lbRamValue, String(used_heap).c_str());
  lv_arc_set_value(ui_arcRam, used_heap);
  lv_label_set_text(ui_lbFSValue, String(used_fs).c_str());
  lv_arc_set_value(ui_arcFS, used_fs);

#ifdef USE_PSRAM
  auto used_ps = (float)(ESP.getFreePsram() / ESP.getPsramSize());
#endif
}

void setup()
{

  Serial.begin(115200); /* prepare for possible serial debug */
  board_init();
  set_backlight(0);
  Set_led(0x0000FF, true);
  Ac.SetTemp.send_delay = true;
  Ac.SetTemp.millis_to_delay = 1500;
  Ac.SetTemp.on_send = temperatureSend;
  Ac.SetTemp.on_value_changed = setAcColors;
  Ac.SetTemp.on_assert_result = assertServer;
  BedLights.AutomationRestore.on_value_changed = handle_color_auto;
  BedLights.LightState.on_value_changed = handle_color_auto;
  control_variables.fs_mount = LittleFS.begin(true);
  Serial.printf("...FS mount: %s...\n", control_variables.fs_mount ? "Success" : "Failed");
  WiFi.mode(WIFI_STA); // Important to be explicitly connected as client
  WiFi.begin(WIFISSID, WIFIPASSWD);

  uint8_t counter = 0;
  bool color = true;
  bool notconnected = false;
  while (WiFi.status() != WL_CONNECTED && !notconnected)
  {

    if (millis() % 100 == 0)
    {
      counter++;
      Serial.print(".");
    }

    if (counter == 10)
    {
      color = !color;
      Set_led(color * 0x0000FF, true);
      Serial.print(WiFi.status());
      Serial.println();
      counter = 0;
    }

    yield();
    if (millis() > 15000)
    {
      uint not_the_same_dumb_fucking = millis();
      Serial.print("Reseting at: ");
      Serial.print(not_the_same_dumb_fucking);
      Serial.print("ms / ");
      Serial.print(not_the_same_dumb_fucking / 1000);
      Serial.print("sec / ");
      Serial.print(not_the_same_dumb_fucking / 60000);
      Serial.println("min");
      notconnected = true;
      ESP.restart();
    }
  }
  Set_led(0x00FF00, true);
  ArduinoOTA.setHostname(DEVICE_NAME);
  ArduinoOTA.onStart(startOTA);
  ArduinoOTA.onEnd(endOTA);
  ArduinoOTA.onProgress(progressOTA);
  ArduinoOTA.onError(errorOTA);
  ArduinoOTA.begin();
  Serial.printf("Welcome to NightMare Systems. %s unit.\n", DEVICE_NAME);
  // touch_callback = touch_detected;
  lv_init();
  tft.begin();               /* TFT init */
  tft.setRotation(ROTATION); /* Landscape orientation, flipped */

  lv_disp_draw_buf_init(&draw_buf, buf, NULL, screenWidth * screenHeight / 10);

  /*Initialize the display*/
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  /*Change the following line to your display resolution*/
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  /*Initialize the (dummy) input device driver*/

  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);
  uint16_t calData[5] = {405, 3238, 287, 3292, 2};
  tft.setTouch(calData);
  ui_init();
  lv_obj_add_event_cb(ui_btAcPower, handle_ac, LV_EVENT_RELEASED, &AC_BUTTON_POWER_VAR);
  lv_obj_add_event_cb(ui_btAcSet, handle_ac, LV_EVENT_RELEASED, &AC_BUTTON_SET_VAR);
  lv_obj_add_event_cb(ui_btAcPlus, handle_ac, LV_EVENT_RELEASED, &AC_BUTTON_PLUS_VAR);
  lv_obj_add_event_cb(ui_btAcMinus, handle_ac, LV_EVENT_RELEASED, &AC_BUTTON_MINUS_VAR);
  lv_obj_add_event_cb(ui_btColorAuto, handle_color_auto2, LV_EVENT_RELEASED, &COLOR_BUTTON_AUTO_VAR);
  lv_obj_add_event_cb(ui_btReset, handle_config, LV_EVENT_RELEASED, &CONFIG_RESET_VAR);
  lv_obj_add_event_cb(ui_swLed, handle_config, LV_EVENT_VALUE_CHANGED, &CONFIG_LED_ENABLE_VAR);
  set_backlight(255);
  getTime();
  control_variables.boot = now() - millis() / 1000;
  hive_client.setInsecure();
  HiveMQ.setServer(MQTT_URL, MQTT_PORT);
  HiveMQ.setCallback(HandleMqtt);
  tcpClient.client->setTimeout(1);
  tcpClient.setMessageHandler(HandleTcp);
  tcpClient.setCllientInfo("Screen_bed", "2.4\" screen interface");
  tcp_reconnect();
  color = true;
  int color_c = 0xFF0000;
  if (tcpClient.connected() || HiveMQ.connected())
  {
    color_c = 0x00FF00;
  }

  for (size_t i = 0; i < 10; i++)
  {
    color = !color;
    Set_led(color_c * color, true);
    delay(100);
  }

  MqttSend("Sherlock/console/in", "getinfo", false);
  xSemaphore_tcp = xSemaphoreCreateBinaryStatic(&xSemaphore_tcp_Buffer);
  if (xSemaphore_tcp == NULL)
  {
    Serial.println("!!! Semaphore is null !!!");
    /* There was insufficient FreeRTOS heap available for the semaphore to
    be created successfully. */
  }
  else
  {
    xSemaphoreGive(xSemaphore_tcp);
    Serial.println("...Semaphore created...");
    /* The semaphore can now be used. Its handle is stored in the
    xSemahore variable.  Calling xSemaphoreTake() on the semaphore here
    will fail until the semaphore has first been given. */
  }
  xTaskCreatePinnedToCore(
      ConnectivityHandle, /* Function to implement the task */
      "TCP",              /* Name of the task */
      8192,               /* Stack size in words */
      NULL,               /* Task input parameter */
      0,                  /* Priority of the task */
      NULL,               /* Task handle. */
      1);                 /* Core where the task should run */
  Timers.create("Server Variables", 10, sync_server_variables, true);
  // Timers.create("Tcp/Mqtt", 20000, tcp_reconnect, true);
  Timers.create("setTime", 1000, set_time, true);
  Timers.create("Animation", 50, set_angle, true); // led cycle test
  Timers.create("boot_time", 60, set_config);
  Serial.println("Setup done");
  setTemp();
  Set_led(0, true);
}
void loop()
{
  lv_timer_handler();
  if (xSemaphoreTake(xSemaphore_tcp, 1) == pdTRUE)
  {
    HiveMQ.loop();
    tcpClient.handleClient();
    if (!xSemaphoreGive(xSemaphore_tcp))
      Serial.printf("...Error releasing semaphore [loop]...\n");
  }
  Timers.run();

  if (Serial.available() > 0)
  {
    String _input = "";
    while (Serial.available() > 0)
    {
      char c = Serial.read();
      // Control bytes from PIO implementation of serial monitor.
      if (c != 10 && c != 13)
      {
        _input += c;
      }
    }
    Serial.print("[Serial] ");
    MqttSend("Adler/console/in", _input, false);
  }

  if (millis() - oldTimers.last_touch > seconds_to_sleep * 1000)
  {
    sleep(true);
  }
}
