/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);

ros::NodeHandle  nh;

std_msgs::Float32 str_msg;
ros::Publisher chatter("heading", &str_msg);

double heading = 0;

void messageCb( const std_msgs::Empty& toggle_msg){
  digitalWrite(LED_BUILTIN, HIGH-digitalRead(LED_BUILTIN));   // blink the led
}

ros::Subscriber<std_msgs::Empty> sub("blink", &messageCb );
void setup()
{
  bno.begin();
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
  delay(1000);

  bno.setExtCrystalUse(true);
}

void loop()
{
  sensors_event_t event;
  bno.getEvent(&event);
  heading = event.orientation.x;
  str_msg.data = heading;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(100);
}

