/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>

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
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
}

void loop()
{
  heading += 1;
  str_msg.data = heading;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(100);
}

