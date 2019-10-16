#include "ros/ros.h"
#include "ros_signalling/blink.h"
#include "std_msgs/String.h"
#include <queue>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
extern "C" {
#include "littleWire.h"
#include "littleWire_util.h"
}
#define NUM_GROUPS 4
#define NUM_LEDS_PER_GROUP 20
typedef struct ledStatus {
  ros_signalling::blink blink;
  int16_t timeoutCounter;
  bool on;
} ledStatus_t;

std::queue<ros_signalling::blink> received;
littleWire *lw = NULL;
const unsigned char pins [] = {PIN1, PIN2, PIN3, PIN4};
void sigintHandler(int sig)
{
  if(lw){
    ROS_INFO("Disconnecting LittleWire \n");
    littlewire_disconnect(lw);
  }
  ros::shutdown();
}

void blinkCallback(const ros_signalling::blink::ConstPtr &msg) {
  ros_signalling::blink blink = *msg;
  received.push(blink);
}

main(int argc, char **argv) {
  bool updated[NUM_GROUPS];
  unsigned char version;
  int maxleds;

  ros::init(argc, argv, "signalling");

  ros::NodeHandle n;
  std::vector<int> leds_per_group;
  n.param("signalling/ws2812/maxleds", leds_per_group);

  std::vector<std::vector<ledStatus_t>> cache(NUM_GROUPS);
  
for (int group = 0; group < NUM_GROUPS; group++) {
    int maxleds = NUM_LEDS_PER_GROUP;
    if(leds_per_group.size() > group){
      maxleds + leds_per_group[group];
    }
    updated[group] = true;
    cache[group].resize(maxleds);
    for (auto x = cache[group].begin(); x < cache[group].end(); x++) {
      x->blink.R = 0;
      x->blink.G = 0;
      x->blink.B = 0;
      x->on = false;
    }
  }

  uint16_t period = 10;
  ros::Rate rate(1000 / period);

  ros::Subscriber sub = n.subscribe("/signalling/leds", 10, blinkCallback);

  while (ros::ok()) {
    if (!lw) {
      int count = littlewire_search();
      if (count == 0) {
        ROS_INFO("No LittleWire found!\n");
        continue;
      }

      lw = littleWire_connect();
      if (lw == NULL) {
        ROS_INFO("LittleWire not found!\n");
        continue;
      }

      version = readFirmwareVersion(lw);
      ROS_INFO("Little Wire firmware version: %d.%d\n", ((version & 0xF0) >> 4),
               (version & 0x0F));
      if (version < 0x12) {
        ROS_ERROR("This node requires firmware version 1.2. Please update");
        return 0;
      }
    }
  
      for (int group = 0; group < NUM_GROUPS; group++) {
        for (auto x = cache[group].begin(); x < cache[group].end(); x++) {
          if (x->timeoutCounter) {
            x->timeoutCounter -= period;
            if (x->timeoutCounter <= 0) {
              if (x->on) {
                if (!x->blink.single) {
                  x->timeoutCounter = x->blink.msecOff;
                } else {
                  x->timeoutCounter = 0;
                }
              } else {
                x->timeoutCounter = x->blink.msecOn;
              }
              x->on = !x->on;
              updated[group] = true;
          }
        }
      }
    }

    if (!received.empty()) {
      ros_signalling::blink blink = received.front();
      received.pop();
      if (blink.group < NUM_GROUPS && blink.led < cache[blink.group].size()){
        cache[blink.group][blink.led].blink = blink;
        cache[blink.group][blink.led].on = true;
        cache[blink.group][blink.led].timeoutCounter = blink.msecOn;
        ROS_INFO("led %d(%d.%d.%d), On(%d)/Off(%d)", blink.led, blink.R,
                  blink.G, blink.B, blink.msecOn, blink.msecOff);
        updated[blink.group] = true;
      } else {
        ROS_WARN("Received command for a led out of allowed range (LED %d/GROUP %d)",
                 blink.led, blink.group);
      }
    }

    for (int group = 0; group < NUM_GROUPS; group++) {
      if(updated[group]){
        updated[group] = false;
        for (auto x = cache[group].begin(); x < cache[group].end(); x++) {
          if (x->on) {
            const ros_signalling::blink *b = &x->blink;
            ws2812_preload(lw, b->R, b->G, b->B);
          } else {
            ws2812_preload(lw, 0, 0, 0);
          }
        }
        ws2812_flush(lw, pins[group]);
        ROS_INFO("wr group(%d)", group);
      }
    }

    ros::spinOnce();
    rate.sleep();
  }
}