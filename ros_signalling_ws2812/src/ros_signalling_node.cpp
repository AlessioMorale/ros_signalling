#include "ros/ros.h"
#include "ros_signalling/blink.h"
#include "std_msgs/String.h"
#include <queue>
#include <stdio.h>
#include <stdlib.h>
extern "C" {
#include "littleWire.h"
#include "littleWire_util.h"
}

typedef struct ledStatus {
  ros_signalling::blink blink;
  int16_t timeoutCounter;
  bool on;
} ledStatus_t;

std::queue<ros_signalling::blink> received;

void blinkCallback(const ros_signalling::blink::ConstPtr &msg) {
  ros_signalling::blink blink = *msg;
  received.push(blink);
}

main(int argc, char **argv) {
  littleWire *lw = NULL;
  bool updated = true;
  unsigned char version;
  int maxleds;
  std::vector<ledStatus_t> cache(20);
  for (auto i = cache.begin(); i != cache.end(); i++) {
    i->blink.R = 0;
    i->blink.G = 0;
    i->blink.B = 0;
    i->on = false;
  }

  ros::init(argc, argv, "signalling");

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/signalling/leds", 10, blinkCallback);

  n.param("signalling/ws2812/maxleds", maxleds, 20);

  uint16_t period = 10;
  ros::Rate rate(1000 / period);

  while (ros::ok()) {
    if (!lw) {
      lw = littleWire_connect();

      if (lw == NULL) {
        ROS_INFO("LittleWire found!\n");
      }
      continue;
      version = readFirmwareVersion(lw);
      ROS_INFO("Little Wire firmware version: %d.%d\n", ((version & 0xF0) >> 4),
               (version & 0x0F));
      if (version < 0x12) {
        ROS_ERROR("This node requires firmware version 1.2. Please update");
        return 0;
      }
    }

    for (auto i = cache.begin(); i < cache.end(); i++) {
      if (i->timeoutCounter) {
        i->timeoutCounter -= period;
        if (i->timeoutCounter <= 0) {
          if (i->on) {
            if (!i->blink.single) {
              i->timeoutCounter = i->blink.msecOff;
            } else {
              i->timeoutCounter = 0;
            }
          } else {
            i->timeoutCounter = i->blink.msecOn;
          }
          i->on = !i->on;
          updated = true;
        }
      }
    }
    if (!received.empty()) {
      ros_signalling::blink blink = received.front();
      received.pop();
      if (blink.led < maxleds) {
        cache[blink.led].blink = blink;
        cache[blink.led].on = true;
        cache[blink.led].timeoutCounter = blink.msecOn;
        ROS_DEBUG("led %d(%d.%d.%d), On(%d)/Off(%d)", blink.led, blink.R,
                  blink.G, blink.B, blink.msecOn, blink.msecOff);
        updated = true;
      } else {
        ROS_WARN("Received led command for a led higher (%d) than the maximum "
                 "configured(%d)",
                 blink.led, maxleds);
      }
    }

    if (updated) {
      updated = false;
      for (auto i = cache.cbegin(); i < cache.cend(); i++) {
        if (i->on) {
          const ros_signalling::blink *b = &i->blink;
          ws2812_preload(lw, b->R, b->G, b->B);
        } else {
          ws2812_preload(lw, 0, 0, 0);
        }
      }

      ws2812_flush(lw, 1);
    }

    ros::spinOnce();
    rate.sleep();
  }
}