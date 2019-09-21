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

std::queue<ros_signalling::blink> received;

void blinkCallback(const ros_signalling::blink::ConstPtr &msg) {
  ros_signalling::blink blink = *msg;
  received.push(blink);
}

main(int argc, char **argv) {
  littleWire *lw = NULL;
  bool updated = true;
  unsigned char version;

  std::vector<ros_signalling::blink> cache(20);
  for (auto i = cache.begin(); i != cache.end(); i++) {
    i->R = 0;
    i->G = 0;
    i->B = 0;
  }

  ros::init(argc, argv, "signalling");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/signalling/leds", 1000, blinkCallback);

  ros::Rate rate(100);

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

    if (!received.empty()) {
      ros_signalling::blink blink = received.front();
      received.pop();
      cache[blink.led] = blink;
      ROS_DEBUG("led %d(%d.%d.%d)\n", blink.led, blink.R, blink.G, blink.B);
      updated = true;
    }

    if (updated) {
      updated = false;
      for (auto i = cache.cbegin(); i < cache.cend(); i++) {
        ws2812_preload(lw, i->R, i->G, i->B);
      }

      ws2812_flush(lw, 1);
    }
    rate.sleep();
    ros::spinOnce();
  }
}