Hot dang, I did it. I struggled. I can't help you get it working on your own devices, I'm sure it's possible. If it helps, I started with the ot_cli demo. Then changed the code a bunch. I also have the sdkconfig option to automatically set up the thread network enabled.

This code should be flashed on two ESP32-C6 devices. They find each other, and then when you press the BOOT button on one of those devices, the LED on the other device will blink. It goes both ways. Use it for slow morse code, making another part of the room more red without being there. Who knows.

To help search engines, this is for ESP32-C6 and uses openthread to create a Thread network local to two or more ESP32-C6 devices. It uses UDP to broadcast a heartbeat to other nodes. The other nodes receive the UDP message and in this case blink an LED.
