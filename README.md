# Fastled network reciever

This ie meant to be used with https://github.com/jeffborg/esp32-fastled-webserver-platformio whereas the animation is for this esp device is passed in via udp packets from the sender unit

The packets are buffered as there is a timestamp from the sender and they are played back at correct time scales

Also this receiver unit will send back 2 udp packets for every frame

1. when the udp packet is placed into the buffer (for the sender to work out the network delay)
2. When the udp packet is actually played (for the sender to work out the playback delay)

# todo

Need to play an animation of some sort on connecting wifi

