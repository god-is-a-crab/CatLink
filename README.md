# CatLink
This is a GPS tracking solution for cats.

## Overview

There are 3 components of the CatLink system:

- Tracker (collar) - This device tracks the animal's position
- Gateway - This device manages communication with the tracker and the user's phone
- CatLink app - Displays animal position, radio link quality and other information

![](docs/diagram.jpg)

## How to use

<img src="docs/how-to.jpg" width="50%">

<img src="docs/gateway_board.jpg" width="50%">

1. Wake up the gateway using the RST button, a white LED will flash to indicate the gateway is pending connection
2. Press the bluetooth button (number 2 in image), wait for green check mark
3. Press the tracker connect button (number 3 in image), wait for green check mark
4. Wait for tracker position marker (number 5 in image) to update
5. Monitor heartbeat symbol (number 8 in image) to confirm tracker is communicating

## Images

### Collar
![collar](docs/collar.jpg)

### Gateway
![gateway](docs/gateway.jpg)
