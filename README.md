# SelfieMission <div float:right;><img src="MissionImages/20190920 First Selfie.png" width="192" hieght="108"/></div>

This project contains schematics, bills of material, and source code used during the [Selfie Mission](https://surfingsatellites.org/2020/08/12/the-selfie-mission/).

## Payload Cut Down
The Payload Cut Down provides an alternative method of terminating the flight by separating the balloon from the payload.  This time-based cut down is based on an Arduino Pro Mini.

## Selfie Module
The Selfie Module is the heart of the selfie sequence.  This module operated the arm (both extending and retrieving) through a motor controller, communicated with the selfie camera over wifi, use a Hall Effect sensor to stop arm movement, and communicated status updates with the Selfie Flight Computer.<br/>
<img src="MissionImages/SelfieModule.JPG" width="460" height="345">

## Selfie Flight Computer
The Selfie Flight Computer provided telemetry information through the radio, monitored battery levels, captured GPS information for telemetry, captured Selfie Module status information for telemetry, and monitored temperature.

## Selfie Base Station
The Selfie Base Station received telemetry from the payload.  It provides for communicating telemetry over RS-232 to a computer.  The program (written in C#) used for monitoring telemetry is included.
