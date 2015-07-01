Sensor
======

Arduino based humidity and temperature sensor with ZigBee data transfer as described
in Haenisch, T., Mai, D. Using a Sensor Network for Energy Optimization
of Paper Machine Dryer Sections, Athens Journal of Technology Engineering, September 2014,
http://www.atiner.gr/journals/technology/2014-1-3-3-Hanisch.pdf

Node
====
This directory contains the code for the sensor nodes. It is written in for the Arduino platform. There are two variants: The XBeeSens is for nodes with only one HYT 939 sensor, the XBSens_OT is for one HYT939 and two ADT7410 sensors per node.

It would be nice to put these two together in one version that automatically detects, which sensors are attached ..... 

(c) Till Hänisch
