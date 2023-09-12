# MTE544 - Autonomous Mobile Robots - Setup

## Install pre-requisites of the course
If you have not installed Ubuntu 22.04 yet, please do that first.
Download the script (or if you installed ```git```, clone this repository with ```git clone https://github.com/aalghooneh/MTE544_student```)

Run the script:
```
sh setup_mte544.sh
```
that will take care of installing everything you need for this course, including setting up some environmental variables.


## To check the latency of the topics in TurtleBot4s
NOTE: This part may be needed when you will be using the physical robot to check the latency in the communications.

Use the Latency check script like this:

```
./latency_check.py topic msgType
# for example for scan topic
./latency_check.py /scan LaserScan 
```


