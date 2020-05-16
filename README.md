# Nova Fitness SDS dust sensors Mbed OS library
Supports Nova Fitness SDS011.
This library attempts to provide easy-to-use abstraction over Laser Dust Sensor SDS011 (https://cdn-reichelt.de/documents/datenblatt/X200/SDS011-DATASHEET.pdf).
Each response coming from sensor is validated whether it has correct head, command id, checksum and tail.

## Quickstart
```C++
#include "mbed.h"
#include <sds011.h>

#ifdef TARGET_STM32F767
RawSerial sds1(PC_12,PD_2);
#endif

#ifdef TARGET_STM32F446RE
RawSerial sds1(A0,A1); //PA_0, PA_1
#endif

SDS011 my_sds1;
float fPm25, fPm10;
bool received = false;
float p10, p25;

void fSDS011RxCb(int event) {
  if(event == SDS011::Event::RX_DATA) {
    fPm25 = my_sds1.getPM25();
    fPm10 = my_sds1.getPM10();
    received = true;
  }
}

int main() {
  Callback<void(int)> callback(fSDS011RxCb);
  my_sds1.init(&sds1,(fSDS011RxCb));
  printf("Start\n");
    
  while(1) {
    if(received) {
      printf("SDS P2.5=%f P10=%f\n", fPm25, fPm10);
      received=0;
    }
    thread_sleep_for(100);
  }
  return 0;
}
```
For more complex example see https://github.com/ATM-HSW/exampleSDS011.

## Initialization
Communication with the sensor is handled by an instance of RawSerial. It is passed directly to the constructor. When a complete data packet is received, a callback function is called by the library.
```C++
RawSerial sds1(PC_12,PD_2);

void fSDS011RxCb(int event) {
  fPm25 = my_sds1.getPM25();
  fPm10 = my_sds1.getPM10();
}

int main() {
  Callback<void(int)> callback(fSDS011RxCb);
  my_sds1.init(&sds1,(fSDS011RxCb));

  // ...
```

## Supported operations
Some operations listed in [Laser Dust Sensor Control Protocol V1.3](https://cdn.sparkfun.com/assets/parts/1/2/2/7/5/Laser_Dust_Sensor_Control_Protocol_V1.3.pdf) are supported. They are listed below:
* reports PM2.5, PM10 values (reads data from RawSerial, does not write anything to the serial),
* set working state to "sleeping"/"working",
* set working period,
* query firmware version (year, month, day).


### Reading PM2.5 and PM10 values
The functions getPM25() and getPM10() return the last received values sent from sensor, they don't send any request to sensor.
```C++
fPm25 = my_sds1.getPM25();
fPm10 = my_sds1.getPM10();
```

### Reading firmware version
The function querys the firmware version from the sensor.
```C++
fPm25 = my_sds1.getPM25();
fPm10 = my_sds1.getPM10();
```

### Setting custom working period
In order to set custom working period you need to specify single argument - duration (minutes) of the cycle. One cycle means working 30 sec, doing measurement and sleeping for ```duration-30 [sec]```. This setting is recommended when using 'active' reporting mode.
```C++
my_sds1.sendCommand(SDS011::Command::PERIOD,1);
```

### Setting sensor state to 'sleeping'
```C++
my_sds1.sendCommand(SDS011::Command::SLEEP);
```

### Waking up
```C++
my_sds1.sendCommand(SDS011::Command::WAKEUP);
```

## References
* Reichelt SDS011 [Laser Dust Sensor Control Protocol V1.3](https://cdn-reichelt.de/documents/datenblatt/X200/SDS011-DATASHEET.pdf)
* http://www.inovafitness.com/en/a/chanpinzhongxin/95.html
