/**
    File: mainClock.h
    Authors: Oktavian Gniot
    Version: 0.4.2 beta 2017-05-05

    Description: Header file for the motor module.
                 This file contains the definitions of the battery module. This
                 module is only a placeholder for future development to keep in
                 mind what could be implemented later on.
*/

#ifndef VIBRABOT_FIRMWARE_MAIN_CLOCK_H_
#define VIBRABOT_FIRMWARE_MAIN_CLOCK_H_

#include <stdint.h>
#include <stdbool.h>



typedef struct{
	uint8_t systemTick:1;
	uint8_t systemTickOverflow:1;
}MainClockStatus_t;



class MainClock {
private:
	MainClockStatus_t status;
	
public:

	void startTimer(void);

	void setTick(void);
	bool isTick(void);
  int getTicks(void);
  void resetTicks(void);
	bool hasOverflow(void);
	void clearOverflow(void);
};

#endif
