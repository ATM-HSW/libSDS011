// SDS011 dust sensor PM2.5 and PM10
// ---------------------------------
//
// By R. Zschiegner (rz@madavi.de)
// April 2016
//
// Documentation:
//      - The iNovaFitness SDS011 datasheet
//

#include "mbed.h"

class SDS011 {
  public:
    SDS011();
    void init(RawSerial * serial, const event_callback_t &callback);
	  void sendCommand(int cmd);
	  void sendCommand(int cmd, int param);
	  int isSleep();
	  int getPeriod();
		float getPM25();
		float getPM10();
	  int getID();
	  int getFW();
		int readable();
	
	  int _statechange;
		volatile int _state;
		volatile uint8_t _value;
  	volatile int _checksum_is;

  private:
    void sleep();
    void wakeup();
	  void firmware();
	  void period(int param);
    void statemachine(uint8_t data);
		static void rx0(int event); 
		static void rx1(int event); 
    RawSerial  *_sds_data;
		uint8_t _readbuf[1];
		volatile int _iPm25;
  	volatile int _iPm10;
  	volatile int _sds011_data;
  	volatile int _sleep_work_data;
	  volatile int _firmware_data;
	  volatile int _periode_data;
	  volatile int _id_set;	
  	volatile int _checksum_ok;
  	volatile int _error;
    volatile int _ID;
    volatile int _iId;
    volatile int _iFw;
    volatile int _FW;
    volatile int _period;
		volatile float _fPm25;
		volatile float _fPm10;
	  volatile int _sleep;
		volatile uint8_t _instancenumber;
		static uint8_t _instancecounter;
		static SDS011 *_instances[2];
		volatile int _received;
		event_callback_t _callback;
    
  private:
    enum States {
      STATE_AA = 0,
      STATE_CMD_ID,
      STATE_PM25_1B,
      STATE_PM25_2B,
      STATE_PM10_1B,
      STATE_PM10_2B,
      STATE_ID_1B,
      STATE_ID_2B,
      STATE_CHECKSUM,
      STATE_AB,
      STATE_END,
      STATE_5_6_7_8,
      STATE_QUERY_SET_MODE,
      STATE_SLEEP_WAKE_MODE,
      STATE_0,
      STATE_0_1,
      STATE_0_2,
      STATE_FW_1B,
      STATE_FW_2B,
      STATE_FW_3B,
      STATE_PERIOD_MODE,
      STATE_PERIOD
	  };
    
  public:
    enum Event {
      RX_DATA = 1,
      RX_SLEEP_WORK,
      RX_FIRMWARE,
      RX_PERIOD
    };

    enum Command {
      SLEEP = 1,
      WAKEUP,
      FIRMWARE,
      PERIOD
    };
};
