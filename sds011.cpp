/// SDS011 dust sensor PM2.5 and PM10
// ---------------------
//
// By R. Zschiegner (rz@madavi.de)
// April 2016
//
// Documentation:
//      - The iNovaFitness SDS011 datasheet
//

#include "sds011.h"

static char PERIODCMD[19] = {
    0xAA,   // head
    0xB4,   // command id
    0x08,   // data byte 1 Set working period
    0x01,   // data byte 2
    0x00,   // data byte 3
    0x00,   // data byte 4
    0x00,   // data byte 5
    0x00,   // data byte 6
    0x00,   // data byte 7
    0x00,   // data byte 8
    0x00,   // data byte 9
    0x00,   // data byte 10
    0x00,   // data byte 11
    0x00,   // data byte 12
    0x00,   // data byte 13
    0xFF,   // data byte 14 (device id byte 1)
    0xFF,   // data byte 15 (device id byte 2)
    0x05,   // checksum
    0xAB    // tail
};
static const char FIRMWARECMD[19] = {
    0xAA,   // head
    0xB4,   // command id
    0x07,   // data byte 1 Check firmware version
    0x00,   // data byte 2
    0x00,   // data byte 3
    0x00,   // data byte 4
    0x00,   // data byte 5
    0x00,   // data byte 6
    0x00,   // data byte 7
    0x00,   // data byte 8
    0x00,   // data byte 9
    0x00,   // data byte 10
    0x00,   // data byte 11
    0x00,   // data byte 12
    0x00,   // data byte 13
    0xFF,   // data byte 14 (device id byte 1)
    0xFF,   // data byte 15 (device id byte 2)
    0x05,   // checksum
    0xAB    // tail
};

static const char SLEEPCMD[19] = {
    0xAA,   // head
    0xB4,   // command id
    0x06,   // data byte 1 Set sleep and work
    0x01,   // data byte 2 (set mode)
    0x00,   // data byte 3 (sleep)
    0x00,   // data byte 4
    0x00,   // data byte 5
    0x00,   // data byte 6
    0x00,   // data byte 7
    0x00,   // data byte 8
    0x00,   // data byte 9
    0x00,   // data byte 10
    0x00,   // data byte 11
    0x00,   // data byte 12
    0x00,   // data byte 13
    0xFF,   // data byte 14 (device id byte 1)
    0xFF,   // data byte 15 (device id byte 2)
    0x05,   // checksum
    0xAB    // tail
};

static const char WAKEUPCMD[19] = {
    0xAA,   // head
    0xB4,   // command id
    0x06,   // data byte 1
    0x01,   // data byte 2 (set mode)
    0x01,   // data byte 3 (wakeup)
    0x00,   // data byte 4
    0x00,   // data byte 5
    0x00,   // data byte 6
    0x00,   // data byte 7
    0x00,   // data byte 8
    0x00,   // data byte 9
    0x00,   // data byte 10
    0x00,   // data byte 11
    0x00,   // data byte 12
    0x00,   // data byte 13
    0xFF,   // data byte 14 (device id byte 1)
    0xFF,   // data byte 15 (device id byte 2)
    0x06,   // checksum
    0xAB    // tail
};

static const char SETIDCMD[19] = {
    0xAA,   // head
    0xB4,   // command id
    0x05,   // data byte 1
    0x00,   // data byte 2 (set mode)
    0x00,   // data byte 3 (wakeup)
    0x00,   // data byte 4
    0x00,   // data byte 5
    0x00,   // data byte 6
    0x00,   // data byte 7
    0x00,   // data byte 8
    0x00,   // data byte 9
    0x00,   // data byte 10
    0x00,   // data byte 11
    0x00,   // data byte 12
    0x00,   // data byte 13
    0xFF,   // data byte 14 (device id byte 1)
    0xFF,   // data byte 15 (device id byte 2)
    0x00,   // checksum
    0xAB    // tail
};

static const char QUERYCMD[19] = {
    0xAA,   // head
    0xB4,   // command id
    0x04,   // data byte 1 (Query data command)
    0x00,   // data byte 2
    0x00,   // data byte 3 
    0x00,   // data byte 4
    0x00,   // data byte 5
    0x00,   // data byte 6
    0x00,   // data byte 7
    0x00,   // data byte 8
    0x00,   // data byte 9
    0x00,   // data byte 10
    0x00,   // data byte 11
    0x00,   // data byte 12
    0x00,   // data byte 13
    0xFF,   // data byte 14 (device id byte 1)
    0xFF,   // data byte 15 (device id byte 2)
    0x02,   // checksum
    0xAB    // tail
};


uint8_t SDS011::_instancecounter = 0;
SDS011 *SDS011::_instances[2] = {NULL,NULL};

SDS011::SDS011() {
	this->_instancenumber = SDS011::_instancecounter++;
	this->_instances[this->_instancenumber] = this;
	
	_iPm25 = 0;
  _iPm10 = 0;
  _checksum_is = 0;
  _sds011_data = 0;
  _sleep_work_data = 0;
	_firmware_data = 0;
  _id_set = 0;
  _checksum_ok = 0;
  _error = 1;
  _iId = 0;
}

// --------------------------------------------------------
// SDS011:read
// --------------------------------------------------------
void SDS011::rx0(int event) {
	uint8_t *ptr = SDS011::_instances[0]->_readbuf;
	SDS011::_instances[0]->statemachine(*ptr);
	*ptr=0;
	(SDS011::_instances[0]->_sds_data)->read(ptr, 1, &SDS011::rx0, SERIAL_EVENT_RX_COMPLETE);
}

void SDS011::rx1(int event) {
	uint8_t value = SDS011::_instances[1]->_readbuf[0];
	SDS011::_instances[0]->_readbuf[0]=0;
	SDS011::_instances[1]->statemachine(value);
	SDS011::_instances[1]->_sds_data->read(SDS011::_instances[1]->_readbuf, 1, Callback<void(int)>(&SDS011::rx1), SERIAL_EVENT_RX_COMPLETE);
}

void SDS011::statemachine(uint8_t value) {
	int event = 0;
	this->_value = value;
	this->_statechange = 1;
  if      (this->_state == SDS011::States::STATE_AA && value == 0xAA)             { this->_state = SDS011::States::STATE_CMD_ID;                                 this->_checksum_is = 0; }
  else if (this->_state == SDS011::States::STATE_CMD_ID && value == 0xC0)         { this->_state = SDS011::States::STATE_PM25_1B; }
  else if (this->_state == SDS011::States::STATE_CMD_ID && value == 0xC5)         { this->_state = SDS011::States::STATE_5_6_7_8; }
  else if (this->_state == SDS011::States::STATE_5_6_7_8 && value == 0x05)        { this->_state = SDS011::States::STATE_0_2;                                    this->_checksum_is += value; this->_id_set = 1;}
  else if (this->_state == SDS011::States::STATE_5_6_7_8 && value == 0x06)        { this->_state = SDS011::States::STATE_QUERY_SET_MODE;                         this->_checksum_is += value; this->_sleep_work_data = 1; }
  else if (this->_state == SDS011::States::STATE_5_6_7_8 && value == 0x07)        { this->_state = SDS011::States::STATE_FW_1B;                                  this->_checksum_is += value; this->_firmware_data = 1; }
  else if (this->_state == SDS011::States::STATE_5_6_7_8 && value == 0x08)        { this->_state = SDS011::States::STATE_PERIOD_MODE;                            this->_checksum_is += value; this->_periode_data = 1; }
  else if (this->_state == SDS011::States::STATE_QUERY_SET_MODE && value == 0x00) { this->_state = SDS011::States::STATE_SLEEP_WAKE_MODE;                        this->_checksum_is += value; }
  else if (this->_state == SDS011::States::STATE_QUERY_SET_MODE && value == 0x01) { this->_state = SDS011::States::STATE_SLEEP_WAKE_MODE;                        this->_checksum_is += value; }
  else if (this->_state == SDS011::States::STATE_SLEEP_WAKE_MODE)                 { this->_state = SDS011::States::STATE_0;        this->_sleep = value;         this->_checksum_is += value; }
  else if (this->_state == SDS011::States::STATE_PERIOD_MODE && value == 0x00)    { this->_state = SDS011::States::STATE_PERIOD;                                 this->_checksum_is += value; }
  else if (this->_state == SDS011::States::STATE_PERIOD_MODE && value == 0x01)    { this->_state = SDS011::States::STATE_PERIOD;                                 this->_checksum_is += value; }
  else if (this->_state == SDS011::States::STATE_PERIOD)                          { this->_state = SDS011::States::STATE_0;        this->_period = value;        this->_checksum_is += value; }
  else if (this->_state == SDS011::States::STATE_0_2 /*&& value == 0x00*/)        { this->_state = SDS011::States::STATE_0_1;                                    this->_checksum_is += value; }
  else if (this->_state == SDS011::States::STATE_0_1 /*&& value == 0x00*/)        { this->_state = SDS011::States::STATE_0;                                      this->_checksum_is += value; }
  else if (this->_state == SDS011::States::STATE_0 /*&& value == 0x00*/)          { this->_state = SDS011::States::STATE_ID_1B;                                  this->_checksum_is += value; }
  else if (this->_state == SDS011::States::STATE_PM25_1B)                         { this->_state = SDS011::States::STATE_PM25_2B;  this->_iPm25 = value;         this->_checksum_is += value; }
  else if (this->_state == SDS011::States::STATE_PM25_2B)                         { this->_state = SDS011::States::STATE_PM10_1B;  this->_iPm25 += (value << 8); this->_checksum_is += value; this->_sds011_data = 1; }
  else if (this->_state == SDS011::States::STATE_PM10_1B)                         { this->_state = SDS011::States::STATE_PM10_2B;  this->_iPm10 = value;         this->_checksum_is += value; }
  else if (this->_state == SDS011::States::STATE_PM10_2B)                         { this->_state = SDS011::States::STATE_ID_1B;    this->_iPm10 += (value << 8); this->_checksum_is += value; }
  else if (this->_state == SDS011::States::STATE_FW_1B)                           { this->_state = SDS011::States::STATE_FW_2B;    this->_iFw = value;           this->_checksum_is += value; }
  else if (this->_state == SDS011::States::STATE_FW_2B)                           { this->_state = SDS011::States::STATE_FW_3B;    this->_iFw += (value << 8);   this->_checksum_is += value; }
  else if (this->_state == SDS011::States::STATE_FW_3B)                           { this->_state = SDS011::States::STATE_ID_1B;    this->_iFw += (value << 16);  this->_checksum_is += value; }
  else if (this->_state == SDS011::States::STATE_ID_1B)                           { this->_state = SDS011::States::STATE_ID_2B;    this->_iId = value;           this->_checksum_is += value; }
  else if (this->_state == SDS011::States::STATE_ID_2B)                           { this->_state = SDS011::States::STATE_CHECKSUM; this->_ID += (value << 8);    this->_checksum_is += value; }
  else if (this->_state == SDS011::States::STATE_CHECKSUM)                        { this->_state = SDS011::States::STATE_AB;       this->_checksum_ok = (value == (_checksum_is % 256));      }
  else if (this->_state == SDS011::States::STATE_AB && value == 0xAB)             { this->_state = SDS011::States::STATE_END; }
	else                                                                            { this->_state = SDS011::States::STATE_END; }

  if (this->_state == SDS011::States::STATE_END) {
		if (this->_sds011_data == 1 && this->_checksum_ok == 1) {
      this->_fPm10 = (float)this->_iPm10/10.0;
      this->_fPm25 = (float)this->_iPm25/10.0;
      this->_sds011_data =0; this->_iPm10 = 0; this->_iPm25 = 0.0;
	  	this->_state = SDS011::States::STATE_AA;
	    this->_received = 1;
			this->_sds011_data = 0;
			this->_ID = this->_iId;
			event = SDS011::Event::RX_DATA;
		} else if(this->_periode_data && this->_checksum_ok == 1) {
			this->_periode_data = 0;
			event = SDS011::Event::RX_PERIOD;
		} else if(this->_sleep_work_data && this->_checksum_ok == 1) {
			this->_sleep_work_data = 0;
			event = SDS011::Event::RX_SLEEP_WORK;
		} else if(this->_firmware_data && this->_checksum_ok == 1) {
			this->_firmware_data = 0;
			this->_FW = this->_iFw;
			event = SDS011::Event::RX_FIRMWARE;
		}
    if (_callback && event > 0) {
      _callback.call(event);
    }
		this->_state = SDS011::States::STATE_AA;
		this->_checksum_is = 0;
		this->_checksum_ok = 0;
    this->_error = 0;
  }
}

float SDS011::getPM10() {
	return this->_fPm10;
}

float SDS011::getPM25() {
	return this->_fPm25;
}

int SDS011::getID() {
	return this->_ID;
}

int SDS011::getFW() {
	return this->_FW;
}

int SDS011::readable() {
	if(this->_received) {
		this->_received = 0;
		return 1;
	}
	return 0;
}

void SDS011::sendCommand(int cmd, int param) {
	switch(cmd) {
		case SLEEP: sleep(); break;
		case WAKEUP: wakeup(); break;
		case FIRMWARE: firmware(); break;
		case PERIOD: period(param); break;			
	}
}
void SDS011::sendCommand(int cmd) {
  sendCommand(cmd, -1);
}

// --------------------------------------------------------
// SDS011:sleep
// --------------------------------------------------------
void SDS011::sleep() {
  for (uint8_t i = 0; i < 19; i++) {
    _sds_data->putc(SLEEPCMD[i]);
  }
}

// --------------------------------------------------------
// SDS011:wakeup
// --------------------------------------------------------
void SDS011::wakeup() {
  for (uint8_t i = 0; i < 19; i++) {
    _sds_data->putc(WAKEUPCMD[i]);
  }
}

// --------------------------------------------------------
// SDS011:firmware
// --------------------------------------------------------
void SDS011::firmware() {
  for (uint8_t i = 0; i < 19; i++) {
    _sds_data->putc(FIRMWARECMD[i]);
  }
}

void SDS011::period(int param) {
	int chksum=0;
	if(param < 0 || param > 30)
		return;
	PERIODCMD[4]=(uint8_t)param;
  for (uint8_t i = 2; i < 17; i++) {
    chksum+=PERIODCMD[i];
  }
	PERIODCMD[17]=(uint8_t)chksum&0xff;
  for (uint8_t i = 0; i < 19; i++) {
    _sds_data->putc(PERIODCMD[i]);
  }
	
}


int SDS011::isSleep() {
	return this->_sleep;
}

int SDS011::getPeriod() {
	return this->_period;
}


void SDS011::init(RawSerial * serial, const event_callback_t &callback) {
  _sds_data = serial;
	_sds_data->baud(9600);
	_callback = callback;
	switch(this->_instancecounter) {
		case 1:
	    _sds_data->read(this->_readbuf, 1, Callback<void(int)>(&SDS011::rx0), SERIAL_EVENT_RX_COMPLETE);
		  break;
		case 2:
	    _sds_data->read(this->_readbuf, 1, Callback<void(int)>(&SDS011::rx1), SERIAL_EVENT_RX_COMPLETE);
		  break;
	}
	this->_state = STATE_AA;
	this->wakeup();
}
