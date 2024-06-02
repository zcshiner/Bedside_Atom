/*
Everset ES100 Library V1.1 
*/

#ifndef ES100_h
#define ES100_h

const uint32_t CLOCK_FREQ    = 100000;		// Hz
const uint32_t DEFAULT_CLOCK = 400000;		// Hz

const uint8_t  ES100_ADDR               =	0x32;		// ES100 i2c Address
const uint8_t  ES100_CONTROL0_REG       =	0x00;
const uint8_t  ES100_CONTROL1_REG       = 0x01;
const uint8_t  ES100_IRQ_STATUS_REG     = 0x02;
const uint8_t  ES100_STATUS0_REG        = 0x03;
const uint8_t  ES100_YEAR_REG           = 0x04;
const uint8_t  ES100_MONTH_REG          = 0x05;
const uint8_t  ES100_DAY_REG            = 0x06;
const uint8_t  ES100_HOUR_REG	          =	0x07;
const uint8_t  ES100_MINUTE_REG	        = 0x08;
const uint8_t  ES100_SECOND_REG	        = 0x09;
const uint8_t  ES100_NEXT_DST_MONTH_REG =	0x0A;
const uint8_t  ES100_NEXT_DST_DAY_REG 	= 0x0B;
const uint8_t  ES100_NEXT_DST_HOUR_REG  = 0x0C;
const uint8_t  ES100_DEVICE_ID_REG      = 0x0D;

struct ES100Control0
{
  bool  start;          // When the START bit is written with a 1, the status, date, and time registers are all cleared, bits
                        // 4:1 are sampled, and the ES100 begins receiving and processing the input signal.
  bool  ant1off;        // ES100 stops receiving and processing the input signal. This will automatically occur at the end
                        // of a successful reception. It can be forced to occur by the host processor writing a 0. (default)
  bool  ant2off;        // Antenna 1 input disabled.
                        // Antenna 1 input enabled. (default)
  bool  startAntenna;   // Antenna 2 input disabled.
                        // Antenna 2 input enabled. (default)
  bool  trackingEnable; // Start reception with antenna 2.
                        // Start reception with antenna 1. (default)
};

struct ES100IRQstatus
{
	bool  rxComplete;
  bool  cycleComplete;
};

struct ES100Status0
{
	// Data in the struct is only valid when rxOk = 1.
	bool    rxOk;      	// 0 (0b0)  Indicates that a successful reception has not occured.
						          // 1 (0b1)  Indicated that a successful reception has occured.
	bool	  antenna;    // 0 (0b0)  Indicates that the reception occured on Antenna 1.
						          // 1 (0b1)  Indicates that the reception occured on Antenna 2.
	uint8_t	leapSecond; // 0 (0b00) Indicates that the current month WILL NOT have a leap second.
	                    // 2 (0b10) Indicates that the current month WILL have a negative leap second.
						          // 3 (0b11) Indicates that the current month WILL have a positive leap second.
	uint8_t	dstState;   // 0 (0b00) Indicates that Daylight Savings Time (DST) is not in effect.
						          // 1 (0b01) Indicates that DST ends today.
						          // 2 (0b10) Indicates that DST begins totay.
						        	// 3 (0b11) Indicates that DST is in effect.
	bool	  tracking;   // 0 (0b0)  Indicates that the reception attenpt was a 1-minute frame operation.
							        // 1 (0b1)  Indicates that the reception attemps was a tracking operation.
};

struct ES100DateTime
{
	uint8_t		hour;
	uint8_t		minute;
	uint8_t		second;
	uint8_t		day;
	uint8_t		month;
	uint8_t		year;
};

struct ES100NextDst
{
	uint8_t		month;
	uint8_t		day;
	uint8_t		hour;
};

struct ES100Data
{
  ES100Status0 Status0;
  ES100DateTime DateTimeUTC;
};




class ES100
{
	public:
    void			      begin(uint8_t int_pin, uint8_t en_pin);
		void		    	  enable();
		void		    	  disable();    
    // False to start with Antennna 1, true for Antenna 2, singleAntenna disables the other antenna
		uint8_t    		  startRx(bool startAntenna = false, bool singleAntenna = false);
		uint8_t    		  startRxTracking(bool startAntenna = false);    
		uint8_t	    	  stopRx();    
    ES100Data       getData();
    ES100Control0   getControl0();
    ES100IRQstatus  getIRQStatus();
    ES100Status0 	  getStatus0();
		ES100DateTime   getUTCdateTime();
		ES100NextDst 	  getNextDst();
    uint8_t		  	  getDeviceID();

	private:
		uint8_t			_int_pin;
		uint8_t			_en_pin;

		uint8_t 	bcdToDec(uint8_t);
		void	  	_writeRegister(uint8_t addr, uint8_t data);
		uint8_t		_readRegister(uint8_t addr);
		void	  	_I2Cwrite(uint8_t addr, uint8_t numBytes, uint8_t *ptr);
		void	  	_I2Cread(uint8_t addr, uint8_t numBytes, uint8_t *ptr);
		void 		  shiftTime(int *year, int *month, int *day, int *hours, int *minutes, int *seconds);
};
#endif
