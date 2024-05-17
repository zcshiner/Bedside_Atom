/*
Everset ES100 Library V1.1 
*/

#ifndef ES100_h
#define ES100_h


const uint32_t CLOCK_FREQ     = 100000;		// Hz
const uint32_t  DEFAULT_CLOCK =	400000;		// Hz

const uint8_t ES100_ADDR =					0x32;		// ES100 i2c Address
//#define ES100_EN_DELAY				100000		// ES100 enable delay
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

struct ES100Status0
{
	// Data in the struct is only valid when rxOk = 1.
	uint8_t		rxOk = 0;	// 0 (0x0)  Indicates that a successful reception has not occured.
							// 1 (0x1)  Indicated that a successful reception has occured.
	uint8_t		antenna;	// 0 (0x0)  Indicates that the reception occured on Antenna 1.
							// 1 (0x1)  Indicates that the reception occured on Antenna 2.
	uint8_t		leapSecond; // 0 (0x00) Indicates that the current month WILL NOT have a leap second.
	                        // 2 (0x10) Indicates that the current month WILL have a negative leap second.
							// 3 (0x11) Indicates that the current month WILL have a positive leap second.
	uint8_t		dstState;	// 0 (0x00) Indicates that Daylight Savings Time (DST) is not in effect.
							// 1 (0x01) Indicates that DST ends today.
							// 2 (0x10) Indicates that DST begins totay.
							// 3 (0x11) Indicates that DST is in effect.
	uint8_t		tracking;	// 0 (0x0)  Indicates that the reception attenpt was a 1-minute frame operation.
							// 1 (0x1)  Indicates that the reception attemps was a tracking operation.
};

struct ES100Data
{
	ES100DateTime	dateTime;
	ES100NextDst 	nextDST;
	ES100Status0 	status;
	uint8_t			irqStatus;
	int 			timerValue;		// This hold the millis() when the interrupt occured, will be useful in the user code to handle the second boundary. New valid data should be handled within 65536 milli seconds, after that the variable will overflow and the user won't be able to calculate the right second boundary.
};

class ES100
{
	public:
		ES100Data     getData();
		ES100DateTime	getDateTime();
		ES100NextDst 	getNextDst();
		ES100Status0 	getStatus0();
		void			    begin(uint8_t int_pin, uint8_t en_pin);
		uint8_t		  	getDeviceID();
		uint8_t		  	getIRQStatus();
		void		    	enable();
		void		    	disable();
		void	    		startRx(uint8_t = false);
		void	    		stopRx();
		uint8_t 	  	getRxOk();
		uint8_t 	  	getAntenna();
		uint8_t 	  	getLeapSecond();
		uint8_t   		getDstState();
		uint8_t   		getTracking();
		
		int timezone   = 0;			    // time shift with a specific timezone. Can be placed any time that int can handle
		int DSTenabled = false;			// time shift depending on the DST

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
