/*!
 * @file ES100.h
 *
 * Everset ES100 WWVB (BPSK) receiver Library V2.1
*/

#ifndef ES100_h
#define ES100_h

/** @brief DST State decode names */
enum dstState_codes{
  DSTinactive = 0b00,
  DSTend = 0b01,
  DSTbegin = 0b10,
  DSTactive = 0b11
};

/** @brief Leap Second decode names */
enum leapSecond_codes{
  noLeapSecond = 0b00,
  positiveLeapSecond = 0b10,
  negativeLeapSecond = 0b11
};

/** @brief Control0 register data */
struct ES100Control0
{
  bool  start;          /**< When the START bit is written with a 1, the status, date, and time registers are all cleared,
                        // bits 4:1 are sampled, and the ES100 begins receiving and processing the input signal.
                        // Writing 0 to ES100 stops receiving and processing the input signal. This will automatically occur at the end
                        // of a successful reception. It can be forced to occur by the host processor writing a 0. (default) */
  bool  ant1off;        /**< 1: Antenna 1 input disabled or 0: Antenna 1 input enabled. (default) */
  bool  ant2off;        /**< 1: Antenna 2 input disabled or 0: Antenna 2 input enabled. (default) */
  bool  startAntenna;   /**< 1:Start reception with antenna 2 or 0: Start reception with antenna 1. (default) */
  bool  trackingEnable; /**< 1: Tracking mode enabled or 0: Tracking mode disabled. (default) */
};

/** @brief IRQ Status register data */
struct ES100IRQstatus
{
	bool  rxComplete;     /**< 1: Reception Complete. Indicates that IRQ- went active due to a successful reception. */
  bool  cycleComplete;  /**< 1: Cycle Complete. Indicates that IRQ- went active due to the unsuccessful completion of a reception attempt. */
};

/** @brief Status0 register data */
struct ES100Status0
{
	// Data in the struct is only valid when rxOk = 1.
	bool    rxOk;       /**<  0 (0b0)  Indicates that a successful reception has not occured.
                            1 (0b1)  Indicated that a successful reception has occured. */
	bool	  antenna;    /**<  0 (0b0)  Indicates that the reception occured on Antenna 1.
                            1 (0b1)  Indicates that the reception occured on Antenna 2. */
	leapSecond_codes	leapSecond; /**< noLeapSecond: 0 (0b00) Indicates that the current month WILL NOT have a leap second.
                                noLeapSecond:       1 (0b01) Also indicates that the current month WILL NOT have a leap second.
                                positiveLeapSecond: 2 (0b10) Indicates that the current month WILL have a negative leap second.
                                negativeLeapSecond: 3 (0b11) Indicates that the current month WILL have a positive leap second. */
	dstState_codes	dstState; /**<  DSTinactive: 0 (0b00) Indicates that Daylight Savings Time (DST) is not in effect.
                                  DSTend:      1 (0b01) Indicates that DST ends today.
                                  DSTbegin:    2 (0b10) Indicates that DST begins totay.
                                  DSTactive:   3 (0b11) Indicates that DST is in effect. */
	bool	  tracking;   /**<  0 (0b0)  Indicates that the reception attenpt was a 1-minute frame operation.
                            1 (0b1)  Indicates that the reception attemps was a tracking operation. */
};

/** @brief Combined date/time register data */
struct ES100DateTime
{
	uint8_t		hour;
	uint8_t		minute;
	uint8_t		second;
	uint8_t		day;
	uint8_t		month;
	uint8_t		year;
};

/** @brief Next DST register data */
struct ES100NextDst
{
	uint8_t		month;
	uint8_t		day;
	uint8_t		hour;
};

/** @brief Superset of Status0 and DateTime */
struct ES100Data
{
  ES100Status0 Status0;       /**< struct ES100Status0 */
  ES100DateTime DateTimeUTC;  /**< struct ES100DateTime */
};

/** @brief ES100 Antenna Selection */
enum es100Antenna : bool {
  ANT_1,
  ANT_2
};


/*!
	@brief Class to interface with ES100 Receiver
*/
class ES100
{
	public:
		/*!
			@brief  Setup hardware to interface with ES100 Receiver
			@param  int_pin  Pin number of interrupt request (IRQ) pin
			@param  en_pin   Pin number of enable pin
		*/
		void begin(uint8_t int_pin, uint8_t en_pin);

		/*!
			@brief  Handshake to enable ES100 Receiver
		*/
		void enable();

		/*!
			@brief  Brings enable pin low to disable ES100 Receiver
		*/
		void disable();

		/*!
			@brief  Start a 1-Minute Frame Reception
			@param  startAntenna   ANT_1 or ANT_2
			@param  singleAntenna  True disables the other antenna
			@return EXIT_SUCCESS or EXIT_FAILURE
		*/
		static uint8_t    		  startRx(es100Antenna startAntenna = ANT_1, bool singleAntenna = false);

		/*!
			@brief  Start a Tracking Reception
			@param  startAntenna   ANT_1 or ANT_2
			@param  singleAntenna  True disables the other antenna
			@return EXIT_SUCCESS or EXIT_FAILURE
		*/
		static uint8_t    		  startRxTracking(es100Antenna startAntenna = ANT_1);
		
		/*!
			@brief  Write stop bit and end reception
			@return EXIT_SUCCESS or EXIT_FAILURE
		*/   
		static uint8_t	    	  stopRx();

		/*!
			@brief  read Status0 and combined UTC date registers from receiver
			@return ES100Data struct
		*/ 
		static ES100Data       getData();

		/*!
			@brief  read Control0 register from receiver
			@return ES100Control0 struct
		*/ 
		static ES100Control0   getControl0();

		/*!
			@brief  read IRQ Status register from receiver
			@return ES100IRQstatus struct
		*/ 
		static ES100IRQstatus  getIRQStatus();

		/*!
			@brief  read Status0 register from receiver
			@return ES100Status0 struct
		*/ 
		static ES100Status0 	  getStatus0();

		/*!
			@brief  read year, month, day, hour, minute, second registers from receiver
					convert received data from BCD to decimal
			@return ES100DateTime struct
		*/ 
		static ES100DateTime   getUTCdateTime();

		/*!
			@brief  read Next DST egisters from receiver and convert from BCD to decimal
			@return ES100NextDst struct
		*/ 
		static ES100NextDst 	  getNextDst();

		/*!
			@brief  read Device ID register from receiver
			@return hexadecimal device ID
		*/ 
		static uint8_t		  	  getDeviceID();

	private:
    static const uint32_t CLOCK_FREQ    = 100000;		// Hz
    static const uint32_t DEFAULT_CLOCK = 400000;		// Hz

    static const uint8_t ES100_ADDR               = 0x32;		// ES100 i2c Address
    static const uint8_t ES100_CONTROL0_REG       = 0x00;
    static const uint8_t ES100_CONTROL1_REG       = 0x01;
    static const uint8_t ES100_IRQ_STATUS_REG     = 0x02;
    static const uint8_t ES100_STATUS0_REG        = 0x03;
    static const uint8_t ES100_YEAR_REG           = 0x04;
    static const uint8_t ES100_MONTH_REG          = 0x05;
    static const uint8_t ES100_DAY_REG            = 0x06;
    static const uint8_t ES100_HOUR_REG	          = 0x07;
    static const uint8_t ES100_MINUTE_REG	        = 0x08;
    static const uint8_t ES100_SECOND_REG	        = 0x09;
    static const uint8_t ES100_NEXT_DST_MONTH_REG = 0x0A;
    static const uint8_t ES100_NEXT_DST_DAY_REG 	= 0x0B;
    static const uint8_t ES100_NEXT_DST_HOUR_REG  = 0x0C;
    static const uint8_t ES100_DEVICE_ID_REG      = 0x0D;
		
    uint8_t			_int_pin;
		uint8_t			_en_pin;

		/*!
			@brief  convert Binary Coded Decimal data to decimal
			@param  value
			@return decimal data
		*/ 
		static uint8_t 	bcdToDec(uint8_t value);

		/*!
			@brief  Write data to receiver register
			@param  addr Register address
			@param  data Data to write
		*/ 
		static void	  	_writeRegister(uint8_t addr, uint8_t data);

		/*!
			@brief  Read data to receiver register
			@param  addr Register address
			@return Data from register
		*/ 
		static uint8_t		_readRegister(uint8_t addr);

		/*!
			@brief  Write data over I2C
			@param  addr Register Address
			@param  numBytes Number of bytes to write
			@param  *ptr pointer to data to write
		*/ 
		static void	  	_I2Cwrite(uint8_t addr, uint8_t numBytes, uint8_t *ptr);

		/*!
			@brief  Read data over I2C
			@param  addr Register Address
			@param  numBytes Number of bytes to read
			@param  *ptr pointer to read data into
		*/ 
		static void	  	_I2Cread(uint8_t addr, uint8_t numBytes, uint8_t *ptr);

};
#endif
