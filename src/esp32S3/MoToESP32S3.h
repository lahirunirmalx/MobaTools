#ifndef MOTOESP32S3_H
#define MOTOESP32S3_H
// ESP32 specific defines for Cpp files
#define TODO 	// ignore TODO-marker

#if MESSAGES > 0
#pragma message "ESP32S3 (Nano-ESP32) specific cpp includes"
#endif

void ISR_Servo();
void ISR_Stepper();

inline __attribute__((__always_inline__)) void _noStepIRQ() {
    portENTER_CRITICAL(&stepperMux);
    #if defined COMPILING_MOTOSTEPPER_CPP
    //SET_TP3;
    #endif
}
inline __attribute__((__always_inline__)) void  _stepIRQ(bool force = true) { 
    // paramter force needed for compatibility with other architectures
        #if defined COMPILING_MOTOSTEPPER_CPP
            //CLR_TP3;
        #endif
    portEXIT_CRITICAL(&stepperMux);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if defined COMPILING_MOTOSERVO_CPP
// Values for Servo: -------------------------------------------------------
constexpr uint8_t INC_PER_MICROSECOND = 8;		// one speed increment is 0.125 µs
constexpr uint8_t  COMPAT_FACT = 1; // no compatibility mode for ESP                   
// defaults for macros that are not defined in architecture dependend includes
constexpr uint8_t INC_PER_TIC = INC_PER_MICROSECOND / TICS_PER_MICROSECOND;
#define time2tic(pulse)  ( (pulse) *  INC_PER_MICROSECOND )
#define tic2time(inc)  ( ( inc ) / INC_PER_MICROSECOND )
#define AS_Speed2Inc(speed) (speed)
//-----------------------------------------------------------------
/* Bei Stepper.cpp und servo.cpp wird das gleiche seizeTimerAS(); zur initiierung aufgerufen,
   da bisher für Servos und Stepper der gleiche Timer verwendet wurde ( nur unterchiedliche CMP-Register )
   Der ESP hat aber je Timer nur ein Alarmregister, deshalb muss für Stepper und Servos jeweils ein
   eigener Timer genutzt ( und initiiert ) werden. Deshalb wird hier im .h File für den Servo ein 
   eigener Timer-Init eingerichtet. Dies muss als 'inline' gemacht, damit er im Linker nicht mit dem
   Stepper- und Softled-Timer kollidiert. Da der Aufruf in beiden Fällen den gleichen Namen
   hat, könnte der Linker das nicht unterscheiden.
*/
static bool servoTimerInitialized = false;

static inline __attribute__((__always_inline__))void seizeTimerAS() {
    // Initiieren des Servo Timers ------------------------
    if ( !servoTimerInitialized ) {
		#if (ESP_ARDUINO_VERSION_MAJOR == 2)
			#pragma message "Info: using esp core 2.x.x"
        servoTimer = timerBegin(SERVO_TIMER, DIVIDER, true); // true= countup
        timerAttachInterrupt(servoTimer, &ISR_Servo, true);  // true= edge Interrupt
        timerAlarmWrite(servoTimer, ISR_IDLETIME*TICS_PER_MICROSECOND , false); // false = no autoreload );
        timerAlarmEnable(servoTimer);
		#elif (ESP_ARDUINO_VERSION_MAJOR == 3)
		#pragma message "Info: using esp core 3.x.x"
        // core 3.0.3 hw_timer_t * timerBegin(uint32_t frequency);   // frequency in Hz    
		servoTimer = timerBegin(2000000);  // frequency
        // core 3.0.3void timerAttachInterrupt(hw_timer_t * timer, void (*userFunc)(void));
        timerAttachInterrupt(servoTimer, &ISR_Servo); // assume edge - zs6buj
        timerAlarm(servoTimer, ISR_IDLETIME*TICS_PER_MICROSECOND , false, 0); // false = no autoreload );
		#else
		 #error "ESP-core version unsupported"
		#endif
			
        servoTimerInitialized = true;  
        MODE_TP1;   // set debug-pins to Output
        MODE_TP2;
        MODE_TP3;
        MODE_TP4;
    }
}

static inline __attribute__((__always_inline__)) void enableServoIsrAS() {
	// With ESP32S3 this is done in servoTimerInitialize above
}

static inline __attribute__((__always_inline__)) void setServoCmpAS(servoCmp_t aktAlarm) {
	// Set compare-Register for next servo IRQ
	#if (ESP_ARDUINO_VERSION_MAJOR == 2)
     timerAlarmWrite(servoTimer, aktAlarm , false); // no autorelaod
     timerAlarmEnable(servoTimer);
	#elif (ESP_ARDUINO_VERSION_MAJOR == 3)
    //3.0.3 void timerAlarm(hw_timer_t * timer, uint64_t alarm_value, bool autoreload, uint64_t reload_count);
    timerAlarm(servoTimer, aktAlarm , false, 0); // no autorelaod, 0=unlimited - zs6buj
	#else
		 #error "ESP-core version unsupported"
    #endif
}	

#endif
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef COMPILING_MOTOSOFTLED32_CPP
	extern uint64_t lastAlarm, aktAlarm;

	void seizeTimerAS();

	static inline __attribute__((__always_inline__)) void enableSoftLedIsrAS() {
	}



#endif  

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if defined COMPILING_MOTOSTEPPER_CPP
extern uint64_t lastAlarm, aktAlarm;

void seizeTimerAS();

static inline __attribute__((__always_inline__)) void enableStepperIsrAS() {
	// dummy
}

spi_t *spiHs = NULL;
static uint8_t spiInitialized = false;
static inline __attribute__((__always_inline__)) void initSpiAS(uint8_t ss=SS, uint8_t sck=SCK, uint8_t mosi=MOSI ) {
	if ( spiInitialized ) return;
	// initialize SPI hardware.
	// MSB first, default Clk Level is 0, shift on leading edge
	spiHs = spiStartBus(SPI_USED, SPI_CLOCK_DIV4, SPI_MODE0, SPI_MSBFIRST);
	if ( spiHs == NULL ) {
		Serial.println( "Init SPI failed");
	} else {
		//Serial.println( "Init SPI ok");
		spiAttachSCK(spiHs, sck);
		// MISO is not used, only serial output
		spiAttachMOSI(spiHs, mosi);
		spiAttachSS(spiHs, 0, ss);
		spiSSEnable(spiHs);
		spiInitialized = true;  // only if the bus could be started ( ISR checks this flag )
	}
}

static inline __attribute__((__always_inline__)) void startSpiWriteAS( uint8_t spiData[] ) {
   SET_TP2;
   // All this SPI-Transfers are blocking!!
   // This is called from the stepper ISR, so the variant without the core's SPI mutex must be
   // used: spiWriteWord() takes a FreeRTOS semaphore, which is not allowed in an ISR.
   // spiWriteShortNL() swaps the bytes for MSB-first exactly like spiWriteWord(), so the byte
   // order on the bus is unchanged. The SPI HW must be exclusive to MoToStepper.
   spiWriteShortNL(spiHs, (spiData[1]<<8) + spiData[0] );
   CLR_TP2;
}    


#endif
  
#endif