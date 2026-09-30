// AVR HW-spcific Functions
#ifdef ARDUINO_ARCH_AVR
#define debugTP
//#define debugPrint
#include <MobaTools.h>

#if MESSAGES > 0  
#pragma message "HW specfic - avr ---"
#endif

uint8_t noStepISR_Cnt;   // Counter for nested StepISr-disable

//void ISR_Stepper(void);     // defined in MoToISR.cpp
nextCycle_t nextCycle;
static nextCycle_t cyclesLastIRQ = 1;  // cycles since last IRQ
// ---------- OCRxB Compare Interrupt used for stepper motor and Softleds ----------------
void stepperISR(nextCycle_t cyclesLastIRQ) __attribute__ ((weak));
void softledISR(nextCycle_t cyclesLastIRQ) __attribute__ ((weak));
ISR ( TIMERx_COMPB_vect) {
    uint16_t tmp;
  // Timer1 Compare B, used for stepper motor, starts every CYCLETIME us
    // 26-09-15 An Interrupt is only created at timeslices, where data is to output
    SET_TP1;
    nextCycle = ISR_IDLETIME  / CYCLETIME ;// min ist one cycle per IDLETIME
	CLR_TP1;
    if ( stepperISR ) stepperISR(cyclesLastIRQ);
    //============  End of steppermotor ======================================
   if ( softledISR ) softledISR(cyclesLastIRQ);
    // ======================= end of softleds =====================================
    // set compareregister to next interrupt time;
    // compute next IRQ-Time in us, not in tics, so we don't need long
	SET_TP1;
    noInterrupts(); // when manipulating 16bit Timerregisters IRQ must be disabled
    if ( nextCycle == 1 )  {
        CLR_TP1;
        // this is timecritical: Was the ISR running longer then CYCELTIME?
        // compute length of current IRQ ( which startet at OCRxB )
        // we assume a max. runtime of 1000 Tics ( = 500µs , what nevver should happen )
        tmp = GET_COUNT - OCRxB ;
		// From V3.0 on timer overflow is generally at 0xFFFF. No need to cope with when using uint16_t variables
        //if ( tmp > 1000 ) tmp += TIMER_OVL_TICS; // there was a timer overflow
        if ( tmp > (CYCLETICS-10) ) {
            // runtime was too long, next IRQ mus be started immediatly
            //SET_TP3;
            tmp = GET_COUNT+10; 
        } else {
            tmp = OCRxB + CYCLETICS;
        }
        //OCRxB = ( tmp > TIMER_OVL_TICS ) ? tmp -= TIMER_OVL_TICS : tmp ;
		OCRxB = tmp;
        SET_TP1;
    } else {
		SET_TP2;
        // time till next IRQ is more then one cycletime
        // NOT REQUIRED compute next IRQ-Time in us, not in tics, so we don't need long
        //tmp = ( OCRxB / TICS_PER_MICROSECOND + nextCycle * CYCLETIME );
        //if ( tmp > TIMERPERIODE ) tmp = tmp - TIMERPERIODE;
        //OCRxB = tmp * TICS_PER_MICROSECOND;
		OCRxB = OCRxB + nextCycle * CYCLETIME*TICS_PER_MICROSECOND;
		CLR_TP2;
    }
    interrupts();
    cyclesLastIRQ = nextCycle;
    CLR_TP1; // Oszimessung Dauer der ISR-Routine
}
////////////////////////////////////////////////////////////////////////////////////////////

void seizeTimerAS() {
    static bool timerInitialized = false;
    if ( !timerInitialized ) {
        uint8_t oldSREG = SREG;
        cli();
        
        TCCRxA =0; // Normal Mode TOP is 0xFFFF
		#if F_CPU == 32000000 // this is only possible with LGT8Fx
		TCCRxB = _BV(CS11)|_BV(CS10); // div 64 clock prescaler 
		#else
        TCCRxB = _BV(CS11); // div 8 clock prescaler 
		#endif
	  
        //ICRx = TIMERPERIODE * TICS_PER_MICROSECOND;  // timer periode is 20000us 
        OCRxA = FIRST_PULSE;
        OCRxB = 400;
        // Serial.print( " Timer initialized " ); Serial.println( TIMSKx, HEX );
        SREG = oldSREG;  // undo cli() 
        timerInitialized = true;  
        MODE_TP1;   // set debug-pins to Output
        MODE_TP2;
        MODE_TP3;
        MODE_TP4;
        DB_PRINT("Testpins initialisiert");
    }
}

extern uint8_t spiStepperData[2]; // step pattern to be output on SPI
extern uint8_t spiByteCount;

#ifdef SPCR
// use an ISR only if we have a 'real' SPI Hardware
static volatile byte received; // Dummy to clear SPIF ( static: must not clash with sketch globals )
ISR ( SPI_STC_vect ) { 
#ifdef  ARDUINO_AVR_LARDU_328E // ISR for LGT8Fx
    SET_TP4;
	// Check type of interrupt (  tansfer complete )
	if ( SPSR & (1<<SPIF) ) {
		// Interruptflag is not cleared - this can only be possible if sendbuffer
		// is empty ( transfer finished )
		if ( SPFR & (1<<WREMPT) ){
			SET_TP3;
			// Data is send completely
			SET_SS;
			spiByteCount = 0;
			SPCR &= ~(1<<SPIE);    // Interrupt disable
			CLR_TP3;
		} 
		received = SPDR;

	}
    CLR_TP4;

#else  // ISR for AVR ATMega
    SET_TP4;
    // output step-pattern on SPI, set SS when ready
    if ( spiByteCount++ == 0 ) {
        // end of shifting out high Byte, shift out low Byte
        SPDR = spiStepperData[0];
    } else {
        // end of data shifting
        //digitalWrite( SS, HIGH );
        SET_SS;
        spiByteCount = 0;
    }
    CLR_TP4;
#endif    
}
#endif

void enableSoftLedIsrAS() {
}

#endif