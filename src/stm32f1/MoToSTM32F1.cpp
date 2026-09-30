// STM32F1 HW-spcific Functions
#ifdef ARDUINO_ARCH_STM32F1
#define bool int
//#define debugTP
#define debugPrint
#include <MobaTools.h>
//#include <utilities/MoToDbg.h>

//#warning "HW specfic - STM32F1 ---"

uint8_t noStepISR_Cnt = 0;   // Counter for nested StepISr-disable

void stepperISR(nextCycle_t cyclesLastIRQ)  __attribute__ ((weak));
void softledISR(nextCycle_t cyclesLastIRQ)  __attribute__ ((weak));
nextCycle_t nextCycle;
static nextCycle_t cyclesLastIRQ = 1;  // cycles since last IRQ
void ISR_Stepper() {
    // Timer4 Channel 1, used for stepper motor and softleds, starts every nextCycle us
    // nextCycle ist set in stepperISR and softledISR
    //SET_TP1;
    nextCycle = ISR_IDLETIME  / CYCLETIME ;// min ist one cycle per IDLETIME
    if ( stepperISR ) stepperISR(cyclesLastIRQ);
    //============  End of steppermotor ======================================
    if ( softledISR ) softledISR(cyclesLastIRQ);
    // ======================= end of softleds =====================================
    // set compareregister to next interrupt time;
	uint16_t actCompare = timer_get_compare(MT_TIMER, STEP_CHN);
	uint16_t add2Ocr = nextCycle * TICS_PER_MICROSECOND; // tics to add to current compare reg
	uint16_t minDiff = (timer_get_count(MT_TIMER)+MIN_TIC_DIFF) - actCompare;
	if (  minDiff >= add2Ocr ) {
		// counter is already too far
        //CLR_TP2;
		add2Ocr = minDiff;
		nextCycle = add2Ocr / TICS_PER_MICROSECOND;
        //SET_TP2;
	}
	
    timer_set_compare( MT_TIMER, STEP_CHN, actCompare+add2Ocr ) ;
    cyclesLastIRQ = nextCycle;
    //CLR_TP1; // Oszimessung Dauer der ISR-Routine
}
////////////////////////////////////////////////////////////////////////////////////////////
void seizeTimerAS() {
    static bool timerInitialized = false;
    if ( !timerInitialized ) {
		DB_PRINT("Initialize MoToTimer");
        timer_init( MT_TIMER );
        timer_pause(MT_TIMER);
        // IRQ-Priorität von timer 4 interrupt auf lowest (15) setzen
        nvic_irq_set_priority ( NVIC_TIMER4, 15); // Timer 4 - stmduino sets all priorities to lowest level
                                                  // To be sure we set it here agai. These long lasting IRQ's 
                                                  // MUST be lowest priority
        timer_oc_set_mode( MT_TIMER, SERVO_CHN, TIMER_OC_MODE_FROZEN, 0 );  // comparison between output compare register and counter 
                                                                    //has no effect on the outputs
        timer_oc_set_mode( MT_TIMER, STEP_CHN, TIMER_OC_MODE_FROZEN, 0 );
        timer_set_prescaler(MT_TIMER, 36-1 );    // = 0.5µs Tics at 72MHz
        // V3: the timer runs freely and overflows at max count. All compare values are computed
        // in uint16_t ( wrap at 65536 ), so the reload must be 0xFFFF ( period 65536 tics ).
        timer_set_reload(MT_TIMER, 0xFFFF );
        timer_set_compare(MT_TIMER, STEP_CHN, 400 );
        //timer_attach_interrupt(MT_TIMER, TIMER_STEPCH_IRQ, (voidFuncPtr)ISR_Stepper );
        timer_set_compare(MT_TIMER, SERVO_CHN, FIRST_PULSE );
        timer_resume(MT_TIMER);
        timerInitialized = true;  
        MODE_TP1;
        MODE_TP2;
        MODE_TP3;
        MODE_TP4;
    }
}

/*
void enableServoIsrAS() {
}
*/
extern "C" {
// ------------------------  ISR for SPI-Stepper ------------------------
static int rxData;
#ifdef USE_SPI2
void __irq_spi2(void) {// STM32  spi2 irq vector
    rxData = spi_rx_reg(SPI2);            // Get dummy data (Clear RXNE-Flag)
    digitalWrite(BOARD_SPI2_NSS_PIN,HIGH);
}
#else
void __irq_spi1(void) {// STM32  spi1 irq vector
    //SET_TP4;
    rxData = spi_rx_reg(SPI1);            // Get dummy data (Clear RXNE-Flag)
    digitalWrite(BOARD_SPI1_NSS_PIN,HIGH);
    //CLR_TP4;
}
#endif
} // end of extern "C"



#endif
