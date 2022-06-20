// SM172 pulse duty cycle thermometer
//  returns true if new averaged temperature 

#define NUMPERIODS  1000         //NUMOBS has to be large enough that the temperature calculation can complete before the next periodComplete is ready
volatile boolean periodComplete;
uint32_t isrPeriod;
uint32_t isrPulsewidth;
volatile uint32_t totalPeriod     = 0;
volatile uint32_t totalPulseWidth = 0;
volatile uint16_t count           = 0;

bool SM172_getTemperature(float *temperature) {

  if (periodComplete) {                            // Check if the period is complete
    *temperature = 212.77 * ((float)totalPulseWidth/ (float)totalPeriod) - 68.085;
    periodComplete = false;
    return(true);
    }
  return(false);
}

void TCC0_Handler()                              // Interrupt Service Routine (ISR) for timer TCC0
{     
  static uint16_t count = 0;
  // Check for match counter 0 (MC0) interrupt
  if (TCC0->INTFLAG.bit.MC0)             
  {   
    isrPeriod += TCC0->CC[0].reg;                 // Copy the period
    if(++count > NUMPERIODS) {
      periodComplete = true;                       // Indicate that the period is complete
      totalPeriod = isrPeriod;                   
      totalPulseWidth = isrPulsewidth;
      count = isrPeriod = isrPulsewidth = 0;
      count = 0;
    }
  }

  // Check for match counter 1 (MC1) interrupt
  if (TCC0->INTFLAG.bit.MC1)           
  {
    isrPulsewidth += TCC0->CC[1].reg;             // Copy the pulse-width
  }
}
void SM172_setup(){
  Serial.println("SM172_setup");
 PM->APBCMASK.reg |= PM_APBCMASK_EVSYS;     // Switch on the event system peripheral
 
  GCLK->GENDIV.reg = GCLK_GENDIV_DIV(1) |    // Divide the 48MHz system clock by 3 = 16MHz
                     GCLK_GENDIV_ID(4);      // Set division on Generic Clock Generator (GCLK) 5 

  GCLK->GENCTRL.reg = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                      GCLK_GENCTRL_GENEN |         // Enable GCLK 4
                      GCLK_GENCTRL_SRC_DFLL48M |   // Set the clock source to 48MHz
                      GCLK_GENCTRL_RUNSTDBY |      // Run during sleep
                      GCLK_GENCTRL_ID(4);          // Set clock source on GCLK 4
  while (GCLK->STATUS.bit.SYNCBUSY);               // Wait for synchronization

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |         // Route GCLK4 to TCC0 and TCC1
                      GCLK_CLKCTRL_GEN_GCLK4 |     
                      GCLK_CLKCTRL_ID_TCC0_TCC1;   

  // Enable the port multiplexer on digital pin D6
  PORT->Group[PORTA].PINCFG[20].bit.PMUXEN = 1;
  // Set-up the pin as an EIC (interrupt) peripheral on D6
  PORT->Group[PORTA].PMUX[20 >> 1].reg |= PORT_PMUX_PMUXE_A;
  
  EIC->EVCTRL.reg |= EIC_EVCTRL_EXTINTEO4;                                 // Enable event output on external interrupt 4
  EIC->CONFIG[0].reg |= EIC_CONFIG_SENSE4_HIGH;                            // Set event detecting a HIGH level
  EIC->INTENCLR.reg = EIC_INTENCLR_EXTINT4;                                // Clear the interrupt flag on channel 4
  EIC->CTRL.reg |= EIC_CTRL_ENABLE;                                        // Enable EIC peripheral
  while (EIC->STATUS.bit.SYNCBUSY);                                        // Wait for synchronization
  EIC->WAKEUP.reg = EIC_WAKEUP_WAKEUPEN4;                                  // cth wakeup on external interrupt 4
  
  EVSYS->USER.reg = EVSYS_USER_CHANNEL(1) |                                // Attach the event user (receiver) to channel 0 (n + 1)
                    EVSYS_USER_USER(EVSYS_ID_USER_TCC0_EV_1);              // Set the event user (receiver) as timer TCC0, event 1

  EVSYS->CHANNEL.reg = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT |                // No event edge detection
                       EVSYS_CHANNEL_PATH_ASYNCHRONOUS |                   // Set event path as asynchronous
                       EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_4) |    // Set event generator (sender) as external interrupt 4
                       EVSYS_CHANNEL_CHANNEL(0);                           // Attach the generator (sender) to channel 0

  TCC0->EVCTRL.reg |= TCC_EVCTRL_MCEI1 |           // Enable the match or capture channel 1 event input
                      TCC_EVCTRL_MCEI0 |           //.Enable the match or capture channel 0 event input
                      TCC_EVCTRL_TCEI1 |           // Enable the TCC event 1 input
                      /*TCC_EVCTRL_TCINV1 |*/      // Invert the event 1 input         
                      TCC_EVCTRL_EVACT1_PPW;       // Set up the timer for capture: CC0 period, CC1 pulsewidth
                                       
  NVIC_SetPriority(TCC0_IRQn, 0);      // Set the Nested Vector Interrupt Controller (NVIC) priority for TCC0 to 0 (highest)
  NVIC_EnableIRQ(TCC0_IRQn);           // Connect the TCC0 timer to the Nested Vector Interrupt Controller (NVIC)
 
  TCC0->INTENSET.reg = TCC_INTENSET_MC1 |            // Enable compare channel 1 (CC1) interrupts
                       TCC_INTENSET_MC0;             // Enable compare channel 0 (CC0) interrupts
 
  TCC0->CTRLA.reg |= TCC_CTRLA_CPTEN1 |              // Enable capture on CC1
                     TCC_CTRLA_CPTEN0 |              // Enable capture on CC0
                     TCC_CTRLA_PRESCSYNC_PRESC |     // Reload timer on the next prescaler clock
                     TCC_CTRLA_PRESCALER_DIV1;      // Set prescaler to 16, 16MHz/16 = 1MHz               
                     
  TCC0->CTRLA.bit.ENABLE = 1;                        // Enable TCC0
  while (TCC0->SYNCBUSY.bit.ENABLE);                 // Wait for synchronization
}
