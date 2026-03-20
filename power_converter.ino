#define CPU_FREQ  16000000UL
#define PWM_FREQ  2500UL
#define ADC_RANGE 1023
#define SAMPLE_RATE_PER_CHANNEL 2500UL
#define NUM_CHANNELS  4
#define ADC_CONVERSION_RATE (NUM_CHANNELS * SAMPLE_RATE_PER_CHANNEL)
#define T_s  1.0f/SAMPLE_RATE_PER_CHANNEL
#define MPPT_T_s   0.1f
#define MPPT_TICKS   (uint16_t)(MPPT_T_s * SAMPLE_RATE_PER_CHANNEL)


// Analog pins for ADC readings
const uint8_t Vpv_pin = A0;
const uint8_t Ipv_pin = A1;
const uint8_t Vbuck_1_pin = A2;
const uint8_t Vbuck_2_pin = A3;

// PWM pins work on timer3 which controls pins 5 (OC3A), 2 (OC3B), 3 (OC3C)
const uint8_t buck_1_pwm_pin = 5;
const uint8_t buck_2_pwm_pin = 2;

// Initialising variables
volatile uint16_t Vpv_adc_reading;
volatile uint16_t Ipv_adc_reading;
volatile uint16_t Vbuck_1_adc_reading;
volatile uint16_t Vbuck_2_adc_reading;

volatile float Vpv;
volatile float Ipv;
volatile float Vbuck_1;
volatile float Vbuck_2;

const float adc_voltage = 5.0f;
const float Vpv_scale = 5.55f;
const float Ipv_scale = 1.0f;
const float Vbuck_1_scale = 5.55f;
const float Vbuck_2_scale = 5.55f;

const float Vbuck_1_ref = 10.0f;
const float Vbuck_2_ref = 5.0f;
volatile float buck_1_duty = 0.5f;
volatile float buck_2_duty = 0.7f;
volatile float mppt_duty = 0.5f;

volatile float Vpv_prev = 0.0f;
volatile float Ipv_prev = 0.0f;

struct pi_controller {
  float kp;
  float ki;
  float integral;
  float out_min;
  float out_max;
};

pi_controller pi_buck_1;
pi_controller pi_buck_2;
pi_controller pi_mppt;

volatile uint8_t mppt_mode = 0;
volatile uint8_t fixed_voltage_mode = 1;
volatile uint8_t current_mode = fixed_voltage_mode; 
volatile bool overvoltage_detected = false;

// Timer1 flag to know when to sample
volatile uint8_t current_adc_channel = 0;
volatile bool sampling_done = false; 
volatile bool MPPT_ready = false;
volatile uint16_t MPPT_counter = 0;

// Function declarations
void setup_PWM_on_timer3();
void setup_sampling_on_timer1();
void setup_ADC_autotrigger();
void set_buck_1_duty(float duty);
void set_buck_2_duty(float duty);
float update_pi(pi_controller &pi, float reference, float measurement, float Ts, float sign);
float update_mppt(float Vpv, float Ipv);
float voltage_to_current(float Vreading) {
  return 0.0285714f * Vreading;  // Calibrated and found using the physical current sensor circuit
}
void setup() {
  Serial.begin(115200);

  // ISR and timing stuff setup
  setup_PWM_on_timer3();
  setup_sampling_on_timer1();
  setup_ADC_autotrigger();

  // Initial duty cycles
  set_buck_1_duty(buck_1_duty);
  delay(250);
  set_buck_2_duty(buck_2_duty);

  // Initialising PI controllers
  pi_buck_2.kp = 0.375f;
  pi_buck_2.ki = 0.665f;
  pi_buck_2.out_min = 0.0f;
  pi_buck_2.out_max = 1.0f;
  pi_buck_2.integral = 0.0f;

  pi_buck_1.kp = 0.675f;
  pi_buck_1.ki = 1.765f;
  pi_buck_1.out_min = 0.0f;
  pi_buck_1.out_max = 1.0f;
  pi_buck_1.integral = 0.0f;

  pi_mppt.kp = 0.175f;
  pi_mppt.ki = 0.365f;
  pi_mppt.out_min = 0.0f;
  pi_mppt.out_max = 1.0f;
  pi_mppt.integral = 0.0f;

  // Select starting mode
  current_mode = fixed_voltage_mode; 

}

void loop() {
  /*
  */
  static uint16_t counter = 0;
  if (++counter >= 60000) { // slowly prints
    counter = 0;
    Serial.print("buck_1 V: ");
    Serial.print(Vbuck_1);
    Serial.print(" duty: ");
    Serial.print(buck_1_duty);
    Serial.print(" | buck_2 V: ");
    Serial.print(Vbuck_2);
    Serial.print(" duty: ");
    Serial.print(buck_2_duty);
    Serial.print(" | Vpv: ");
    Serial.print(Vpv);
    Serial.print(" | Ipv: ");
    Serial.println(Ipv*1000.0f);
  }

  if (overvoltage_detected) {
    overvoltage_detected = false;
    Serial.println("WARNING: Voltage exceeding 24V detected");
    set_buck_1_duty(0.0f);
    set_buck_2_duty(0.0f);
    delay(2000);
    return;
  }
  
}



void setup_PWM_on_timer3() {
  pinMode(buck_1_pwm_pin, OUTPUT);
  pinMode(buck_2_pwm_pin, OUTPUT);

  // Stop timer
  TCCR3A = 0;
  TCCR3B = 0;

  // Fast PWM, TOP = ICR3 (Mode 14: WGM33:0 = 1110)
  TCCR3A |= (1 << WGM31);
  TCCR3B |= (1 << WGM33) | (1 << WGM32);

  // Non-inverting mode on OC3A and OC3B (pins 5 and 2)
  TCCR3A |= (1 << COM3A1) | (1 << COM3B1);

  // Choose prescaler = 8 -> CS31 = 1
  TCCR3B |= (1 << CS30);

  // Compute TOP for desired PWM frequency:
  // f_pwm = F_CPU / (prescaler * (1 + ICR3))
  // => ICR3 = F_CPU / (prescaler * f_pwm) - 1
  uint16_t top = (CPU_FREQ / (1UL * PWM_FREQ)) - 1;
  ICR3 = top;

  // Start with 50% duty for both
  OCR3A = top / 2;  // buck_1 duty
  OCR3B = top / 2;  // buck_2 duty

}

// TIMER1 (ADC trigger + ISR)
void setup_sampling_on_timer1() {
  noInterrupts();

  TCCR1A = 0;
  TCCR1B = 0;

  // CTC mode (Clear Timer on Compare Match with OCR1A as TOP)
  TCCR1B |= (1 << WGM12);

  // Prescaler = 8 -> CS11 = 1
  TCCR1B |= (1 << CS11);

  // Compute OCR1A for required sample frequency:
  // f_s = F_CPU / (prescaler * (1 + OCR1A))
  // => OCR1A = F_CPU / (prescaler * f_s) - 1
  uint16_t ocr1 = (uint16_t)(CPU_FREQ / (8UL * ADC_CONVERSION_RATE) - 1);
  OCR1A = ocr1;
  OCR1B = ocr1 / 2;

  // Enable compare match A interrupt
  TIMSK1 |= (1 << OCIE1A);
  
  interrupts();
}

// ADC SETUP (auto-trigger, fast)
void setup_ADC_autotrigger() {
  ADMUX = (1 << REFS0) | 0;

  ADCSRA = 0;
  ADCSRA |= (1 << ADEN) | (1 << ADATE) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS0);

  ADCSRB = 0;
  ADCSRB |= (1 << ADTS2) | (1 << ADTS0);

  ADCSRA |= (1 << ADSC);
}

void set_buck_1_duty(float duty) {
  if (duty < 0.0f) duty = 0.0f;
  if (duty > 1.0f) duty = 1.0f;
  OCR3A = (uint16_t)(duty * ICR3);
}

void set_buck_2_duty(float duty) {
  if (duty < 0.0f) duty = 0.0f;
  if (duty > 1.0f) duty = 1.0f;
  OCR3B = (uint16_t)(duty * ICR3);
}

ISR(TIMER1_COMPA_vect) {
  
  if (!sampling_done) {
    return;
  }
  sampling_done = false;

  // Determine if it's time for MPPT sampling (10Hz)
  MPPT_counter++;
  if (MPPT_counter >= MPPT_TICKS) {
    MPPT_ready = true;
    MPPT_counter = 0;
  }

  Vpv = ((float)Vpv_adc_reading / ADC_RANGE) * adc_voltage * Vpv_scale;
  Ipv = ((float)Ipv_adc_reading / ADC_RANGE) * adc_voltage * Ipv_scale;
  Ipv = voltage_to_current(Ipv);
  Vbuck_1 = ((float)Vbuck_1_adc_reading / ADC_RANGE) * adc_voltage * Vbuck_1_scale;
  Vbuck_2 = ((float)Vbuck_2_adc_reading / ADC_RANGE) * adc_voltage * Vbuck_2_scale;

  // Reduce power if 24V or higher is detected
  if ( (Vpv >= 24.0f) || (Vbuck_1 >= 24.0f) || (Vbuck_2 >= 24.0f) ) {
    buck_1_duty = 0.0f;
    buck_2_duty = 0.0f;
    overvoltage_detected = true;
    set_buck_1_duty(buck_1_duty);
    set_buck_2_duty(buck_2_duty);
    return;
  }


  if (current_mode == fixed_voltage_mode) {
    buck_1_duty = update_pi(pi_buck_1, Vbuck_1_ref, Vbuck_1, T_s); 
  } else if ( (current_mode == mppt_mode) && (MPPT_ready == true)) {    
    buck_1_duty = update_mppt(Vpv, Ipv);
    MPPT_ready = false;
  }
  
  buck_2_duty = update_pi(pi_buck_2, Vbuck_2_ref, Vbuck_2, T_s); 
  
  set_buck_1_duty(buck_1_duty);
  set_buck_2_duty(buck_2_duty);
}

ISR(ADC_vect) {
  uint16_t raw_value = ADC;
  
  switch (current_adc_channel) {
    case 0:
      Vpv_adc_reading = raw_value;
      break;
    case 1:
      Ipv_adc_reading = raw_value;
      break;
    case 2:
      Vbuck_1_adc_reading = raw_value;
      break;
    case 3:
      Vbuck_2_adc_reading = raw_value;
      sampling_done = true;
      break;
  }

  current_adc_channel++;
  if (current_adc_channel >= NUM_CHANNELS) {
    current_adc_channel = 0;
  }

  uint8_t admux_upper = ADMUX & 0xF0;
  ADMUX = admux_upper | current_adc_channel;

  TIFR1 |= (1 << OCF1B);

}


// Updates the PI
float update_pi(pi_controller &pi, float reference, float measurement, float Ts) {
  float error = reference - measurement;
  pi.integral += pi.ki * error * Ts;
  
  // Anti-windup: clamp integral output range
  if (pi.integral > 1.0f) pi.integral = 1.0f;
  if (pi.integral < -1.0f) pi.integral = -1.0f;
  float output = (pi.kp * error) + pi.integral;

  // Saturate output
  if (output > pi.out_max) output = pi.out_max;
  if (output < pi.out_min) output = pi.out_min;
  
  return output;
}

float update_mppt(float Vpv, float Ipv) {
  float Ppv = Vpv * Ipv;
  float Ppv_prev = Vpv_prev * Ipv_prev;
  float dPpv = Ppv - Ppv_prev;  // Power change (positive going up)
  float dVpv = Vpv - Vpv_prev;  // Panel voltage change (positive going up)
  float duty_step = 0.01f;
  if (dPpv > 0.0f) {
    if (dVpv > 0.0f) {
      mppt_duty += duty_step;
    } else {
      mppt_duty -= duty_step;
    }
  } else {
    if (dVpv > 0.0f) {
      mppt_duty -= duty_step;
    } else {
      mppt_duty += duty_step;
    }
  }
  // Clamp duty cycle
  if (mppt_duty < 0.0f) mppt_duty = 0.0f;
  if (mppt_duty > 1.0f) mppt_duty = 1.0f;
  // Update previous values
  Vpv_prev = Vpv;
  Ipv_prev = Ipv;
  return mppt_duty;
}