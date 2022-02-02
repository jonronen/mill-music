/*
 * Arduino-based drum machine
 * Copyright 2012-2022 Jon Ronen-Drori
 *
 *
 * A simple Arduino-based drum machine
 * with low-pass filter, resonance, distortion, and tremolo
 *
 * Inspiration and pieces of code taken from:
 * - Auduino (http://code.google.com/p/tinkerit/wiki/Auduino)
 * - Arduino Octosynth (http://www.instructables.com/id/The-Arduino-OctoSynth/)
 * - sfxr (http://www.drpetter.se/project_sfxr.html)
 * - Speaker PCM (http://www.arduino.cc/playground/Code/PCMAudio)
 */


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "Arduino.h"

// digital inputs
#define TONE1_PIN       6
#define TONE2_PIN       8
#define TONE3_PIN       9
#define TONE4_PIN       10

#define DIST_PIN        0
#define TREM_PIN        1

// "analog" outputs
#define PWM1_PIN       3
#define PWM2_PIN       5
#define PWM1_VALUE     OCR2B
#define PWM2_VALUE     OCR0B

// interrupt on timer comparison
#define PWM_INTERRUPT TIMER1_COMPA_vect

#define ATTACK_CTRL 0
#define RELEASE_CTRL 1
#define LOW_PASS_CTRL 2
#define RESONANCE_CTRL 3

void setup();
void loop();
static void reset_sample();

//
// Wave type and state
//

static const short g_noise_shit[256] = {
    786,   300,  -316,   337,   153,    91,  -438,   626,   149,   151,  -491,    52,    94,  -714,  -783,   961,
    875,   209,  -381,  -372,   273,   111,   468,  -292,  -979,  -602,  -350,   330,  -533,   395,   126,   650,
    149,  -819,  -265,  -191,   194,  -888,    25,    88,   718,  -966,   276,  -345,   252,   369,   462,  -667,
    903,  -332,  -872,  -632,  -603,  -237,  -813,  -238,   444,  -448,  -833,    -5,  -361,    -1,   250,   468,
    376,   213,  -638,  -216,  -136,  -192,  -321,   869,  -140,   166,  -375,   732,   480,  -107,  -819,  -290,
    -68,  -645,   301,   236,   808,  -191,  -163,   975,   813,  -376,   830,   677,   532,  -538,   371,  -881,
    193,   106,   657,   757,   505,   -62,   153,  -187,   597,   -27,   147,   934,   626,   713,   128,   708,
   -740,  -310,  -325,   739,  -471,  -942,   938,    85,    85,   141,  -538,   519,  -309,  -371,  -962,  -134,
   -693,   253,   -29,   838,   969,  -449,   114,  -286,  -910,   791,  -570,  -649,   763,  -245,  -618,   549,
    935,  -793,   813,  -561,  -919,   613,  -451,   518,  -900,   343,   404,  -288,    37,   174,    40,  -730,
    815,   969,  -678,  -678,   586,   869,   425,  -237,   600,  -404,  -230,   870,   899,  -894,   474,   880,
   -235,   -64,  -149,   302,    -3,  -318,  -955,  -498,  -491,  -542,  -515,   206,   457,   927,  -214,  -661,
  -1015,   398,  -528,   989,  -123,   748,   987,  -967,  -795,  -301,  -394,  -994,   680,    53,   901,  -983,
   -850,  -617,  -589,  -155,   372,  -265,  -731,   856,   821,  -703,  -217,  -227,   109,   -61,  -669,    -7,
    934,  -297,   507,    15,   139,  -657,   243,   678,   689,   806,   347,   620,  -797,   320,  -635,    14,
    947,   408,  -848,   542,  -499,   158,  -736,   681,  -660,  -223,   644,   775,   809,  1008,    36,  -245
};

typedef enum _wave_type {
  WAVE_SAWTOOTH,
  WAVE_TRIANGLE,
  WAVE_NOISE,
  WAVE_SQUARE
} wave_type;

typedef struct _wave_state {
  unsigned short phase;
  unsigned char noise_idx;
  unsigned char type; /* wave_type */
} wave_state;

static void wave_reset(wave_state *wav_state)
{
  wav_state->phase = 0;
  wav_state->noise_idx = 0;
}

static void wave_init(wave_state *wav_state, wave_type type)
{
  wav_state->type = type;
  wave_reset(wav_state);
}

static short wave_step(wave_state *wav_state, unsigned short frequency)
{
  //
  // update the phase, assuming a sample rate of 16,384 Hz
  //

  wav_state->phase += frequency;
  if (wav_state->phase & 0xC000)
  {
    wav_state->phase &= 0x3FFF;
  }

  // base waveform
  unsigned short fp;
  fp = wav_state->phase >> 3; // keep fp between zero and 2047

  short sample = 0;

  switch (wav_state->type) {
  case WAVE_SAWTOOTH:
    sample = (short)1023-(short)fp;
    break;
  case WAVE_TRIANGLE:
    sample = (fp < 1024) ? (short)1023-(short)fp*2 : (short)fp*2-3072;
    break;
  case WAVE_NOISE:
    sample = g_noise_shit[wav_state->noise_idx];
    wav_state->noise_idx++;
    break;
  default: /* WAVE_SQUARE */
    sample = (fp & 1024) ? 1023 : -1023;
    break;
  }

  return sample;
}

// base frequency generation
wave_state g_wave_state;

// tone frequency
static unsigned short g_base_freq;

//
// envelope
//

typedef enum _env_stage {
  ATTACK = 0,
  SUSTAIN = 1,
  RELEASE = 2,
  SILENCE = 3
} env_stage;

typedef enum _env_length_index {
  ATTACK_LEN_IDX,
  RELEASE_LEN_IDX,
  ENV_LEN_IDX_MAX
} env_length_index;

typedef struct _envelope_state {
  unsigned char stage;
  unsigned char sustain;
  unsigned short phase;
  unsigned short lengths[ENV_LEN_IDX_MAX];
} envelope_state;

static void envelope_reset(envelope_state *env_state)
{
  env_state->stage=SILENCE;
  env_state->phase=0;
}

static void envelope_init(envelope_state *env_state, unsigned char sustain_enabled)
{
  env_state->lengths[ATTACK_LEN_IDX] = 0;
  env_state->lengths[RELEASE_LEN_IDX] = 0;
  env_state->sustain = sustain_enabled;
  envelope_reset(env_state);
}

static void envelope_start(envelope_state *env_state)
{
  env_state->stage = ATTACK;
}

static void envelope_release(envelope_state *env_state)
{
  if (env_state->stage < RELEASE) {
    env_state->stage = RELEASE;
  }
}

static unsigned char envelope_step(envelope_state *env_state)
{
  unsigned char env_vol = 0xff;

  // compute by stage, keeping env_vol between zero and 0xff
  switch (env_state->stage) {
  case ATTACK:
    env_state->phase++;
    if (env_state->phase >= env_state->lengths[ATTACK_LEN_IDX]) {
      env_state->phase = 0;
      env_state->stage = env_state->sustain ? SUSTAIN : RELEASE;
    }
    env_vol = (unsigned char)(env_state->phase / (env_state->lengths[ATTACK_LEN_IDX]/0x100 + 1));
    break;
  case SUSTAIN:
    env_vol = 255;
    break;
  case RELEASE:
    env_state->phase++;
    if (env_state->phase >= env_state->lengths[RELEASE_LEN_IDX]) {
      env_state->phase = 0;
      env_state->stage = SILENCE;
    }
    // env_vol can only decrease on RELEASE stage
    unsigned char tmp = 255-(unsigned char)(env_state->phase / (env_state->lengths[RELEASE_LEN_IDX]/0x100 + 1));
    if (tmp < env_vol) {
      env_vol = tmp;
    }
    break;
  default: /* case SILENCE */
    // at stage three there's nothing to play
    env_vol = 0;
    break;
  }

  return env_vol;
}

static envelope_state g_env_state;

// low-pass filter
static unsigned char g_lpf_resonance;
static unsigned char g_lpf_freq;
static short g_lpf_prev;
static short g_lpf_prev_delta;

// tremolo and distortion
static unsigned short g_interrupt_cnt;

// buttons
static uint8_t g_button_status;
static uint8_t g_current_pitch_bit;
static uint8_t g_dist_status;
static uint8_t g_trem_status;

static const unsigned char g_rand_shit[256] = {
  127, 245, 223, 255, 254, 255, 255, 255, 255, 191, 254, 255, 255, 255, 255, 223,
  255, 191, 254, 247, 255, 255, 255, 255, 223, 255, 61,  255, 183, 255, 255, 255,
  255, 191, 249, 255, 255, 245, 255, 250, 215, 255, 255, 255, 127, 255, 255, 255,
  255, 223, 255, 255, 255, 255, 255, 255, 255, 127, 255, 254, 243, 255, 239, 255,
  237, 255, 255, 255, 255, 247, 255, 255, 255, 255, 187, 255, 251, 251, 255, 223,
  255, 253, 255, 255, 255, 255, 127, 255, 191, 255, 255, 255, 255, 255, 223, 255,
  255, 251, 123, 255, 253, 254, 239, 251, 255, 255, 223, 255, 255, 255, 255, 255,
  251, 255, 251, 223, 255, 231, 251, 255, 255, 255, 221, 255, 251, 255, 255, 223,
  255, 219, 255, 245, 255, 255, 255, 255, 255, 255, 251, 255, 59,  255, 255, 223,
  255, 191, 255, 127, 255, 223, 255, 127, 255, 255, 255, 255, 255, 255, 255, 119,
  255, 255, 255, 253, 255, 255, 253, 255, 255, 254, 252, 247, 255, 255, 255, 62,
  255, 251, 247, 255, 189, 255, 255, 255, 255, 126, 251, 253, 255, 255, 191, 255,
  255, 127, 222, 255, 159, 255, 255, 249, 255, 255, 191, 255, 254, 255, 255, 255,
  255, 207, 221, 251, 253, 255, 255, 255, 190, 191, 255, 255, 255, 255, 127, 255,
  255, 243, 255, 247, 255, 255, 253, 255, 255, 223, 255, 255, 255, 251, 255, 255,
  255, 191, 255, 254, 191, 255, 255, 255, 251, 255, 255, 191, 255, 255, 255, 255
};

void setup()
{
  pinMode(PWM1_PIN, OUTPUT);
  pinMode(PWM2_PIN, OUTPUT);
  
  // Set up Timer 2 to do pulse width modulation on the speaker
  // pin.
  
  // Use internal clock (datasheet p.160)
  ASSR &= ~(_BV(EXCLK) | _BV(AS2));
  
  // Set fast PWM mode  (p.157)
  TCCR2A |= _BV(WGM21) | _BV(WGM20);
  TCCR2B &= ~_BV(WGM22);
  TCCR0A |= _BV(WGM01) | _BV(WGM00);
  TCCR0B &= ~_BV(WGM02);
  
  // Do non-inverting PWM on pins OC0B and OC2B (p.155)
  // On the Arduino this is pins 3 and 5.
  TCCR0A = (TCCR0A | _BV(COM0B1)) & ~_BV(COM0B0);
  TCCR0A &= ~(_BV(COM0A1) | _BV(COM0A0));
  TCCR2A = (TCCR2A | _BV(COM2B1)) & ~_BV(COM2B0);
  TCCR2A &= ~(_BV(COM2A1) | _BV(COM2A0));
  
  // No prescaler (p.158)
  TCCR0B = (TCCR0B & ~(_BV(CS02) | _BV(CS01))) | _BV(CS00);
  TCCR2B = (TCCR2B & ~(_BV(CS22) | _BV(CS21))) | _BV(CS20);
  
  // Set initial pulse width to the first sample.
  PWM1_VALUE = 0x80;
  PWM2_VALUE = 0x80;
  
  // Set up Timer 1 to send a sample every interrupt.
  
  cli();
  
  // disable Timer0 - this means that delay() is no longer available
  TIMSK0 &= (~TOIE0);
  
  // Set CTC mode (Clear Timer on Compare Match) (p.133)
  // Have to set OCR1A *after*, otherwise it gets reset to 0!
  TCCR1B = (TCCR1B & ~_BV(WGM13)) | _BV(WGM12);
  TCCR1A = TCCR1A & ~(_BV(WGM11) | _BV(WGM10));
  
  // No prescaler (p.134)
  TCCR1B = (TCCR1B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);
  
  // Set the compare register (OCR1A).
  // OCR1A is a 16-bit register, so we have to do this with
  // interrupts disabled to be safe.
  OCR1A = 976;    // 16e6 / 16384
  
  // Enable interrupt when TCNT1 == OCR1A (p.136)
  TIMSK1 |= _BV(OCIE1A);
  
  // set up the initial values for all the controls

  envelope_init(&g_env_state, 0); /* no sustain in drums */

  wave_init(&g_wave_state, WAVE_TRIANGLE);

  g_base_freq = 440;

  g_lpf_resonance = 0;
  
  // set the parameters but don't start playing
  reset_sample();
  
  // play notes
  pinMode(TONE1_PIN, INPUT);
  digitalWrite(TONE1_PIN, HIGH);
  pinMode(TONE2_PIN, INPUT);
  digitalWrite(TONE2_PIN, HIGH);
  pinMode(TONE3_PIN, INPUT);
  digitalWrite(TONE3_PIN, HIGH);
  pinMode(TONE4_PIN, INPUT);
  digitalWrite(TONE4_PIN, HIGH);
  g_button_status = 0; // no buttons are pressed
  g_current_pitch_bit = 0; // no buttons are pressed

  // distortion
  pinMode(DIST_PIN, INPUT);
  digitalWrite(DIST_PIN, HIGH);
  g_dist_status = 0; // not pressed

  // tremolo
  pinMode(TREM_PIN, INPUT);
  digitalWrite(TREM_PIN, HIGH);
  g_trem_status = 0; // not pressed

  // now enable interrupts
  sei();
}

static unsigned char get_button_status()
{
  unsigned char ans = 0;
  if (digitalRead(TONE1_PIN) == LOW) ans |= 0x01;
  if (digitalRead(TONE2_PIN) == LOW) ans |= 0x02;
  if (digitalRead(TONE3_PIN) == LOW) ans |= 0x04;
  if (digitalRead(TONE4_PIN) == LOW) ans |= 0x08;
  return ans;
}

static unsigned short get_base_freq(unsigned char status)
{
  if (status & 0x08) return 100;
  if (status & 0x04) return 80;
  if (status & 0x02) return 60;
  return 50;
}

void loop()
{
  uint8_t new_status;
  uint8_t tmp;
  uint8_t tmp_pitch_bit;

  //
  // the loop updates the sound parameters
  //
  // since we don't want all the parameters updated when playing,
  // some parameters are updated only on silence and on sustain,
  // and others are updated only on silence
  //

  // envelopes are updated only on silence and sustain
  if ((g_env_state.stage != ATTACK) && (g_env_state.stage != RELEASE)) {
    g_env_state.lengths[ATTACK_LEN_IDX] = 1000; /* analogRead(ATTACK_CTRL) * 4; */
    g_env_state.lengths[RELEASE_LEN_IDX] = 5000; /* 1000 + analogRead(RELEASE_CTRL) * 8; */
  }

  // LPF and resonance are always welcome
  tmp = 0; /* (uint8_t)(analogRead(LOW_PASS_CTRL) / 4); */
  g_lpf_freq = (tmp > 20) ? 255 - tmp : 255;
  tmp = 0; /* analogRead(RESONANCE_CTRL) / 4; */
  g_lpf_resonance = (tmp > 20) ? tmp : 0;

  // so are tremolo and distortion
  g_trem_status = digitalRead(TREM_PIN)==LOW ? 1 : 0;
  g_dist_status = digitalRead(DIST_PIN)==LOW ? 1 : 0;

  //
  // pitch buttons
  //

  new_status = get_button_status();

  // is the current pitch released?
  if ((g_current_pitch_bit & new_status) == 0) {
    envelope_release(&g_env_state);
  }

  // are all buttons released?
  if (new_status == 0) {
    g_current_pitch_bit = 0x00;
  }
  // or is a different pitch selected?
  else if (new_status != g_button_status) {
    tmp_pitch_bit = g_current_pitch_bit;

    // if a new button is pressed - select it
    if (new_status & (~g_button_status)) {
      tmp_pitch_bit = new_status & (~g_button_status);
    }
    // or if the current pitch is released - select another
    else if ((new_status & (~g_current_pitch_bit)) == 0) {
      tmp_pitch_bit = new_status;
    }

    // select only one pitch
    tmp_pitch_bit &= (~(tmp_pitch_bit-1));

    // if it's really a different pitch, play it
    if (tmp_pitch_bit != g_current_pitch_bit) {
      g_current_pitch_bit = tmp_pitch_bit;

      // change the pitch with interrupts disabled
      cli();
      g_base_freq = get_base_freq(g_current_pitch_bit);
      reset_sample();
      envelope_start(&g_env_state);
      sei();
    }
  }

  g_button_status = new_status;
}

static void reset_sample()
{
  wave_reset(&g_wave_state);

  // reset filters
  g_lpf_prev = 0;
  g_lpf_prev_delta = 0;
  
  // reset tremolo, distortion, and shit
  g_interrupt_cnt = 0;
  
  // reset envelope
  envelope_reset(&g_env_state);
}

/* interrupt handler - that's where the synthesis happens */
SIGNAL(PWM_INTERRUPT)
{
  short sample;
  unsigned char env_vol;
  short vib_fix;

  //
  // volume/low-pass envelope
  //

  env_vol = envelope_step(&g_env_state);

  g_interrupt_cnt++;
  // tremolo adjustments
  if (g_trem_status) {
    // do the shit at ~16Hz
    if (g_interrupt_cnt & 0x200) {
      env_vol /= 2;
    }
  }
  
  // distortion adjustments
  if (g_dist_status) {
    env_vol &= g_rand_shit[g_interrupt_cnt & 0xFF];
  }

  //
  // current sample computation
  //
  sample = wave_step(&g_wave_state, g_base_freq); // sample is between -1024 and 1023
  
  //
  // low-pass filter
  //

  // start with the resonance - multiply g_lpf_prev_delta by the factor
  g_lpf_prev_delta /= 0x10;
  g_lpf_prev_delta *= g_lpf_resonance;
  g_lpf_prev_delta /= 0x10;

  //
  // now add the low-pass part
  //
  // since sample and g_lpf_prev are both between -1024 and 1023,
  // we can be sure that g_lpf_prev_delta will not be
  // incremented/decremented by more than 2047
  //
  g_lpf_prev_delta +=
    ((((short)sample-(short)g_lpf_prev) / 0x10) * (short)g_lpf_freq) / 0x10;
  if (g_lpf_prev_delta > 2047) g_lpf_prev_delta = 2047;
  else if (g_lpf_prev_delta < -2048) g_lpf_prev_delta = -2048;
 
  // accumulate it to the filter's output
  g_lpf_prev += g_lpf_prev_delta;

  if (g_lpf_prev > 1023) g_lpf_prev = 1023;
  if (g_lpf_prev < -1024) g_lpf_prev = -1024;

  // filter output
  sample = g_lpf_prev;

  // now sample is between -1024 and 1023
  // scale it between -128 and 127
  sample = sample / 8;
  
  // adjust with the volume envelope
  sample *= env_vol;
  sample = sample / 256;

  PWM1_VALUE=(unsigned char)(sample + 128);
  PWM2_VALUE=(unsigned char)(sample + 128);
}

