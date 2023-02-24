// -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
// Copyright (C) 2013 Henner Zeller <h.zeller@acm.org>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation version 2.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://gnu.org/licenses/gpl-2.0.txt>

#define __STDC_FORMAT_MACROS
#include <inttypes.h>

#include "comm-h3.h"
#include "gpio-h3.h"
#include "gpio.h"

#include <assert.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

//static const off_t GPIO_REG_BASE=0x01C20000;
//static const size_t GPIO_REG_OFF=0x800;
//static const size_t GPIO_REG_LEN=0x1800;


//static uint32_t*   h3_gpio_hack=NULL;
struct gpio_t      hub75_gpio[14] = {0};
struct gpio_bank_t hub75_bank;



/*
 * nanosleep() takes longer than requested because of OS jitter.
 * In about 99.9% of the cases, this is <= 25 microcseconds on
 * the Raspberry Pi (empirically determined with a Raspbian kernel), so
 * we substract this value whenever we do nanosleep(); the remaining time
 * we then busy wait to get a good accurate result.
 *
 * You can measure the overhead using DEBUG_SLEEP_JITTER below.
 *
 * Note: A higher value here will result in more CPU use because of more busy
 * waiting inching towards the real value (for all the cases that nanosleep()
 * actually was better than this overhead).
 *
 * This might be interesting to tweak in particular if you have a realtime
 * kernel with different characteristics.
 */
#define EMPIRICAL_NANOSLEEP_OVERHEAD_US 12

/*
 * In case of non-hardware pulse generation, use nanosleep if we want to wait
 * longer than these given microseconds beyond the general overhead.
 * Below that, just use busy wait.
 */
#define MINIMUM_NANOSLEEP_TIME_US 5

/* In order to determine useful values for above, set this to 1 and use the
 * hardware pin-pulser.
 * It will output a histogram atexit() of how much how often we were over
 * the requested time.
 * (The full histogram will be shifted by the EMPIRICAL_NANOSLEEP_OVERHEAD_US
 *  value above. To get a full histogram of OS overhead, set it to 0 first).
 */
#define DEBUG_SLEEP_JITTER 0


/* RPI Constants that shouldn't be used */
#define BCM2708_PERI_BASE        0x20000000
#define BCM2709_PERI_BASE        0x3F000000
#define BCM2711_PERI_BASE        0xFE000000
#define GPIO_REGISTER_OFFSET         0x200000
#define COUNTER_1Mhz_REGISTER_OFFSET   0x3000
#define GPIO_PWM_BASE_OFFSET	(GPIO_REGISTER_OFFSET + 0xC000)
#define GPIO_CLK_BASE_OFFSET	0x101000
#define REGISTER_BLOCK_SIZE (4*1024)
#define PWM_CTL      (0x00 / 4)
#define PWM_STA      (0x04 / 4)
#define PWM_RNG1     (0x10 / 4)
#define PWM_FIFO     (0x18 / 4)
#define PWM_CTL_CLRF1 (1<<6)	// CH1 Clear Fifo (1 Clears FIFO 0 has no effect)
#define PWM_CTL_USEF1 (1<<5)	// CH1 Use Fifo (0=data reg transmit 1=Fifo used for transmission)
#define PWM_CTL_POLA1 (1<<4)	// CH1 Polarity (0=(0=low 1=high) 1=(1=low 0=high)
#define PWM_CTL_SBIT1 (1<<3)	// CH1 Silence Bit (state of output when 0 transmission takes place)
#define PWM_CTL_MODE1 (1<<1)	// CH1 Mode (0=pwm 1=serialiser mode)
#define PWM_CTL_PWEN1 (1<<0)	// CH1 Enable (0=disable 1=enable)
#define PWM_STA_EMPT1 (1<<1)
#define PWM_STA_FULL1 (1<<0)
#define CLK_PASSWD  (0x5A<<24)
#define CLK_CTL_MASH(x)((x)<<9)
#define CLK_CTL_BUSY    (1 <<7)
#define CLK_CTL_KILL    (1 <<5)
#define CLK_CTL_ENAB    (1 <<4)
#define CLK_CTL_SRC(x) ((x)<<0)
#define CLK_CTL_SRC_PLLD 6  /* 500.0 MHz */
#define CLK_DIV_DIVI(x) ((x)<<12)
#define CLK_DIV_DIVF(x) ((x)<< 0)
#define CLK_PWMCTL 40
#define CLK_PWMDIV 41
#define MAX_PWM_BIT_USE 224
#define PWM_BASE_TIME_NS 2

// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x).
//#define INP_GPIO(g) *(s_GPIO_registers+((g)/10)) &= ~(7ull<<(((g)%10)*3))
//#define OUT_GPIO(g) *(s_GPIO_registers+((g)/10)) |=  (1ull<<(((g)%10)*3))

//#define GPIO_SET *(gpio+7)  // sets   bits which are 1 ignores bits which are 0
//#define GPIO_CLR *(gpio+10) // clears bits which are 1 ignores bits which are 0

// We're pre-mapping all the registers on first call of GPIO::Init(),
// so that it is possible to drop privileges afterwards and still have these
// usable.
static volatile uint32_t *s_GPIO_registers = NULL;
static volatile uint32_t *s_Timer1Mhz = NULL;
//static volatile uint32_t *s_PWM_registers = NULL;
//static volatile uint32_t *s_CLK_registers = NULL;

namespace rgb_matrix {
#define GPIO_BIT(x) (1ull << x)

GPIO::GPIO() : output_bits_(0), input_bits_(0), reserved_bits_(0),
               slowdown_(1)
#ifdef ENABLE_WIDE_GPIO_COMPUTE_MODULE
             , uses_64_bit_(false)
#endif
{
}

gpio_bits_t GPIO::InitOutputs(gpio_bits_t outputs,
                              bool adafruit_pwm_transition_hack_needed) {
  if (s_GPIO_registers == NULL) {
    fprintf(stderr, "Attempt to init outputs but not yet Init()-ialized.\n");
    return 0;
  }

  gpio_init(&hub75_gpio[0], "PA12");	
  gpio_init(&hub75_gpio[1], "PA11");	
  gpio_init(&hub75_gpio[2], "PA06");
	gpio_init(&hub75_gpio[3], "PA01");
  gpio_init(&hub75_gpio[4], "PA00");
	gpio_init(&hub75_gpio[5], "PA03");
	gpio_init(&hub75_gpio[6], "PA19");
	gpio_init(&hub75_gpio[7], "PA07");
  gpio_init(&hub75_gpio[8], "PA08");
	gpio_init(&hub75_gpio[9], "PA09");
	gpio_init(&hub75_gpio[10], "PA10");
	gpio_init(&hub75_gpio[11], "PA20");
	gpio_init(&hub75_gpio[12], "PA13");
	gpio_init(&hub75_gpio[13], "PA14");  

	DBG_MSG("PA00 data pointer val = 0x%08X", hub75_gpio[4].dat_ptr);  
	DBG_MSG("PA13 data pointer val = 0x%08X", hub75_gpio[12].dat_ptr);    

	for ( uint32_t i = 0; i < 14; i++ ) {
		hub75_bank.gpio[i] = &hub75_gpio[i];
	}
	hub75_bank.size = 14;

  gpio_bank_set_output(&hub75_bank);  

  // Epic Hack: get the memory address to the uint32_t that controls PORTA on Allwinner H3
  // directly modify the bits here to control the GPIO output.
  h3_gpio_hack = &*(hub75_gpio[12].dat_ptr); 

  /*
  *h3_gpio_hack = 0x00001000; // SET PA13
  DBG_MSG("PORTA val = 0x%08X", *h3_gpio_hack);        
  sleep(1);
    *h3_gpio_hack = 0x00000000; // clear PA13
    DBG_MSG("PORTA val = 0x%08X", *h3_gpio_hack);        
    gpio_set_output_value(&hub75_gpio[1], 1);        
  sleep(2); 

 //  *h3_gpio_hack =  ((uint32_t)1<<(10)); // set PA10
    gpio_set_output_value(&hub75_gpio[10], 1);
  	DBG_MSG("PA10 data pointer val = 0x%08X", hub75_gpio[12].dat_ptr);    

    *h3_gpio_hack = ((uint32_t)1<<(20)); // enable PA20

  	DBG_MSG("PORTA val = 0x%08X", *h3_gpio_hack);        
    DBG_MSG("PORTA val calc'd = 0x%08X", ((uint32_t)1<<(20)));  

  sleep(2);   
  */


/*
  // Hack: for the PWM mod, the user soldered together GPIO 18 (new OE)
  // with GPIO 4 (old OE).
  // Since they are connected inside the HAT, want to make extra sure that,
  // whatever the outside system set as pinmux, the old OE is _not_ also
  // set as output so that these GPIO outputs don't fight each other.
  //
  // So explicitly set both of these pins as input initially, so the user
  // can switch between the two modes "adafruit-hat" and "adafruit-hat-pwm"
  // without trouble.
  if (adafruit_pwm_transition_hack_needed) {
    INP_GPIO(4);
    INP_GPIO(18);
    // Even with PWM enabled, GPIO4 still can not be used, because it is
    // now connected to the GPIO18 and thus must stay an input.
    // So reserve this bit if it is not set in outputs.
    reserved_bits_ = GPIO_BIT(4) & ~outputs;
  }

  outputs &= ~(output_bits_ | input_bits_ | reserved_bits_);
#ifdef ENABLE_WIDE_GPIO_COMPUTE_MODULE
  const int kMaxAvailableBit = 45;
  uses_64_bit_ |= (outputs >> 32) != 0;
#else
  const int kMaxAvailableBit = 31;
#endif
  for (int b = 0; b <= kMaxAvailableBit; ++b) {
    if (outputs & GPIO_BIT(b)) {
      INP_GPIO(b);   // for writing, we first need to set as input.
      OUT_GPIO(b);
    }
  }
  output_bits_ |= outputs;

  */

  return outputs;
}

gpio_bits_t GPIO::RequestInputs(gpio_bits_t inputs) {
  if (s_GPIO_registers == NULL) {
    fprintf(stderr, "Attempt to init inputs but not yet Init()-ialized.\n");
    return 0;
  }
  /*

  inputs &= ~(output_bits_ | input_bits_ | reserved_bits_);

#ifdef ENABLE_WIDE_GPIO_COMPUTE_MODULE
  const int kMaxAvailableBit = 45;
  uses_64_bit_ |= (inputs >> 32) != 0;
#else
  const int kMaxAvailableBit = 31;
#endif
  for (int b = 0; b <= kMaxAvailableBit; ++b) {
    if (inputs & GPIO_BIT(b)) {
      INP_GPIO(b);
    }
  }
  input_bits_ |= inputs;
  */

  return inputs;
}

// We are not interested in the _exact_ model, just good enough to determine
// What to do.
enum RaspberryPiModel {
  PI_MODEL_1,
  PI_MODEL_2,
  PI_MODEL_3,
  PI_MODEL_4
};

static int ReadFileToBuffer(char *buffer, size_t size, const char *filename) {
  const int fd = open(filename, O_RDONLY);
  if (fd < 0) return -1;
  ssize_t r = read(fd, buffer, size - 1); // assume one read enough
  buffer[r >= 0 ? r : 0] = '\0';
  close(fd);
  return r;
}

static RaspberryPiModel DetermineRaspberryModel() {
  return PI_MODEL_4;
}

static RaspberryPiModel GetPiModel() {
  static RaspberryPiModel pi_model = DetermineRaspberryModel();
  return pi_model;
}

static int GetNumCores() {
  return GetPiModel() == PI_MODEL_1 ? 1 : 4;
}

static uint32_t *mmap_bcm_register(off_t register_offset) {

  uint32_t *result = gpio_system_init();

  DBG_MSG("got gpio ptr to = 0x%08X", result);

  return result;

/*
  off_t base = GPIO_REG_BASE;  // safe fallback guess.

  int mem_fd;
  if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
    // Try to fall back to /dev/gpiomem. Unfortunately, that device
    // is implemented in a way that it _only_ supports GPIO, not the
    // other registers we need, such as PWM or COUNTER_1Mhz, which means
    // we only can operate with degraded performance.
    //
    // But, instead of failing, mmap() then silently succeeds with the
    // unsupported offset. So bail out here.

			fprintf(stderr, "Could not open /dev/mem\n");
      return NULL;
  }

  uint32_t *result =
    (uint32_t*) mmap(NULL,                  // Any adddress in our space will do
                     GPIO_REG_LEN,   // Map length
                     PROT_READ|PROT_WRITE,  // Enable r/w on GPIO registers.
                     MAP_SHARED,
                     mem_fd,                // File to map
                     GPIO_REG_BASE // Offset to bcm register
                     );
  close(mem_fd);

  if (result == MAP_FAILED) {
    perror("mmap error: ");
    fprintf(stderr, "MMapping from base 0x%lx, offset 0x%lx\n",
            GPIO_REG_BASE, GPIO_REG_OFF);
    return NULL;
  }
  return result;
  */
}

static bool mmap_all_bcm_registers_once() {
  if (s_GPIO_registers != NULL) return true;  // alrady done.

  // The common GPIO registers.
  s_GPIO_registers = mmap_bcm_register(GPIO_REGISTER_OFFSET);
  if (s_GPIO_registers == NULL) {
    return false;
  }
/*
  // Time measurement. Might fail when run as non-root.
  uint32_t *timereg = mmap_bcm_register(COUNTER_1Mhz_REGISTER_OFFSET);
  if (timereg != NULL) {
    s_Timer1Mhz = timereg + 1;
  }

  // Hardware pin-pulser. Might fail when run as non-root.
  s_PWM_registers  = mmap_bcm_register(GPIO_PWM_BASE_OFFSET);
  s_CLK_registers  = mmap_bcm_register(GPIO_CLK_BASE_OFFSET);
*/
  return true;
}

bool GPIO::Init(int slowdown) {
  slowdown_ = slowdown;

  // Pre-mmap all bcm registers we need now and possibly in the future, as to
  // allow  dropping privileges after GPIO::Init() even as some of these
  // registers might be needed later.
  if (!mmap_all_bcm_registers_once())
    return false;

/*
  gpio_set_bits_low_ = s_GPIO_registers + (0x1C / sizeof(uint32_t));
  gpio_clr_bits_low_ = s_GPIO_registers + (0x28 / sizeof(uint32_t));
  gpio_read_bits_low_ = s_GPIO_registers + (0x34 / sizeof(uint32_t));
*/

  //gpio_set_bits_low_ =  h3_gpio_hack;
  //gpio_clr_bits_low_ =  h3_gpio_hack;
  //gpio_read_bits_low_ =  h3_gpio_hack;

  
  return true;
}

/*
 * We support also other pinouts that don't have the OE- on the hardware
 * PWM output pin, so we need to provide (impefect) 'manual' timing as well.
 * Hence all various busy_wait_nano() implementations depending on the hardware.
 */

// --- PinPulser. Private implementation parts.
namespace {
// Manual timers.
class Timers {
public:
  static bool Init();
  static void sleep_nanos(long t);
};

// Simplest of PinPulsers. Uses somewhat jittery and manual timers
// to get the timing, but not optimal.
class TimerBasedPinPulser : public PinPulser {
public:
  TimerBasedPinPulser(GPIO *io, gpio_bits_t bits,
                      const std::vector<int> &nano_specs)
    : io_(io), bits_(bits), nano_specs_(nano_specs) {
    if (!s_Timer1Mhz) {
      fprintf(stderr, "FYI: not running as root which means we can't properly "
              "control timing unless this is a real-time kernel. Expect color "
              "degradation. Consider running as root with sudo.\n");
    }
  }

  virtual void SendPulse(int time_spec_number) {
    io_->ClearBits(bits_);
    Timers::sleep_nanos(nano_specs_[time_spec_number]);
    io_->SetBits(bits_);
  }

private:
  GPIO *const io_;
  const gpio_bits_t bits_;
  const std::vector<int> nano_specs_;
};

static bool LinuxHasModuleLoaded(const char *name) {
  FILE *f = fopen("/proc/modules", "r");
  if (f == NULL) return false; // don't care.
  char buf[256];
  const size_t namelen = strlen(name);
  bool found = false;
  while (fgets(buf, sizeof(buf), f) != NULL) {
    if (strncmp(buf, name, namelen) == 0) {
      found = true;
      break;
    }
  }
  fclose(f);
  return found;
}

static void busy_wait_nanos_rpi_1(long nanos);
static void busy_wait_nanos_rpi_2(long nanos);
static void busy_wait_nanos_rpi_3(long nanos);
static void busy_wait_nanos_rpi_4(long nanos);
static void (*busy_wait_impl)(long) = busy_wait_nanos_rpi_3;

// Best effort write to file. Used to set kernel parameters.
static void WriteTo(const char *filename, const char *str) {
  const int fd = open(filename, O_WRONLY);
  if (fd < 0) return;
  (void) write(fd, str, strlen(str));  // Best effort. Ignore return value.
  close(fd);
}

// By default, the kernel applies some throtteling for realtime
// threads to prevent starvation of non-RT threads. But we
// really want all we can get iff the machine has more cores and
// our RT-thread is locked onto one of these.
// So let's tell it not to do that.
static void DisableRealtimeThrottling() {
  if (GetNumCores() == 1) return;   // Not safe if we don't have > 1 core.
  // We need to leave the kernel a little bit of time, as it does not like
  // us to hog the kernel solidly. The default of 950000 leaves 50ms that
  // can generate visible flicker, so we reduce that to 1ms.
  WriteTo("/proc/sys/kernel/sched_rt_runtime_us", "999000");
}

bool Timers::Init() {
  if (!mmap_all_bcm_registers_once())
    return false;

  // Choose the busy-wait loop that fits our Pi.
  switch (GetPiModel()) {
  case PI_MODEL_1: busy_wait_impl = busy_wait_nanos_rpi_1; break;
  case PI_MODEL_2: busy_wait_impl = busy_wait_nanos_rpi_2; break;
  case PI_MODEL_3: busy_wait_impl = busy_wait_nanos_rpi_3; break;
  case PI_MODEL_4: busy_wait_impl = busy_wait_nanos_rpi_4; break;
  }

  DisableRealtimeThrottling();
  // If we have it, we run the update thread on core3. No perf-compromises:
  WriteTo("/sys/devices/system/cpu/cpu3/cpufreq/scaling_governor",
          "performance");
  return true;
}

static uint32_t JitterAllowanceMicroseconds() {
  // If this is a Raspberry Pi with more than one core, we add a bit of
  // additional overhead measured up to the 99.999%-ile: we can allow to burn
  // a bit more busy-wait CPU cycles to get the timing accurate as we have
  // more CPU to spare.
  switch (GetPiModel()) {
  case PI_MODEL_1:
    return EMPIRICAL_NANOSLEEP_OVERHEAD_US;  // 99.9%-ile
  case PI_MODEL_2: case PI_MODEL_3:
    return EMPIRICAL_NANOSLEEP_OVERHEAD_US + 35;  // 99.999%-ile
  case PI_MODEL_4:
    return EMPIRICAL_NANOSLEEP_OVERHEAD_US + 10;  // this one is fast.
  }
  return EMPIRICAL_NANOSLEEP_OVERHEAD_US;
}

void Timers::sleep_nanos(long nanos) {
  // For smaller durations, we go straight to busy wait.

  // For larger duration, we use nanosleep() to give the operating system
  // a chance to do something else.

  // However, these timings have a lot of jitter, so if we have the 1Mhz timer
  // available, we use that to accurately mesure time spent and do the
  // remaining time with busy wait. If we don't have the timer available
  // (not running as root), we just use nanosleep() for larger values.

  if (s_Timer1Mhz) {
    static long kJitterAllowanceNanos = JitterAllowanceMicroseconds() * 1000;
    if (nanos > kJitterAllowanceNanos + MINIMUM_NANOSLEEP_TIME_US*1000) {
      const uint32_t before = *s_Timer1Mhz;
      struct timespec sleep_time = { 0, nanos - kJitterAllowanceNanos };
      nanosleep(&sleep_time, NULL);
      const uint32_t after = *s_Timer1Mhz;
      const long nanoseconds_passed = 1000 * (uint32_t)(after - before);
      if (nanoseconds_passed > nanos) {
        return;  // darn, missed it.
      } else {
        nanos -= nanoseconds_passed; // remaining time with busy-loop
      }
    }
  } else {
    // Not running as root, not having access to 1Mhz timer. Approximate large
    // durations with nanosleep(); small durations are done with busy wait.
    if (nanos > (EMPIRICAL_NANOSLEEP_OVERHEAD_US + MINIMUM_NANOSLEEP_TIME_US)*1000) {
      struct timespec sleep_time
        = { 0, nanos - EMPIRICAL_NANOSLEEP_OVERHEAD_US*1000 };
      nanosleep(&sleep_time, NULL);
      return;
    }
  }

  busy_wait_impl(nanos);  // Use model-specific busy-loop for remaining time.
}

static void busy_wait_nanos_rpi_1(long nanos) {
  if (nanos < 70) return;
  // The following loop is determined empirically on a 700Mhz RPi
  for (uint32_t i = (nanos - 70) >> 2; i != 0; --i) {
    asm("nop");
  }
}

static void busy_wait_nanos_rpi_2(long nanos) {
  if (nanos < 20) return;
  // The following loop is determined empirically on a 900Mhz RPi 2
  for (uint32_t i = (nanos - 20) * 100 / 110; i != 0; --i) {
    asm("");
  }
}

static void busy_wait_nanos_rpi_3(long nanos) {
  if (nanos < 20) return;
  for (uint32_t i = (nanos - 15) * 100 / 73; i != 0; --i) {
    asm("");
  }
}

static void busy_wait_nanos_rpi_4(long nanos) {
  if (nanos < 20) return;
  // Interesting, the Pi4 is _slower_ than the Pi3 ? At least for this busy loop
  for (uint32_t i = (nanos - 5) * 100 / 132; i != 0; --i) {
    asm("");
  }
}

#if DEBUG_SLEEP_JITTER
static int overshoot_histogram_us[256] = {0};
static void print_overshoot_histogram() {
  fprintf(stderr, "Overshoot histogram >= empirical overhead of %dus\n"
          "%6s | %7s | %7s\n",
          JitterAllowanceMicroseconds(), "usec", "count", "accum");
  int total_count = 0;
  for (int i = 0; i < 256; ++i) total_count += overshoot_histogram_us[i];
  int running_count = 0;
  for (int us = 0; us < 256; ++us) {
    const int count = overshoot_histogram_us[us];
    if (count > 0) {
      running_count += count;
      fprintf(stderr, "%s%3dus: %8d %7.3f%%\n", (us == 0) ? "<=" : " +",
              us, count, 100.0 * running_count / total_count);
    }
  }
}
#endif

// A PinPulser that uses the PWM hardware to create accurate pulses.
// It only works on GPIO-12 or 18 though.

} // end anonymous namespace

// Public PinPulser factory
PinPulser *PinPulser::Create(GPIO *io, gpio_bits_t gpio_mask,
                             bool allow_hardware_pulsing,
                             const std::vector<int> &nano_wait_spec) {
  if (!Timers::Init()) return NULL;

  //if (allow_hardware_pulsing && HardwarePinPulser::CanHandle(gpio_mask)) {
 //   return new HardwarePinPulser(gpio_mask, nano_wait_spec);
  //} else {
    return new TimerBasedPinPulser(io, gpio_mask, nano_wait_spec);
  //}
}

// For external use, e.g. in the matrix for extra time.
uint32_t GetMicrosecondCounter() {
  if (s_Timer1Mhz) return *s_Timer1Mhz;

  // When run as non-root, we can't read the timer. Fall back to slow
  // operating-system ways.
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  const uint64_t micros = ts.tv_nsec / 1000;
  const uint64_t epoch_usec = (uint64_t)ts.tv_sec * 1000000 + micros;
  return epoch_usec & 0xFFFFFFFF;
}

} // namespace rgb_matrix
