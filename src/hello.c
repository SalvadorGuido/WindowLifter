#include "S32K144.h"          /* include peripheral declarations S32K144 */
#include "clocks_and_modes.h"
volatile unsigned int idle_counter = 0; /* main loop idle counter */
volatile unsigned int lpit0_ch0_flag_counter = 0;
volatile unsigned int lpit0_ch1_flag_counter = 0;
volatile unsigned int lpit0_ch2_flag_counter = 0;
volatile unsigned int Register;
volatile unsigned int anti_pinch=0;
volatile unsigned char c8_up=0;
volatile unsigned char c8_down=0;
volatile unsigned int  i32_value=0;
volatile int exit_code = 0;

