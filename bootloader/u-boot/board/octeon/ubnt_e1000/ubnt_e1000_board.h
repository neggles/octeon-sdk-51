#include <galileo/core.h>

struct cregs {
	uint8_t addr;
	uint8_t vl;
};

#define MAX_MIX_ENV_VARS             4

#define BOARD_E1000_MAJOR            0
#define BOARD_E1020_MAJOR            1

#define MIN2SEC                      60

/* PCA9548 Registers */
#define PCA9548_I2CADDR              0x70
#define PCA9548_CONFIG_REG           0
#define PCA9548_CHAN_ALLOFF          NO_BIT
#define PCA9548_CHAN0                BIT0
#define PCA9548_CHAN1                BIT1
#define PCA9548_CHAN2                BIT2
#define PCA9548_CHAN3                BIT3
#define PCA9548_CHAN4                BIT4
#define PCA9548_CHAN5                BIT5
#define PCA9548_CHAN6                BIT6
#define PCA9548_CHAN7                BIT7

/* ADT7475 Registers */
#define ADT7475_I2CADDR              0x2e
#define REMOTE1_TEMP_READ_REG        0x25
#define LOCAL_TEMP_READ_REG          0x26
#define REMOTE2_TEMP_READ_REG        0x27
#define TACH1_LOW_REG                0x28
#define TACH1_HIGH_REG               0x29
#define PWM1_CUR_DUTY_REG            0x30
#define PWM2_CUR_DUTY_REG            0x31
#define PWM3_CUR_DUTY_REG            0x32
#define PWM1_CONFIG_REG              0x5c
#define PWM2_CONFIG_REG              0x5d
#define PWM3_CONFIG_REG              0x5e
#define REMOTE1_TR_PWM1_FREQ_REG     0x5f
#define LOCAL_TR_PWM2_FREQ_REG       0x60
#define REMOTE2_TR_PWM3_FREQ_REG     0x61
#define ENHANCE_ACOUS1_REG           0x62
#define PWM1_MIN_DUTY_REG            0x64
#define PWM2_MIN_DUTY_REG            0x65
#define PWM3_MIN_DUTY_REG            0x66
#define REMOTE1_TEMP_TMIN_REG        0x67
#define REMOTE1_THERM_TEMP_LMT_REG   0x6a
#define CONFIG5_REG                  0x7c
#define CONFIG4_REG                  0x7d
#define TACH_OSC_FREQ               90000
#define PWM2STEP                       39


