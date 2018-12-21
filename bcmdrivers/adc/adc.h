#ifndef __ADC_H__
#define __ADC_H__

#define MAX_ADC_AUX_CHANNEL     (8)
#define ADC_CHANNEL_NAME        "adc-chan"
#define ADC_READ_TIMEOUT        (HZ*2)

#define ADC_AUXData_RDY_INTR    (1<<3) 

#define ADC_PWR_LDO             (1<<5)
#define ADC_PWR_ADC             (1<<4)
#define ADC_PWR_BG              (1<<3)

#define ADC_CONTROLLER_EN       (1<<16)
#endif
