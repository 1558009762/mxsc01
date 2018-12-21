#ifndef __TOUCH_SCREEN_H__
#define __TOUCH_SCREEN_H__

#define PEN_DOWN_STATUS         1
#define PEN_UP_STATUS           0
#define TS_DEFAULT_FIFO_THES    1

#define X_MIN                   0
#define Y_MIN                   0
#define X_MAX                   0x3FF
#define Y_MAX                   0x3FF

#define TS_PEN_INTR_MASK        (1<<0)
#define TS_FIFO_INTR_MASK       (1<<2)

#define TS_PEN_DOWN             (1<<0)

#define TS_CONTROLLER_EN_BIT    (1<<16)
#define TS_CONTROLLER_PWR_LDO   (1<<5)
#define TS_CONTROLLER_PWR_ADC   (1<<4)
#define TS_CONTROLLER_PWR_BGP   (1<<3)
#define TS_CONTROLLER_PWR_TS    (1<<2)
#define TS_WIRE_MODE_BIT        (1<<1)


#endif
