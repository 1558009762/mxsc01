#ifndef __RTC_H__
#define __RTC_H__

#define IPROC_BBL_REG_BASE  0x03026000
#define IPROC_BBL_POWER_STS 0x0301c02c
#define IPROC_BBL_AUTH_REG  0x03024c74

#define IPROC_RTC_INTR_PERIODIC 174
#define IPROC_RTC_INTR_ALARM    165

#define IPROC_RTC_ALARM_GRANULARITY 128

#define REG_BIT(x)          (1<<x)

typedef enum
{
    IREG_BBL_RTC_PER        = 0x00000000,
    IREG_BBL_RTC_MATCH      = 0x00000004,
    IREG_BBL_RTC_DIV        = 0x00000008,
    IREG_BBL_RTC_SECOND     = 0x0000000C,
    IREG_BBL_INTERRUPT_EN   = 0x00000010,
    IREG_BBL_INTERRUPT_stat = 0x00000014,
    IREG_BBL_INTERRUPT_clr  = 0x00000018,
    IREG_BBL_CONTROL        = 0x0000001C,
} INDRECT_RTC_REG_ENUM;

typedef enum
{
    BBL_PER_125ms           = 0x00000001,
    BBL_PER_250ms           = 0x00000002,
    BBL_PER_500ms           = 0x00000004,
    BBL_PER_1s              = 0x00000008,
    BBL_PER_2s              = 0x00000010,
    BBL_PER_4s              = 0x00000020,
    BBL_PER_8s              = 0x00000040,
    BBL_PER_16s             = 0x00000080,
    BBL_PER_32s             = 0x00000100,
    BBL_PER_64s             = 0x00000200,
    BBL_PER_128s            = 0x00000400,
    BBL_PER_256s            = 0x00000800,
} RTC_PERIODIC_TIMER_INTERVAL;

#endif

