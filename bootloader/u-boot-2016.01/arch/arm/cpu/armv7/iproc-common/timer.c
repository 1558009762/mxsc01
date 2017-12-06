/*
 * Copyright 2014 Broadcom Corporation.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <div64.h>
#include <asm/io.h>
#include <asm/iproc-common/timer.h>
#include <asm/iproc-common/sysmap.h>

extern uint32_t iproc_get_periph_clk(void);

static inline uint64_t timer_global_read(void)
{
	uint64_t cur_tick;
	uint32_t count_h;
	uint32_t count_l;

	do {
		count_h = cpu_to_le32(readl(IPROC_PERIPH_GLB_TIM_REG_BASE +
				TIMER_GLB_HI_OFFSET));
		count_l = cpu_to_le32(readl(IPROC_PERIPH_GLB_TIM_REG_BASE +
				TIMER_GLB_LOW_OFFSET));
		cur_tick = cpu_to_le32(readl(IPROC_PERIPH_GLB_TIM_REG_BASE +
				 TIMER_GLB_HI_OFFSET));
	} while (cur_tick != count_h);

	return (cur_tick << 32) + count_l;
}

void timer_global_init(void)
{
	writel(0, IPROC_PERIPH_GLB_TIM_REG_BASE + TIMER_GLB_CTRL_OFFSET);
	writel(0, IPROC_PERIPH_GLB_TIM_REG_BASE + TIMER_GLB_LOW_OFFSET);
	writel(0, IPROC_PERIPH_GLB_TIM_REG_BASE + TIMER_GLB_HI_OFFSET);
  	writel(cpu_to_le32(TIMER_GLB_TIM_CTRL_TIM_EN),
	       IPROC_PERIPH_GLB_TIM_REG_BASE + TIMER_GLB_CTRL_OFFSET);
}

int timer_init(void)
{
	timer_global_init();
	return 0;
}

unsigned long get_timer(unsigned long base)
{
	uint64_t count;
	uint64_t ret;
	uint64_t tim_clk;
	uint64_t periph_clk;

	count = timer_global_read();

	periph_clk = (uint64_t)(iproc_get_periph_clk()/1000);

	tim_clk = lldiv(periph_clk,
			(((cpu_to_le32(readl(IPROC_PERIPH_GLB_TIM_REG_BASE +
				 TIMER_GLB_CTRL_OFFSET)) &
			TIMER_GLB_TIM_CTRL_PRESC_MASK) >> 8) + 1));

	ret = lldiv(count, (uint32_t)tim_clk);

	/* returns msec */
	return ret - base;
}

void __udelay(unsigned long usec)
{
	uint64_t cur_tick, end_tick;
	uint64_t tim_clk;
	uint64_t periph_clk;

	periph_clk = (uint64_t)(iproc_get_periph_clk()/1000000);

	tim_clk = lldiv(periph_clk,
			(((cpu_to_le32(readl(IPROC_PERIPH_GLB_TIM_REG_BASE +
				 TIMER_GLB_CTRL_OFFSET)) &
			TIMER_GLB_TIM_CTRL_PRESC_MASK) >> 8) + 1));

	cur_tick = timer_global_read();

	end_tick = tim_clk;
	end_tick *= usec;
	end_tick += cur_tick;

	do {
		cur_tick = timer_global_read();

	} while (cur_tick < end_tick);
}

void timer_systick_init(uint32_t tick_ms)
{
	/* Disable timer and clear interrupt status*/
	writel(0, IPROC_PERIPH_PVT_TIM_REG_BASE + TIMER_PVT_CTRL_OFFSET);
	writel(cpu_to_le32(TIMER_PVT_TIM_INT_STATUS_SET),
	       IPROC_PERIPH_PVT_TIM_REG_BASE + TIMER_PVT_STATUS_OFFSET);
	writel(cpu_to_le32((iproc_get_periph_clk()/1000) * tick_ms),
	       IPROC_PERIPH_PVT_TIM_REG_BASE + TIMER_PVT_LOAD_OFFSET);
	writel(cpu_to_le32(TIMER_PVT_TIM_CTRL_INT_EN |
	       TIMER_PVT_TIM_CTRL_AUTO_RELD |
	       TIMER_PVT_TIM_CTRL_TIM_EN),
	       IPROC_PERIPH_PVT_TIM_REG_BASE + TIMER_PVT_CTRL_OFFSET);
}

void timer_systick_isr(void *data)
{
	writel(cpu_to_le32(TIMER_PVT_TIM_INT_STATUS_SET),
	       IPROC_PERIPH_PVT_TIM_REG_BASE + TIMER_PVT_STATUS_OFFSET);
}

/*
 * This function is derived from PowerPC code (read timebase as long long).
 * On ARM it just returns the timer value in msec.
 */
unsigned long long get_ticks(void)
{
	return get_timer(0);
}

/*
 * This is used in conjuction with get_ticks, which returns msec as ticks.
 * Here we just return ticks/sec = msec/sec = 1000
 */
ulong get_tbclk(void)
{
	return 1000;
}
