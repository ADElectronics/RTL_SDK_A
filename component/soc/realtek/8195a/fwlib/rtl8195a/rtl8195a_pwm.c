/*
 *  Routines to access hardware
 *
 *  Copyright (c) 2013 Realtek Semiconductor Corp.
 *
 * --------------------------
 *  bug fixing: pvvx
 */


#include "rtl8195a.h"
#include "hal_peri_on.h"

#ifdef CONFIG_PWM_EN
#include "rtl8195a_pwm.h"
#include "hal_pwm.h"

//extern HAL_PWM_ADAPTER PWMPin[];

//extern HAL_TIMER_OP HalTimerOp;
extern uint32_t gTimerRecord;

/**
  * @brief  Configure a G-Timer to generate a tick with certain time.
  *
  * @param  pwm_id: the PWM pin index
  * @param  tick_time: the time (micro-second) of a tick
  *
  * @retval None
  */
void 
Pwm_SetTimerTick_8195a(
    HAL_PWM_ADAPTER *pPwmAdapt,
    uint32_t tick_time
)
{
    if (tick_time <= MIN_GTIMER_TIMEOUT) {
        tick_time = MIN_GTIMER_TIMEOUT;
    }
    // Initial a G-Timer for the PWM pin
    if (pPwmAdapt->tick_time != tick_time) {
        pPwmAdapt->tick_time = tick_time;
        DBG_PWM_INFO("%s: Timer_Id=%d Count=%d\n", __FUNCTION__, pPwmAdapt->gtimer_id, tick_time);
        // if timer is running ?
        if(gTimerRecord & (1 << pPwmAdapt->gtimer_id)) {
        	HalTimerReLoadRtl8195a_Patch(pPwmAdapt->gtimer_id, tick_time);
        } else {
            TIMER_ADAPTER TimerAdapter;
            TimerAdapter.IrqDis = 1;    // Disable Irq
            TimerAdapter.IrqHandle.IrqFun = (IRQ_FUN) NULL;
            TimerAdapter.IrqHandle.IrqNum = TIMER2_7_IRQ;
            TimerAdapter.IrqHandle.Priority = 10;
            TimerAdapter.IrqHandle.Data = (uint32_t)NULL;
            TimerAdapter.TimerId = pPwmAdapt->gtimer_id;
            TimerAdapter.TimerIrqPriority = 0;
            TimerAdapter.TimerLoadValueUs = tick_time-1;
            TimerAdapter.TimerMode = 1; // auto-reload with user defined value
            HalTimerInitRtl8195a_Patch((void*) &TimerAdapter);
        }
     }
}


/**
  * @brief  Set the duty ratio of the PWM pin.
  *
  * @param  pwm_id: the PWM pin index
  * @param  period: the period time, in micro-second.
  * @param  pulse_width: the pulse width time, in micro-second.
  *
  * @retval None
  */
void
HAL_Pwm_SetDuty_8195a(
    HAL_PWM_ADAPTER *pPwmAdapt,
    uint32_t period,
    uint32_t pulse_width
)
{
    uint32_t RegAddr;
    uint32_t RegValue;
    uint32_t period_tick;
    uint32_t pulsewidth_tick;
    uint32_t tick_time;
    uint8_t timer_id;
    uint8_t pwm_id;

    pwm_id = pPwmAdapt->pwm_id;
    // Adjust the tick time to a proper value
    if (period < (MIN_GTIMER_TIMEOUT*2)) {
        DBG_PWM_ERR ("HAL_Pwm_SetDuty_8195a: Invalid PWM period(%d), too short!\n", period);
        tick_time = MIN_GTIMER_TIMEOUT;
        period = MIN_GTIMER_TIMEOUT*2;
    }
    else {
        tick_time = period / 1020; // 0x3fc; // a duty cycle be devided into 1020 ticks
        if (tick_time < MIN_GTIMER_TIMEOUT) {
            tick_time = MIN_GTIMER_TIMEOUT;
        }
    }

    Pwm_SetTimerTick_8195a(pPwmAdapt, tick_time);
    tick_time = pPwmAdapt->tick_time;

    period_tick = period/tick_time;
    if (period_tick == 0) {
        period_tick = 1;
    }

    if (pulse_width >= period) {
        pulse_width = period;
    }
    pulsewidth_tick = pulse_width/tick_time;
    
    timer_id = pPwmAdapt->gtimer_id;

    pPwmAdapt->period = period_tick & BIT_MASK_PERI_PWM0_PERIOD;
    pPwmAdapt->pulsewidth = pulsewidth_tick & BIT_MASK_PERI_PWM0_DUTY;
    
    RegAddr = REG_PERI_PWM0_CTRL + (pwm_id*4);
    RegValue = BIT_PERI_PWM0_EN | BIT_PERI_PWM0_GT_SEL(timer_id) | BIT_PERI_PWM0_DUTY(pulsewidth_tick) | BIT_PERI_PWM0_PERIOD(period_tick);
    
    HAL_WRITE32(PERI_ON_BASE, RegAddr, RegValue);
}

/**
  * @brief  Initializes and enable a PWM control pin.
  *
  * @param  pwm_id: the PWM pin index
  * @param  sel: pin mux selection
  * @param  timer_id: the G-timer index assigned to this PWM
  *
  * @retval HAL_Status
  */
HAL_Status 
HAL_Pwm_Init_8195a(
    HAL_PWM_ADAPTER *pPwmAdapt
)
{
    uint32_t pwm_id;
    uint32_t pin_sel;

    pwm_id = pPwmAdapt->pwm_id;
    pin_sel =  pPwmAdapt->sel;
    // Initial a G-Timer for the PWM pin
//p/    Pwm_SetTimerTick_8195a(pPwmAdapt, MIN_GTIMER_TIMEOUT);

    // Set default duty ration
//p/    HAL_Pwm_SetDuty_8195a(pPwmAdapt, 20000, 10000);

    // Configure the Pin Mux
    PinCtrl((PWM0+pwm_id), pin_sel, 1);

    return HAL_OK;
}


/**
  * @brief  Enable a PWM control pin.
  *
  * @param  pwm_id: the PWM pin index
  *
  * @retval None
  */
void 
HAL_Pwm_Enable_8195a(
    HAL_PWM_ADAPTER *pPwmAdapt
)
{
    uint32_t pwm_id;
    
    pwm_id = pPwmAdapt->pwm_id;
    // Configure the Pin Mux
    if (!pPwmAdapt->enable) {
        PinCtrl((PWM0+pwm_id), pPwmAdapt->sel, 1);
        HalTimerEnRtl8195a_Patch(pPwmAdapt->gtimer_id);
        pPwmAdapt->enable = 1;
    }
}


/**
  * @brief  Disable a PWM control pin.
  *
  * @param  pwm_id: the PWM pin index
  *
  * @retval None
  */
void 
HAL_Pwm_Disable_8195a(
    HAL_PWM_ADAPTER *pPwmAdapt
)
{
    uint32_t pwm_id;
    
    pwm_id = pPwmAdapt->pwm_id;
    // Configure the Pin Mux
    if (pPwmAdapt->enable) {
        PinCtrl((PWM0+pwm_id), pPwmAdapt->sel, 0);
        HalTimerDisRtl8195a(pPwmAdapt->gtimer_id);
        pPwmAdapt->enable = 0;
    }
}

#endif  //CONFIG_PWM_EN
