#include <stdint.h>
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "FreeRTOS.h"
#include "task.h"
#include "pwm.h"

extern uint32_t SystemCoreClock;

struct pwm {
	unsigned long	hz;
	unsigned long	period;

	GPIO_TypeDef	*gpio;
	uint16_t	gpio_pin;
	uint32_t	gpio_rcc;
	void (*gpio_rcc_init)(uint32_t, FunctionalState);

	uint32_t	afio_rcc;
	void (*afio_rcc_init)(uint32_t, FunctionalState);

	TIM_TypeDef *tim;
	uint32_t	tim_rcc;
	void (*tim_rcc_init)(uint32_t, FunctionalState);

	void (*init)(TIM_TypeDef *, TIM_OCInitTypeDef *);
};

static struct pwm pwm_list[NPWM] = {
	[PWM0] = {
		.gpio = GPIOA,
		.gpio_pin = GPIO_Pin_8,
		.gpio_rcc = RCC_APB2Periph_GPIOA,
		.gpio_rcc_init = RCC_APB2PeriphClockCmd,

		.afio_rcc = RCC_APB2Periph_AFIO,
		.afio_rcc_init = RCC_APB2PeriphClockCmd,

		.tim = TIM1,
		.tim_rcc = RCC_APB2Periph_TIM1,
		.tim_rcc_init = RCC_APB2PeriphClockCmd,
		.init = TIM_OC1Init,
	},
	[PWM1] = {
		.gpio = GPIOA,
		.gpio_pin = GPIO_Pin_9,
		.gpio_rcc = RCC_APB2Periph_GPIOA,
		.gpio_rcc_init = RCC_APB2PeriphClockCmd,

		.afio_rcc = RCC_APB2Periph_AFIO,
		.afio_rcc_init = RCC_APB2PeriphClockCmd,

		.tim = TIM1,
		.tim_rcc = RCC_APB2Periph_TIM1,
		.tim_rcc_init = RCC_APB2PeriphClockCmd,
		.init = TIM_OC2Init,
	},
	[PWM2] = {
		.gpio = GPIOA,
		.gpio_pin = GPIO_Pin_10,
		.gpio_rcc = RCC_APB2Periph_GPIOA,
		.gpio_rcc_init = RCC_APB2PeriphClockCmd,

		.afio_rcc = RCC_APB2Periph_AFIO,
		.afio_rcc_init = RCC_APB2PeriphClockCmd,

		.tim = TIM1,
		.tim_rcc = RCC_APB2Periph_TIM1,
		.tim_rcc_init = RCC_APB2PeriphClockCmd,
		.init = TIM_OC3Init,
	},
};

int pwm_init(enum pwm_id id, unsigned int hz)
{
	struct pwm *pwm;
	TIM_TimeBaseInitTypeDef ts;
	GPIO_InitTypeDef gs;

	if (id >= NPWM || hz > SystemCoreClock)
		return 1;

	pwm = &pwm_list[id];
	pwm->hz = hz;
	pwm->period = SystemCoreClock / hz - 1;

	if (pwm->period > 0xffff)
		return 1;

	pwm->tim_rcc_init(pwm->tim_rcc, ENABLE);
	pwm->gpio_rcc_init(pwm->gpio_rcc, ENABLE);
	pwm->afio_rcc_init(pwm->afio_rcc, ENABLE);

	gs.GPIO_Pin = pwm->gpio_pin;
	gs.GPIO_Mode = GPIO_Mode_AF_PP;
	gs.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(pwm->gpio, &gs);

	ts.TIM_Prescaler = 0;
	ts.TIM_CounterMode = TIM_CounterMode_Up;
	ts.TIM_Period = pwm->period;
	ts.TIM_ClockDivision = 0;
	ts.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(pwm->tim, &ts);

	pwm_set(id, 0);

	return 0;
}

int pwm_set(enum pwm_id id, unsigned int dc)
{
	struct pwm *pwm;
	TIM_OCInitTypeDef  ocs;

	if (id >= NPWM || dc > 100)
		return 1;

	pwm = &pwm_list[id];

	ocs.TIM_OCMode = TIM_OCMode_PWM2;
	ocs.TIM_OutputState = TIM_OutputState_Enable;
	ocs.TIM_OutputNState = TIM_OutputNState_Enable;
	ocs.TIM_Pulse = pwm->period * dc / 100;
	ocs.TIM_OCPolarity = TIM_OCPolarity_Low;
	ocs.TIM_OCNPolarity = TIM_OCNPolarity_High;
	ocs.TIM_OCIdleState = TIM_OCIdleState_Set;
	ocs.TIM_OCNIdleState = TIM_OCIdleState_Reset;
	pwm->init(pwm->tim, &ocs);

	TIM_Cmd(pwm->tim, ENABLE);
	TIM_CtrlPWMOutputs(pwm->tim, ENABLE);

	return 0;
}

#define SENS_GPIO	GPIOB
#define SENS_RCC	RCC_APB2Periph_GPIOB
#define SENS_RCC_INIT	RCC_APB2PeriphClockCmd

#define SENS_PIN1	GPIO_Pin_12
#define SENS_PIN2	GPIO_Pin_13
#define SENS_PIN3	GPIO_Pin_14

#define SENS_PINS	(SENS_PIN1 | SENS_PIN2)

#define PWM_DEL		((200 * configTICK_RATE_HZ) / 1000)
#define PWM_DC1		70
#define PWM_DC		40

#define GATE_TIMEOUT		(3 * 60 * configTICK_RATE_HZ)

void gate_open(enum pwm_id pwm)
{
	pwm_set(pwm, 0);
}

void gate_close(enum pwm_id pwm)
{
	pwm_set(pwm, PWM_DC1);
	vTaskDelay(PWM_DEL);
	pwm_set(pwm, PWM_DC);
}

void gate_reset()
{
	gate_open(PWM0);
	gate_close(PWM1);
}

#define gate_timeout_restart()				\
do {							\
	now = xTaskGetTickCount();			\
	gate_timeout = now + GATE_TIMEOUT;		\
	tick_wrap = gate_timeout < now ? 1 : 0;		\
} while (0)

void pwm_ctl(void *vpars)
{
	int gate_count = 0, tick_wrap, state = 0;
	portTickType t, now, gate_timeout;
	unsigned long sens = 0, sens_old = 0;
	GPIO_InitTypeDef gs = {
		.GPIO_Pin = SENS_PIN1 | SENS_PIN2 | SENS_PIN3,
		.GPIO_Mode = GPIO_Mode_IPD,
		.GPIO_Speed = GPIO_Speed_50MHz,
	};

	SENS_RCC_INIT(SENS_RCC, ENABLE);
	GPIO_Init(SENS_GPIO, &gs);

	t = xTaskGetTickCount();
	gate_reset();

	while (1) {
		unsigned long s;

		vTaskDelay(100);

		now = xTaskGetTickCount();
		if (now < t)
			tick_wrap = 0;
		t = now;

		sens_old = sens;
		s = GPIO_ReadInputData(SENS_GPIO);
		sens = 0;
		if (s & SENS_PIN1)
			sens |= 1;
		if (s & SENS_PIN2)
			sens |= 2;
		if (s & SENS_PIN3)
			sens |= 4;

		if (gate_count && !tick_wrap &&
			xTaskGetTickCount() >= gate_timeout) {
				gate_reset();
				gate_count = 0;
				state = 0;
		}

		if (!(sens ^ sens_old))
			continue;

		if (state == 0 && (sens & 7) == 4) {
			state = 1;
			gate_close(PWM0);
			gate_open(PWM1);

			gate_timeout_restart();
		}

		if (state == 1) {
			if (sens & 6) {
				gate_timeout_restart();
				gate_count = 0;
			} else
				gate_count = 1;
		}
	}
}
