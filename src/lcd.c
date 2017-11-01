#include "string.h"
#include "FreeRTOS.h"
#include "task.h"
#include "LiquidCrystal_I2C.h"
#include "lcd.h"

#define min(a, b)	((a) < (b) ? (a) : (b))

static char buf[SL * (SC + 1)];
static volatile int update;

char *lcd_getstr(int l)
{
	if (l >= SL)
		l = SL - 1;

	return &buf[l * (SC + 1)];
}

void lcd_setstr(int l, int off, char *s)
{
	int i, sl = min(strlen(s), SC - off);

	if (l >= SL)
		return;

	if (off >= SC)
		return;

	memcpy(&buf[l * (SC + 1) + off], s, sl);
	buf[l * (SC + 1) + SC] = 0;
	update |= 1 << l;
}

void lcd_task(void *vpars)
{
	int l;

	for (l = 0; l < SL; l++)
		lcd_setstr(l, 0, "                    ");

	lcd_setstr(3, 0, "T: 25C  H2O: 234 pm");
	lcd_setstr(2, 13, "P: 25mB");

	LCDI2C_init(PCF8574_ADDR, SC, SL);
	LCDI2C_backlight();
	LCDI2C_clear();

	while (1) {
		if (!update) {
			vTaskDelay(200);
			continue;
		}

		for (l = 0; l < SL; l++) {
			if (!(update & (1 << l)))
				continue;
			LCDI2C_setCursor(0, l);
			LCDI2C_write_String(&buf[l * (SC + 1)]);
		}
		update = 0;
	}
}
