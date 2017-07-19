#include "stdio.h"
#include "stdlib.h"
#include "chat.h"
#include "strtok.h"
#include "version.h"
#include "lcd.h"

#define PROMPT	"> "

void status_err()
{
//  cdc_write_buf(&cdc_out, "\nERR\n", 5);
	return;
}

void status_ok()
{
//  cdc_write_buf(&cdc_out, "\nOK\n", 4);
	return;
}

void vChatTask(void *vpars)
{
	char s[64];
	char cmd[64];
	char *c;
	char *tk;
	int i = 0;

	while (1) {
		cdc_write_buf(&cdc_out, PROMPT, sizeof(PROMPT), 1);
		memset(cmd, 0, sizeof(cmd));
		c = cmd;

		while (1) {
			i = cdc_read_buf(&cdc_in, c, 1);
			if (i)
				cdc_write_buf(&cdc_out, c, 1, 1);
			else {
				vTaskDelay(10);
				continue;
			}
			if (*c == '\r') {
				cdc_write_buf(&cdc_out, "\n", 1, 1);
				break;
			}
			if (*c == 8) { /* backspace */
				*c = 0;
				if (c > cmd)
					c -= 1;
				continue;
			}
			if (c + 1 < cmd + sizeof(cmd))
				c += 1;
		};

		sniprintf(s, sizeof(s), "OK\r\n");
		tk = _strtok(cmd, " \n\r");
		if (strcmp(tk, "ver") == 0) {
			sniprintf(s, sizeof(s), "%s\r\n", __VERSION);
		} else if (strcmp(tk, "disp") == 0) {
			int l, o, i;

			tk = _strtok(NULL, " \n\r");
			if (!tk) {
				int l;
				char *s;

				for (l = 0; l < SL; l++) {
					s = lcd_getstr(l);
					cdc_write_buf(&cdc_out, s, strlen(s), 1);
					cdc_write_buf(&cdc_out, "\n\r", 2, 1);
				}
				goto out;
			}
			l = atoi(tk);

			tk = _strtok(NULL, " \n\r");
			if (!tk) {
				sniprintf(s, sizeof(s), "E: no offset\r\n");
				goto out;
			}
			o = atoi(tk);

			tk = _strtok(NULL, "\n\r");
			if (!tk) {
				sniprintf(s, sizeof(s), "E: no string\r\n");
				goto out;
			}

			for (i = 0; i < strlen(tk); i++)
				if (tk[i] == '_')
					tk[i] = ' ';

			lcd_setstr(l, o, tk);
		} else
			sniprintf(s, sizeof(s), "E: what?\r\n");
out:
		cdc_write_buf(&cdc_out, s, strlen(s), 1);
	}
}
