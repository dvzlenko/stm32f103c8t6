#include "stdio.h"
#include "stdlib.h"
#include "chat.h"
#include "strtok.h"
#include "version.h"

#define PROMPT	"> "

enum {
	CMD_HELP = 0,
	CMD_VER,
	CMD_DATE,
	CMD_LAST
};

char *cmd_list[CMD_LAST] = {
	"help",
	"ver",
	"date",
};

void vChatTask(void *vpars)
{
	char s[64];
	char cmd[64];
	char *c;
	char *tk;
	int i = 0;

	while (1) {
		cdc_write_buf(&cdc_out, PROMPT, sizeof(PROMPT) - 1, 1);
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

		if (strcmp(tk, cmd_list[CMD_VER]) == 0) {
			sniprintf(s, sizeof(s), "%s\r\n", __VERSION);

		} else if (strcmp(tk, cmd_list[CMD_HELP]) == 0) {
			int i;
			char *_s = s;

			for (i = 0; i < CMD_LAST; i++)
				_s += sniprintf(_s, sizeof(s) - strlen(s),
						"%s\r\n", cmd_list[i]);

		} else if (strcmp(tk, cmd_list[CMD_DATE]) == 0) {
			sniprintf(s, sizeof(s), "%d\r\n", xTaskGetTickCount());

		} else
			sniprintf(s, sizeof(s), "E: what?\r\n");
out:
		cdc_write_buf(&cdc_out, s, strlen(s), 1);
	}
}

