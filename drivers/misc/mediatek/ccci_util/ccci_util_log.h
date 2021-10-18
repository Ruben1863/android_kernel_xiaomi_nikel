/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef __CCCI_UTIL_LOG_H__
#define __CCCI_UTIL_LOG_H__

/* No MD id message part */
#define CCCI_UTIL_DBG_MSG(fmt, args...)\
do {\
	ccci_dump_write(0, CCCI_DUMP_INIT, CCCI_DUMP_TIME_FLAG, "[ccci0/util]" fmt, ##args);\
	ccci_dump_write(0, CCCI_DUMP_NORMAL, CCCI_DUMP_TIME_FLAG, "[ccci0/util]" fmt, ##args);\
} while (0)

#define CCCI_UTIL_INF_MSG(fmt, args...)\
do {\
	ccci_dump_write(0, CCCI_DUMP_INIT, CCCI_DUMP_TIME_FLAG, "[ccci0/util]" fmt, ##args);\
	ccci_dump_write(0, CCCI_DUMP_NORMAL, CCCI_DUMP_TIME_FLAG, "[ccci0/util]" fmt, ##args);\
} while (0)

#define CCCI_UTIL_ERR_MSG(fmt, args...)\
do {\
	ccci_dump_write(0, CCCI_DUMP_INIT, CCCI_DUMP_TIME_FLAG, "[ccci0/util]" fmt, ##args);\
	pr_err("[ccci0/util]" fmt, ##args);\
} while (0)

/* With MD id message part */
#define CCCI_UTIL_DBG_MSG_WITH_ID(id, fmt, args...)\
do {\
	ccci_dump_write(id, CCCI_DUMP_INIT, CCCI_DUMP_TIME_FLAG, "[ccci%d/util]" fmt, (id+1), ##args);\
	ccci_dump_write(id, CCCI_DUMP_NORMAL, CCCI_DUMP_TIME_FLAG, "[ccci%d/util]" fmt, (id+1), ##args);\
} while (0)

#define CCCI_UTIL_INF_MSG_WITH_ID(id, fmt, args...)\
do {\
	ccci_dump_write(id, CCCI_DUMP_INIT, CCCI_DUMP_TIME_FLAG, "[ccci%d/util]" fmt, (id+1), ##args);\
	ccci_dump_write(id, CCCI_DUMP_NORMAL, CCCI_DUMP_TIME_FLAG, "[ccci%d/util]" fmt, (id+1), ##args);\
} while (0)

#define CCCI_UTIL_NOTICE_MSG_WITH_ID(id, fmt, args...)\
do {\
	ccci_dump_write(id, CCCI_DUMP_INIT, CCCI_DUMP_TIME_FLAG, "[ccci%d/util]" fmt, (id+1), ##args);\
	pr_notice("[ccci%d/util]" fmt, (id+1), ##args);\
} while (0)

#define CCCI_UTIL_ERR_MSG_WITH_ID(id, fmt, args...)\
do {\
	ccci_dump_write(id, CCCI_DUMP_INIT, CCCI_DUMP_TIME_FLAG, "[ccci%d/util]" fmt, (id+1), ##args);\
	pr_err("[ccci%d/util]" fmt, (id+1), ##args);\
} while (0)

#endif /*__CCCI_UTIL_LOG_H__ */
