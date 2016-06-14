#ifndef __SOUND_DEBUG_H
#define __SOUND_DEBUG_H
/**************
Add to open debug log.
**************/

#include <linux/printk.h>
#include <linux/device.h>
extern int testLogOn;

#undef pr_debug
#undef dev_dbg
#undef dev_info

#define pr_debug(fmt, ...)  \
                    do {\
	                    if(testLogOn)    \
	                         pr_err(fmt, ##__VA_ARGS__); \
	                    else  \
	                         dynamic_pr_debug(fmt, ##__VA_ARGS__); \
                    }while(0)

#define dev_dbg(X,fmt, ...)  \
                    do {\
	                    if(testLogOn)    \
	                        pr_err(fmt, ##__VA_ARGS__); \
	                    else  \
	                        do {						     \
                         	      dynamic_dev_dbg(X, fmt, ##__VA_ARGS__); \
                             } while (0);  \
	                  }while(0)

#define dev_info(X,fmt, ...)  \
                    do {\
	                    if(testLogOn)    \
	                         pr_err(fmt, ##__VA_ARGS__); \
	                    else  \
	                         _dev_info(X, fmt, ##__VA_ARGS__); \
                    }while(0)

#endif /* __SOUND_WSS_H */