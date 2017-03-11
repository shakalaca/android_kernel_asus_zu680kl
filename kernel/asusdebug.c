#include <linux/types.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <linux/kernel.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/workqueue.h>
#include <linux/rtc.h>
#include <linux/list.h>
#include <linux/syscalls.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/export.h>

#define SZ_1M 0x00100000
int entering_suspend = 0;
char messages[256];
int first = 0;

extern char evtlog_bootup_reason[50];
char evtlog_poweroff_reason[50];

extern int ASUSEvt_poweroff_reason;

static const char * const ASUSEvt_poweroff_reason_str[] = {
        [0] = "Software",
        [1] = "PS_HOLD/MSM controlled shutdown",
        [2] = "PMIC watchdog",
        [3] = "Keypad_Reset1",
        [4] = "Keypad_Reset2",
        [5] = "Simultaneous power key and reset line",
        [6] = "Reset line/Volume Down Key",
        [7] = "Long Power Key hold",
        [8] = "N/A",
        [9] = "N/A",
        [10] = "N/A",
        [11] = "Charger ENUM_TIMER, BOOT_DONE",
        [12] = "Thermal Fault Tolerance",
        [13] = "Under Voltage Lock Out",
        [14] = "Overtemp",
        [15] = "Stage 3 reset",
};

extern int nSuspendInProgress;
static struct workqueue_struct *ASUSEvtlog_workQueue;
static int g_hfileEvtlog = -MAX_ERRNO;
static int g_bEventlogEnable = 1;
static char g_Asus_Eventlog[ASUS_EVTLOG_MAX_ITEM][ASUS_EVTLOG_STR_MAXLEN];
static int g_Asus_Eventlog_read = 0;
static int g_Asus_Eventlog_write = 0;
static int bootup = -MAX_ERRNO;
static void do_write_event_worker(struct work_struct *work);
static DECLARE_WORK(eventLog_Work, do_write_event_worker);

static struct mutex mA;
#define AID_SDCARD_RW 1015
static void do_write_event_worker(struct work_struct *work)
{
	char buffer[256];
	memset(buffer, 0, sizeof(char)*256);

	if (IS_ERR((const void *)(long int)g_hfileEvtlog)) {
		long size;

		g_hfileEvtlog = sys_open(ASUS_EVTLOG_PATH".txt", O_CREAT|O_RDWR|O_SYNC, 0444);
		sys_chown(ASUS_EVTLOG_PATH".txt", AID_SDCARD_RW, AID_SDCARD_RW);

		size = sys_lseek(g_hfileEvtlog, 0, SEEK_END);
		if (size >= SZ_1M) {
			sys_close(g_hfileEvtlog);
			sys_rmdir(ASUS_EVTLOG_PATH"_old.txt");
			sys_rename(ASUS_EVTLOG_PATH".txt", ASUS_EVTLOG_PATH"_old.txt");
			g_hfileEvtlog = sys_open(ASUS_EVTLOG_PATH".txt", O_CREAT|O_RDWR|O_SYNC, 0444);
		}

                if (bootup < 0) {
		    if (ASUSEvt_poweroff_reason < 0)
                            strcpy(evtlog_poweroff_reason, "Normal");
                    else
                            strcpy(evtlog_poweroff_reason, ASUSEvt_poweroff_reason_str[ASUSEvt_poweroff_reason]);

		    sprintf(buffer, "\n\n---------------System Boot----%s---------\n"
			    "###### Bootup Reason: %s ######\n"
                            "###### Power off Reason: %s ###### \n",
                            ASUS_SW_VER, evtlog_bootup_reason, evtlog_poweroff_reason);

                    printk(KERN_INFO "\n\n---------------System Boot----%s---------\n"
                            "###### Bootup Reason: %s ######\n"
                            "###### Power off Reason: %s ###### \n",
                            ASUS_SW_VER, evtlog_bootup_reason, evtlog_poweroff_reason);
                }

		bootup = sys_write(g_hfileEvtlog, buffer, strlen(buffer));
		sys_close(g_hfileEvtlog);
	}
	if (!IS_ERR((const void *)(long int)g_hfileEvtlog)) {
		int str_len;
		char *pchar;
		long size;

		g_hfileEvtlog = sys_open(ASUS_EVTLOG_PATH".txt", O_CREAT|O_RDWR|O_SYNC, 0444);
		sys_chown(ASUS_EVTLOG_PATH".txt", AID_SDCARD_RW, AID_SDCARD_RW);

		size = sys_lseek(g_hfileEvtlog, 0, SEEK_END);
		if (size >= SZ_1M) {
			sys_close(g_hfileEvtlog);
			sys_rmdir(ASUS_EVTLOG_PATH"_old.txt");
			sys_rename(ASUS_EVTLOG_PATH".txt", ASUS_EVTLOG_PATH"_old.txt");
			g_hfileEvtlog = sys_open(ASUS_EVTLOG_PATH".txt", O_CREAT|O_RDWR|O_SYNC, 0444);
		}

		while (g_Asus_Eventlog_read != g_Asus_Eventlog_write) {
			mutex_lock(&mA);
			str_len = strlen(g_Asus_Eventlog[g_Asus_Eventlog_read]);
			pchar = g_Asus_Eventlog[g_Asus_Eventlog_read];
			g_Asus_Eventlog_read++;
			g_Asus_Eventlog_read %= ASUS_EVTLOG_MAX_ITEM;
			mutex_unlock(&mA);

			if (pchar[str_len - 1] != '\n') {
				pchar[str_len] = '\n';
				pchar[str_len + 1] = '\0';
			}

			sys_write(g_hfileEvtlog, pchar, strlen(pchar));
			sys_fsync(g_hfileEvtlog);
		}
		sys_close(g_hfileEvtlog);
	}
}

extern struct timezone sys_tz;

void ASUSEvtlog(const char *fmt, ...)
{

	va_list args;
	char *buffer;

	if (g_bEventlogEnable == 0)
		return;
	if (!in_interrupt() && !in_atomic() && !irqs_disabled())
		mutex_lock(&mA);

	buffer = g_Asus_Eventlog[g_Asus_Eventlog_write];
	g_Asus_Eventlog_write++;
	g_Asus_Eventlog_write %= ASUS_EVTLOG_MAX_ITEM;

	if (!in_interrupt() && !in_atomic() && !irqs_disabled())
		mutex_unlock(&mA);

	memset(buffer, 0, ASUS_EVTLOG_STR_MAXLEN);
	if (buffer) {
		struct rtc_time tm;
		struct timespec ts;

		getnstimeofday(&ts);
		ts.tv_sec -= sys_tz.tz_minuteswest * 60;
		rtc_time_to_tm(ts.tv_sec, &tm);
		getrawmonotonic(&ts);
		sprintf(buffer, "(%ld)%04d-%02d-%02d %02d:%02d:%02d :", ts.tv_sec, tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
		/*printk(buffer);*/
		va_start(args, fmt);
		vscnprintf(buffer + strlen(buffer), ASUS_EVTLOG_STR_MAXLEN - strlen(buffer), fmt, args);
		va_end(args);
		/*printk(buffer);*/
		queue_work(ASUSEvtlog_workQueue, &eventLog_Work);
	} else {
		printk("ASUSEvtlog buffer cannot be allocated\n");
	}
}
EXPORT_SYMBOL(ASUSEvtlog);

static ssize_t evtlogswitch_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	if(strncmp(buf, "0", 1) == 0) {
		ASUSEvtlog("ASUSEvtlog disable !!");
		printk("ASUSEvtlog disable !!\n");
		flush_work(&eventLog_Work);
		g_bEventlogEnable = 0;
	}
	if (strncmp(buf, "1", 1) == 0) {
		g_bEventlogEnable = 1;
		ASUSEvtlog("ASUSEvtlog enable !!");
		printk("ASUSEvtlog enable !!\n");
	}

	return count;
}
static ssize_t asusevtlog_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	if (count > 256)
		count = 256;

	memset(messages, 0, sizeof(messages));
	if (copy_from_user(messages, buf, count))
		return -EFAULT;
	ASUSEvtlog(messages);

	return count;
}



/*
	ret value:
		0: lock
		1: unlock
*/
static int gJBStatus = 3;
static int JBStatus_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", gJBStatus);
	return 0;
}

static int JBStatus_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, JBStatus_proc_show, NULL);
}

static ssize_t JBStatus_proc_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	char num[10];
	memset(num, 0, sizeof(num));

	/* no data be written */
	if (!count) {
		return 0;
	}

	/* Input size is too large to write our buffer(num) */
	if (count > (sizeof(num) - 1)) {
		return -EINVAL;
	}

	if (copy_from_user(num, buf, count)) {
		return -EFAULT;
	}

	if (strncmp(num, "0", 1) == 0) {
		gJBStatus = 0;
	} else if (strncmp(num, "1", 1) == 0) {
		gJBStatus = 1;
	} else {
		printk("JBStatus unknown data!!\n");
	}

	return count;
}

static int asusdebug_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int asusdebug_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t asusdebug_read(struct file *file, char __user *buf,
size_t count, loff_t *ppos)
{
	return 0;
}

extern int rtc_ready;
static ssize_t asusdebug_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	u8 messages[256] = {0};
	int nLength = strlen("panic");

	if (count > 256)
		count = 256;
	if (copy_from_user(messages, buf, count))
		return -EFAULT;

	if (strncmp(messages, "panic", nLength) == 0) {
		panic("panic test");
	}

	return count;
}

static const struct file_operations proc_evtlogswitch_operations = {
	.write	  = evtlogswitch_write,
};
static const struct file_operations proc_asusevtlog_operations = {
	.write	  = asusevtlog_write,
};
static const struct file_operations proc_asusdebug_operations = {
	.read	   = asusdebug_read,
	.write	  = asusdebug_write,
	.open	   = asusdebug_open,
	.release	= asusdebug_release,
};
static const struct file_operations proc_JBStatus_operations = {
	.open = JBStatus_proc_open,
	.read = seq_read,
	.write = JBStatus_proc_write,
};

static int __init proc_asusdebug_init(void)
{

	proc_create("asusdebug", S_IALLUGO, NULL, &proc_asusdebug_operations);
	proc_create("asusevtlog", S_IRWXUGO, NULL, &proc_asusevtlog_operations);
	proc_create("asusevtlog-switch", S_IRWXUGO, NULL, &proc_evtlogswitch_operations);
	proc_create("JBStatus", S_IRUGO | S_IWUSR, NULL, &proc_JBStatus_operations);
	mutex_init(&mA);

	ASUSEvtlog_workQueue  = create_singlethread_workqueue("ASUSEVTLOG_WORKQUEUE");

	return 0;
}
module_init(proc_asusdebug_init);


