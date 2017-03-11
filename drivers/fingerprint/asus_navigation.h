#ifndef ASUS_NAVIGATION_H
#define ASUS_NAVIGATION_H

#include <linux/input.h>

#define IOCTL_CMD_SWIPE_UP 827
#define IOCTL_CMD_SWIPE_DOWN 828
#define IOCTL_CMD_SWIPE_LEFT 829
#define IOCTL_CMD_SWIPE_RIGHT 830
#define IOCTL_CMD_TAP 831
#define IOCTL_CMD_DTAP 832
#define IOCTL_CMD_LONGPRESS 833

struct asus_fp_keycodes_s {
	int ioctl_key;
	unsigned int keycode;
};

const struct asus_fp_keycodes_s FP_KEYCODES[] = {
	{IOCTL_CMD_SWIPE_UP,	KEYCODE_FINGERPRINT_SWIPE_UP},
	{IOCTL_CMD_SWIPE_DOWN,	KEYCODE_FINGERPRINT_SWIPE_DOWN},
	{IOCTL_CMD_SWIPE_LEFT,	KEYCODE_FINGERPRINT_SWIPE_LEFT},
	{IOCTL_CMD_SWIPE_RIGHT,	KEYCODE_FINGERPRINT_SWIPE_RIGHT},
	{IOCTL_CMD_TAP,		KEYCODE_FINGERPRINT_TAP},
	{IOCTL_CMD_DTAP,	KEYCODE_FINGERPRINT_DTAP},
	{IOCTL_CMD_LONGPRESS,	KEYCODE_FINGERPRINT_LONGPRESS},
	{26,			26},
};

inline unsigned int asus_translate_keycode(int ioctl_key)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(FP_KEYCODES); ++i)
		if (ioctl_key == FP_KEYCODES[i].ioctl_key)
			return FP_KEYCODES[i].keycode;
	return 0;
}

#endif
