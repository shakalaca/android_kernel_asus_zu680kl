#ifndef ASUS_FP_PANEL_EVENT
#define ASUS_FP_PANEL_EVENT

#include <linux/notifier.h>
#include <linux/fb.h>

struct asus_fp_panel_handler {

	struct notifier_block fb_notifier;
	void *data;
	void (*panel_on)(void *data);
	void (*panel_off)(void *data);
};

int asus_fp_add_panel_handler(struct asus_fp_panel_handler *handler);
int asus_fp_on_panel_events(struct notifier_block *self, unsigned long event,
		void *data);

#endif
