#include "asus_fp_panel_event.h"

#define TAG "AsusFingerprint: "

int asus_fp_add_panel_handler(struct asus_fp_panel_handler *handler)
{
	handler->fb_notifier.notifier_call =
		asus_fp_on_panel_events;
	return fb_register_client(&handler->fb_notifier);
}

int asus_fp_on_panel_events(struct notifier_block *self,
		unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int transition = *(int*)(evdata->data);
	struct asus_fp_panel_handler *handler =
		container_of(self, typeof(*handler), fb_notifier);

	switch (event) {
	case FB_EARLY_EVENT_BLANK:
		if (transition == FB_BLANK_UNBLANK) {
			pr_info(TAG "panel up hook\n");
			handler->panel_on(handler->data);
		}
		if (transition == FB_BLANK_POWERDOWN) {
			pr_info(TAG "panel down hook\n");
			handler->panel_off(handler->data);
		}
		break;
	}
	return 0;
}
