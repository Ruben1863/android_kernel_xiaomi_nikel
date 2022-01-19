#include "accdet.h"
//#include "linux/input/setkpd.h"
#ifdef CONFIG_HCT_TP_GESTRUE
#include "ft_gesture_lib.h"
#endif

#define GESTURE_LEFT		0x20
#define GESTURE_RIGHT		0x21
#define GESTURE_UP		0x22
#define GESTURE_DOWN		0x23
#define GESTURE_DOUBLECLICK	0x24
#define GESTURE_O		0x30
#define GESTURE_W		0x31
#define GESTURE_M		0x32
#define GESTURE_E		0x33
#define GESTURE_L		0x44
#define GESTURE_S		0x46
#define GESTURE_V		0x54
#define GESTURE_Z		0x41

#define KEY_GESTURE_U 		KEY_U
#define KEY_GESTURE_UP 		KEY_UP
#define KEY_GESTURE_DOWN 	KEY_DOWN
#define KEY_GESTURE_LEFT 	KEY_LEFT 
#define KEY_GESTURE_RIGHT 	KEY_RIGHT
#define KEY_GESTURE_O 		KEY_O
#define KEY_GESTURE_E 		KEY_E
#define KEY_GESTURE_M 		KEY_M 
#define KEY_GESTURE_L 		KEY_L
#define KEY_GESTURE_W 		KEY_W
#define KEY_GESTURE_S 		KEY_S 
#define KEY_GESTURE_V 		KEY_V
#define KEY_GESTURE_Z 		KEY_Z

#define FTS_DMA_BUF_SIZE	1024

#define FTS_GESTRUE_POINTS 255
#define FTS_GESTRUE_POINTS_ONETIME  62
#define FTS_GESTRUE_POINTS_HEADER 8
#define FTS_GESTURE_OUTPUT_ADRESS 0xD3
#define FTS_GESTURE_OUTPUT_UNIT_LENGTH 4

struct gesture_item{
	int	gesture_id;
	int	action_id;
	char*	name;
};

unsigned short coordinate_x[150] = {0};
unsigned short coordinate_y[150] = {0};

static struct input_dev * kpd_pwrdev;
static char gesture_value[10] = {};
static char enable = 0;
static char version = 2.0;
static int g_call_state = 0;

static struct gesture_item gesture_array[] = 
{
	{GESTURE_LEFT,	KEY_GESTURE_LEFT,	"LEFT"},
	{GESTURE_RIGHT, KEY_GESTURE_RIGHT,	"RIGHT"},
	{GESTURE_UP, 	KEY_GESTURE_UP, 	"UP"},
	{GESTURE_DOWN, 	KEY_GESTURE_DOWN, 	"DOWN"},
	{GESTURE_DOUBLECLICK, KEY_GESTURE_U, 	"DOUBCLICK"},
	{GESTURE_O, 	KEY_GESTURE_O, 		"o"},
	{GESTURE_E, 	KEY_GESTURE_E,		"e"},
	{GESTURE_M, 	KEY_GESTURE_M,		"m"},
	{GESTURE_L,	KEY_GESTURE_L,		"l"},
	{GESTURE_W,	KEY_GESTURE_W,		"w"},
	{GESTURE_S,	KEY_GESTURE_S,		"s"},
	{GESTURE_V,	KEY_GESTURE_V,		"v"},
	{GESTURE_Z,	KEY_GESTURE_Z,		"z"},
	{0}
};

void setkpddev(struct input_dev * input_device) {
	kpd_pwrdev = input_device;
}

static ssize_t show_gesture_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", gesture_value);
}
 
static ssize_t show_version(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", version);
}

static ssize_t show_enable_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", enable);
}
 
static ssize_t store_enable_status(struct device* dev, struct device_attribute *attr,
							const char *buf, size_t count)
{
	if (buf[1] == '\n') {
		if (buf[0] == '0') {
			enable = 0;
		} else if (buf[0] == '1') {
			enable = 1;
		}
	}

	return count;
}

static DEVICE_ATTR(gesture, 0664, show_gesture_value, NULL);
static DEVICE_ATTR(enable, 0664, show_enable_status, store_enable_status);
static DEVICE_ATTR(version, 0664, show_version, NULL);

int fts_Gesture_init(struct input_dev *input_dev)
{
#ifdef CONFIG_HCT_TP_GESTRUE
	struct gesture_item* items = gesture_array;

	init_para(480,854,60,0,0);

	while(items->gesture_id)
	{
		input_set_capability(input_dev, EV_KEY, items->action_id);
		__set_bit(items->action_id, input_dev->keybit);
		items++;
	}
#endif
	return 0;
}

static void tpd_wakeup(struct input_dev *input_dev)
{
         input_report_key(input_dev, KEY_POWER, 1);
		 input_sync(input_dev);
		 input_report_key(input_dev, KEY_POWER, 0);
		 input_sync(input_dev);
}

static void fts_check_gesture(struct input_dev *input_dev,int gesture_id)
{
	struct gesture_item* items = gesture_array;

	printk("Smartwake: gesture: id: 0x%x\n ", gesture_id);
	*gesture_value = 0;


	for(;items->gesture_id;++items)
	{
		if (items->gesture_id != gesture_id) continue;

                if (!enable)
                    break;

	        sprintf(gesture_value, items->name);
	        tpd_wakeup(kpd_pwrdev);
                input_report_key(input_dev, items->action_id, 1);
                input_sync(input_dev);
                input_report_key(input_dev, items->action_id, 0);
                input_sync(input_dev);

		break;
	}
}

static int ft5x0x_read_Touchdata(struct input_dev *input_dev, struct i2c_client* i2c_client)
{
	unsigned char buf[FTS_GESTRUE_POINTS * 4] = { 0 };
	int ret = -1;
	int i = 0;

	int gestrue_id = 0;
	short pointnum = 0;

	buf[0] = 0xd3;
	printk("Smartwake: reading touch data");

	ret = fts_i2c_Read(i2c_client, buf, 1, buf, FTS_GESTRUE_POINTS_HEADER);
	if (ret < 0) {
		return ret;
	}

	if(buf[0] != 0xfe) {
		gestrue_id =  buf[0];
		fts_check_gesture(input_dev, gestrue_id);
		return -1;
	}

	pointnum = (short)(buf[1]) & 0xff;

	buf[0] = 0xd3;
	ret = fts_i2c_Read(i2c_client, buf, 1, buf, pointnum * 4 + 2+6);
	if (ret < 0)
	{
		return ret;
	}

	printk("Smartwake: reading touch data success!");
#ifdef CONFIG_HCT_TP_GESTRUE
	if (!enable)
		return -1;

	gestrue_id = fetch_object_sample(buf, pointnum);
	fts_check_gesture(input_dev, gestrue_id);
#endif
	return -1;
}

 static int touch_getsure_event_handler(struct input_dev *input_dev, struct i2c_client* i2c_client)
{
 	u8 state = 0;
	fts_read_reg(i2c_client, 0xd0, &state);
	if (state !=1)
		return false;

	ft5x0x_read_Touchdata(input_dev, i2c_client);
	return true;
}

static bool tpd_getsure_suspend(struct i2c_client* i2c_client)
{
	fts_write_reg(i2c_client, 0xd0, 0x01);
	fts_write_reg(i2c_client, 0xd1, 0xff);
	fts_write_reg(i2c_client, 0xd2, 0xff);
	fts_write_reg(i2c_client, 0xd5, 0xff);
	fts_write_reg(i2c_client, 0xd6, 0xff);
	fts_write_reg(i2c_client, 0xd7, 0xff);
	fts_write_reg(i2c_client, 0xd8, 0xff);

	return true;
}

static bool tpd_getsure_resume(struct i2c_client* i2c_client)
{
	fts_write_reg(i2c_client, 0xD0, 0x0);

	return true;
}

