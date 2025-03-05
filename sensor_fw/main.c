#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/sys/byteorder.h>
#ifdef CONFIG_LED_STRIP
#include <zephyr/drivers/led_strip.h>
#endif
#ifdef CONFIG_BOARD_NATIVE_SIM
#include <zephyr/drivers/adc/adc_emul.h>
#include <zephyr/random/random.h>
#endif
#ifdef CONFIG_BT
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/mgmt/mcumgr/transport/smp_bt.h>
#endif
#ifdef CONFIG_BT_ZEPHYR_NUS
#include <zephyr/bluetooth/services/nus.h>
#endif
#ifdef CONFIG_WIFI
#include <zephyr/net/net_ip.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/http/client.h>
#include <zephyr/net/net_config.h>
#include <zephyr/mgmt/mcumgr/transport/smp_udp.h>
#endif

#define BKK_ID 0x19
#define BKK_VERSION 0x1
/* light pulse characteristics */
#define LIGHT_MIN 150			/* minimum valid value */
#define LIGHT_MAX 3500			/* maximum valid value */
#define LIGHT_PULSELEN_MAX_MS 75	/* maximum peak duration */
#define LIGHT_PULSELEN_MAX_SAMPLES (LIGHT_PULSELEN_MAX_MS / LIGHT_SAMPLING_MS)
/* light processing configuration */
#define LIGHT_SAMPLING_MS 25		/* 25 ms between ADC samples */
#define LIGHT_SAMPLING_DURATION_MS 5000 /* sample for 5 seconds before processing */
#define LIGHT_SAMPLING_COUNT (LIGHT_SAMPLING_DURATION_MS / LIGHT_SAMPLING_MS)
#define LIGHT_WINDOW_SAMPLE_COUNT 5
#define LIGHT_THRESHOLD_MIN_PERCENT 2.0 /* minimum percentage difference between min and max measurement to compute a threshold */
/* light emulation (for testing with native simulator) */
#ifdef CONFIG_BOARD_NATIVE_SIM
#define LIGHT_EMU_LOW 300
#define LIGHT_EMU_HIGH 1300
#define LIGHT_EMU_COUNT 100
#endif
/* light store settings */
#define LIGHT_STORE_COUNT (LIGHT_SAMPLING_COUNT * 2)
#define LIGHT_QUEUE_SIZE 2
/* bluetooth settings */
#define REPORT_HDR_SIZE (1 + 1 + 1 + 4)	/* ID + version + sensor type + uptime in seconds */
#define REPORT_SIZE (REPORT_HDR_SIZE + CONFIG_BKK_PULSE_STORE_COUNT)
#ifdef CONFIG_BKK_BT_REPORT_FAST
#define BT_INTERVAL_MIN BT_GAP_ADV_FAST_INT_MIN_2 /* 100ms */
#define BT_INTERVAL_MAX BT_GAP_ADV_FAST_INT_MAX_2 /* 150ms */
#else
#define BT_INTERVAL_MIN BT_GAP_ADV_SLOW_INT_MIN /* 1s */
#define BT_INTERVAL_MAX BT_GAP_ADV_SLOW_INT_MAX /* 1.2s */
#endif

LOG_MODULE_REGISTER(bkk, LOG_LEVEL_DBG);

/* devices */
#ifdef CONFIG_BKK_BLINK
static const struct gpio_dt_spec _led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
#endif
static const struct adc_dt_spec _adc_channel = ADC_DT_SPEC_GET(DT_PATH(zephyr_user));

/* network references */
#ifdef CONFIG_BT
static struct bt_le_ext_adv *_adv = NULL;
static struct k_work _bt_adv_work;
static struct bt_conn *_bt_conn = NULL;
#endif
#ifdef CONFIG_WIFI
static struct net_if *_wifi_if;
static bool _wifi_conn = false;
static uint8_t _wifi_rbuf[512];
#endif

/* light sensor */
struct sens_win {
	int32_t min;
	int32_t max;
	int32_t values[LIGHT_WINDOW_SAMPLE_COUNT];
	int pulselen;
};
static struct {
	int16_t store[LIGHT_STORE_COUNT];
	int pos;
	int valid_count;
	struct sens_win win;
} _sens = {
	{ [0 ... LIGHT_STORE_COUNT-1] = -1 },
	0, 0,
	{ INT_MAX, INT_MIN, { [0 ... LIGHT_WINDOW_SAMPLE_COUNT-1] = -1 }, 0, },
};
#ifdef CONFIG_BOARD_NATIVE_SIM
int _emu_pulses = 0;
#endif

/* pulses of light */
static struct {
	uint8_t store[CONFIG_BKK_PULSE_STORE_COUNT];
	int pos;
	int pos_last;
	//int last; /* count since last pulse */
} _puls = {
	{ [0 ... CONFIG_BKK_PULSE_STORE_COUNT-1] = -1 },
	0, 0,
};

static int board_setup(void);
static void light_timer(struct k_timer *timer);
static void light_work(struct k_work *work);
static int light_process(int);
static char *light_str(int, int);
static inline int32_t light_win_read(struct sens_win *, int);
static inline int light_win_detect(struct sens_win *, int32_t);
static char *light_win_str(struct sens_win *);
#ifdef CONFIG_BOARD_NATIVE_SIM
static int light_adc_emul_func(const struct device *, unsigned int, void *, uint32_t *);
#endif
static void pulse_process(int, int64_t);
static void report_cpy(char *, int64_t);
#ifdef CONFIG_BT
static int bt_adv_update(int64_t, int);
static void bt_adv_start(struct k_work *);
static void bt_ready(int);
#endif
#ifdef CONFIG_WIFI
static int wifi_report(int64_t, int);
static void wifi_report_cb(struct http_response *, enum http_final_call, void *);
static void wifi_event(struct net_mgmt_event_callback *, uint32_t, struct net_if *);
static int http_req(int, const char *, const char *, int);
#endif

K_TIMER_DEFINE(_light_timer, light_timer, NULL);
K_WORK_DEFINE(_light_work, light_work);
K_MSGQ_DEFINE(_light_msg, sizeof(int), LIGHT_QUEUE_SIZE, 4); /* queue of int messages, aligned on 4 bytes */

/*
 * Main and board setup
 */

int
main(void)
{
	int store_pos, pulses, ret, reports = 0, failures = 0;
	int64_t uptime, last_report = 0;
	bool report;

	LOG_INF("blinkiki v%d", BKK_VERSION);

	if (board_setup() < 0)
		return 0;

	k_timer_start(&_light_timer, K_MSEC(LIGHT_SAMPLING_MS), K_MSEC(LIGHT_SAMPLING_MS));

	while (1) {
		ret = k_msgq_get(&_light_msg, &store_pos, K_MSEC(LIGHT_SAMPLING_DURATION_MS*1.5));
		uptime = k_uptime_get();
#ifdef CONFIG_BKK_BLINK
		if (gpio_pin_toggle_dt(&_led) < 0)
			LOG_ERR("toggle LED GPIO failed");
#endif
		if (ret == 0) {
			pulses = light_process(store_pos);
			pulse_process(pulses, uptime);
		} else {
			LOG_ERR("no sensor values received");
			pulse_process(-1, uptime);
		}

		report = false;
#ifdef CONFIG_BT
		if (!_bt_conn) {
			report = true;
			ret = bt_adv_update(uptime, reports);
		}
#endif
#ifdef CONFIG_WIFI
		if (_wifi_conn && uptime - last_report > CONFIG_BKK_WIFI_REPORT_INTERVAL * MSEC_PER_SEC) {
			report = true;
			ret = wifi_report(uptime, reports);
		}
#endif
		if (report) {
			reports++;
			if (ret) {
				failures++;
				LOG_INF("report %d failure ! (%d total failures)", reports, failures);
			} else {
				last_report = uptime;
			}
		}
	}

	return 0;
}

static int
board_setup(void)
{
	int ret;

#if defined(CONFIG_CONSOLE) && defined(CONFIG_BOARD_ADAFRUIT_TRINKET_M0)
	LOG_DBG("setup usb console");

	const struct device *const console = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	uint32_t dtr = 0;

	if (usb_enable(NULL)) {
		return -1;
	}
	while ( !dtr )
	{
		uart_line_ctrl_get( console, UART_LINE_CTRL_DTR, &dtr );
		k_sleep(K_MSEC(100));
	}
	LOG_DBG("console Initialised: %s\n", CONFIG_ARCH);
#endif

#ifdef CONFIG_LED_STRIP
	LOG_DBG("setup disable leds");
#define STRIP_NODE              DT_ALIAS(led_strip)
#define STRIP_NUM_PIXELS        DT_PROP(DT_ALIAS(led_strip), chain_length)
	static struct led_rgb pixels[STRIP_NUM_PIXELS];
	static const struct device *const dev_strip = DEVICE_DT_GET(STRIP_NODE);

	if (!device_is_ready(dev_strip)) {
		memset(&pixels, 0x00, sizeof(pixels));
		ret = led_strip_update_rgb(dev_strip, pixels, STRIP_NUM_PIXELS);
		if (ret)
			LOG_ERR("LED strip update failed: %d", ret);
	} else {
		LOG_ERR("LED strip not ready");
	}
#endif /* CONFIG_LED_STRIP */

#ifdef CONFIG_BKK_BLINK
	LOG_DBG("setup led blink");
	if (!gpio_is_ready_dt(&_led)) {
		LOG_ERR("GPIO led not ready");
		return -1;
	}
	ret = gpio_pin_configure_dt(&_led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		LOG_ERR("Could not setup LED GPIO");
		return -1;
	}
#endif /* CONFIG_BKK_BLINK */

#if defined(CONFIG_BT)
	LOG_DBG("setup bluetooth");
	k_work_init(&_bt_adv_work, bt_adv_start);
	ret = bt_enable(bt_ready);
	if (ret < 0) {
		LOG_ERR("bluetooth init failed : %d", ret);
#ifndef CONFIG_BOARD_NATIVE_SIM
		return -1;
#endif
	}
#endif /* CONFIG_BT */

#ifdef CONFIG_WIFI
	LOG_DBG("setup wifi");
	static struct net_mgmt_event_callback wifi_cb;
	net_mgmt_init_event_callback(&wifi_cb, wifi_event, NET_EVENT_WIFI_CONNECT_RESULT | NET_EVENT_WIFI_DISCONNECT_RESULT);
	net_mgmt_add_event_callback(&wifi_cb);
#ifdef CONFIG_BOARD_NATIVE_SIM
	_wifi_conn = true;
#else
	static struct wifi_connect_req_params wifi_conf;
	_wifi_if = net_if_get_wifi_sta();
	if (!_wifi_if) {
		LOG_ERR("wifi: interface not initialized");
		return -1;
	}
	wifi_conf.ssid = (const uint8_t *)CONFIG_BKK_WIFI_SSID;
	wifi_conf.ssid_length = strlen(CONFIG_BKK_WIFI_SSID);
	wifi_conf.psk = (const uint8_t *)CONFIG_BKK_WIFI_PSK;
	wifi_conf.psk_length = strlen(CONFIG_BKK_WIFI_PSK);
	wifi_conf.security = CONFIG_BKK_WIFI_SECURITY;
	wifi_conf.channel = WIFI_CHANNEL_ANY;
	wifi_conf.band = WIFI_FREQ_BAND_2_4_GHZ;
	LOG_DBG("connecting to wifi network '%s'", CONFIG_BKK_WIFI_SSID);
	ret = net_mgmt(NET_REQUEST_WIFI_CONNECT, _wifi_if, &wifi_conf, sizeof(struct wifi_connect_req_params));
	if (ret)
		LOG_ERR("cannot connect to wifi network '%s'", CONFIG_BKK_WIFI_SSID);
#endif
	(void)net_config_init_app(NULL, "Initializing network");
	while (http_req(HTTP_POST, "/sensor/up", NULL, 0))
		k_sleep(K_MSEC(500));
#ifndef CONFIG_BOARD_NATIVE_SIM
	smp_udp_open();
#endif

#endif /* CONFIG_WIFI */

	LOG_DBG("setup light sensor");
	if (!adc_is_ready_dt(&_adc_channel)) {
		LOG_ERR("ADC controller device %s not ready\n", _adc_channel.dev->name);
		return -1;
	}
	ret = adc_channel_setup_dt(&_adc_channel);
	if (ret < 0) {
		LOG_ERR("Could not setup ADC channel (%d)\n", ret);
		return -1;
	}
#ifdef CONFIG_BOARD_NATIVE_SIM
	adc_emul_value_func_set(DEVICE_DT_GET(DT_NODELABEL(adc0)), 0, light_adc_emul_func, NULL);
#endif

	LOG_DBG("board setup done");

	return 0;
}

/*
 * Light timer and processing functions
 */

static void
light_timer(struct k_timer *timer)
{
	k_work_submit(&_light_work);
}

static void
light_work(struct k_work *work)
{
	int16_t measure;
	struct adc_sequence sequence = {
		.buffer = &measure,
		.buffer_size = sizeof(int16_t),
	};
	static int count = 0;
	int ret;

	(void)adc_sequence_init_dt(&_adc_channel, &sequence);

	ret = adc_read_dt(&_adc_channel, &sequence);
	if (ret < 0) {
		LOG_ERR("adc read error, exiting: %d", ret);
		return;
	}
	_sens.store[_sens.pos + count] = measure;

	count++;
	if (count == LIGHT_SAMPLING_COUNT) {
		k_msgq_put(&_light_msg, &_sens.pos, K_NO_WAIT);
		_sens.pos += LIGHT_SAMPLING_COUNT;
		if (_sens.pos == LIGHT_STORE_COUNT)
			_sens.pos = 0;
		count = 0;
	}
}

/* light_process - detect a new pulses of light from store new values */
static int
light_process(int store_pos)
{
	int pulses = 0;
	int32_t threshold;

	//LOG_DBG("%d samples at %d:\n%s", LIGHT_SAMPLING_COUNT, store_pos, light_str(store_pos));
	LOG_DBG("%d samples at %d", LIGHT_SAMPLING_COUNT, store_pos);
	LOG_DBG("%s", light_str(store_pos, 0));
	LOG_DBG("%s", light_str(store_pos, 40));
	LOG_DBG("%s", light_str(store_pos, 80));
	LOG_DBG("%s", light_str(store_pos, 120));
	LOG_DBG("%s", light_str(store_pos, 160));

	for (int pos=0; pos<LIGHT_SAMPLING_COUNT; pos+=LIGHT_WINDOW_SAMPLE_COUNT) {
		threshold = light_win_read(&_sens.win, store_pos + pos);
		if (threshold > 0)
			pulses += light_win_detect(&_sens.win, threshold);
	}

	LOG_DBG("%"PRId32" pulses", pulses);
#ifdef CONFIG_BOARD_NATIVE_SIM
	LOG_DBG("%"PRId32" simulated pulses", _emu_pulses);
	if (pulses != _emu_pulses)
		k_fatal_halt(1);
	_emu_pulses = 0;
#endif

	return pulses;
}

static inline int32_t
adc_conv(int16_t in)
{

	int32_t out = (int32_t)in;
	int ret = adc_raw_to_millivolts_dt(&_adc_channel, &out);
	if (ret < 0) {
		LOG_ERR("adc error converting to millivolts %"PRId32, out);
		out = -1;
	}
	if (out < 0)
		out = 0;
	return out;
}

static char *
light_str(int store_pos, int start)
{
	static char buf[2000], b2[10];

	buf[0] = '\0';
	for (int i=start; i<start+40; i++) {
		int32_t val = adc_conv(_sens.store[store_pos + i]);
		snprintf(b2, sizeof(b2), "%4"PRId32" ", val);
		strcat(buf, b2);
	}
	return buf;
}

static inline int32_t
light_win_read(struct sens_win *win, int store_pos)
{
	int32_t threshold;

	if (!win->pulselen) { /* if no peak in progress, set min and max */
		win->min = INT_MAX;
		win->max = INT_MIN;
	}
	for (int i=0; i<LIGHT_WINDOW_SAMPLE_COUNT; i++) {
		int32_t val = adc_conv(_sens.store[store_pos + i]);
		win->values[i] = val;
		if (val < win->min)
			win->min = val;
		if (val > win->max)
			win->max = val;
	}
	threshold = win->min + ((win->max - win->min) / 2);
	if (threshold < LIGHT_MIN || threshold > LIGHT_MAX
			|| win->max < win->min * LIGHT_THRESHOLD_MIN_PERCENT) {
		//LOG_DBG("at %d incorrect threshold : min %d max %d thresh %d : %s", store_pos, win->min, win->max, threshold, light_win_str(win));
		threshold = -1;
		win->pulselen = 0;
	}
	return threshold;
}

static inline int
light_win_detect(struct sens_win *win, int32_t threshold)
{
	static int32_t prev = -1;
	int pulses = 0;

	for (int i=0; i<LIGHT_WINDOW_SAMPLE_COUNT; i++) {
		int32_t val = win->values[i];
		if (val > threshold) {
			if (win->pulselen == 0) {
				if (prev < 0 || val > prev)
					win->pulselen = 1;
			} else if (win->pulselen > LIGHT_PULSELEN_MAX_SAMPLES) {
				//LOG_DBG("pulse too long");
				win->pulselen = 0;
			} else {
				win->pulselen++;
			}
		} else if (val < threshold) {
			if (win->pulselen) {
				//LOG_DBG("pulse len %d at %d min %d max %d thres %d : %s", win->pulselen, i, win->min, win->max, threshold, light_win_str(win));
				pulses++; /* end of pulse */
				win->pulselen = 0;
			}
		}
		prev = val;
	}

	return pulses;
}

static char *
light_win_str(struct sens_win *win)
{
	static char buf[256], b2[10];
	buf[0] = '\0';
	for (int i=0; i<LIGHT_WINDOW_SAMPLE_COUNT; i++) {
		snprintf(b2, sizeof(b2), "%"PRId32" ", win->values[i]);
		strcat(buf, b2);
	}
	return buf;
}

#ifdef CONFIG_BOARD_NATIVE_SIM
static int
light_adc_emul_func(const struct device *dev, unsigned int chan, void *data, uint32_t *result)
{
	static int counter = 0;
	static int last_pulse = 0;
	static int pulse_len = 0;

	counter += 1;
	if (pulse_len == 3) {
		*result = LIGHT_EMU_LOW + sys_rand32_get() % 100;
		pulse_len = 0;
		last_pulse = counter;
		_emu_pulses++;
	} else if (pulse_len > 0) {
		*result = LIGHT_EMU_HIGH + sys_rand32_get() % 100;
		pulse_len++;
	} else {
		if (_emu_pulses < LIGHT_EMU_COUNT && counter > last_pulse + 4 && sys_rand32_get() % 5 == 0) {
			*result = LIGHT_EMU_HIGH + sys_rand32_get() % 100;
			pulse_len = 1;
		} else {
			*result = LIGHT_EMU_LOW + sys_rand32_get() % 100;
		}
	}
	return 0;
}
#endif

static void
pulse_process(int pulses, int64_t uptime)
{
	_puls.store[_puls.pos] = pulses;
	_puls.pos_last = _puls.pos;
	_puls.pos++;
	if (_puls.pos == CONFIG_BKK_PULSE_STORE_COUNT)
		_puls.pos = 0;
}

static void
report_cpy(char *buf, int64_t uptime)
{
	LOG_DBG("report at %"PRId64" : pulse %u pos_last %d", uptime, _puls.store[_puls.pos_last], _puls.pos_last);

	/* header */
	buf[0] = BKK_ID;
	buf[1] = BKK_VERSION;
	buf[2] = CONFIG_BKK_SENSOR_TYPE;
	sys_put_be32(uptime / 1000, buf+3);
	/* pulses of light */
	for (int i=0, pos=_puls.pos_last; i<CONFIG_BKK_PULSE_STORE_COUNT; i++) {
		buf[REPORT_HDR_SIZE+i] = _puls.store[pos];
		pos--;
		if (pos < 0)
			pos = CONFIG_BKK_PULSE_STORE_COUNT-1;
	}
	//LOG_HEXDUMP_DBG(buf, REPORT_SIZE, "report");
}


/*
 * Bluetooth functions
 */

#ifdef CONFIG_BT
static void
bt_adv_start(struct k_work *work)
{
	struct bt_le_adv_param adv_param = {
		.id = BT_ID_DEFAULT,
		.sid = 0U, /* Supply unique SID when creating advertising set */
		.secondary_max_skip = 0U,
#ifdef CONFIG_BKK_BT_CODED_PHY
		.options = BT_LE_ADV_OPT_EXT_ADV | BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_CODED,
#else
		.options = BT_LE_ADV_OPT_EXT_ADV | BT_LE_ADV_OPT_CONN,
#endif
		.interval_min = BT_INTERVAL_MIN,
		.interval_max = BT_INTERVAL_MAX,
		.peer = NULL,
	};
	int ret;

	LOG_DBG("bt adv start");
	if (!_adv) {
		ret = bt_le_ext_adv_create(&adv_param, NULL, &_adv);
		if (ret) {
			LOG_ERR("bluetooth could not create extended advertisement : %d", ret);
		}
		bt_adv_update(0, 0);
	}
	ret = bt_le_ext_adv_start(_adv, BT_LE_EXT_ADV_START_DEFAULT);
	if (ret) {
		LOG_ERR("bluetooth could not start extended advertisement : %d", ret);
	}
}

static void
bt_ready(int err)
{
	LOG_DBG("bt ready");
	if (err) {
		LOG_ERR("bluetooth failed to initialise: %d", err);
	} else {
		k_work_submit(&_bt_adv_work);
	}
}

static int
bt_adv_update(int64_t uptime, int reports)
{
	static uint8_t mfg_data[REPORT_SIZE];
	const struct bt_data adv_data[] = {
		BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
		//BT_DATA_BYTES(BT_DATA_UUID128_ALL, SMP_BT_SVC_UUID_VAL),
#ifdef CONFIG_BT_ZEPHYR_NUS
		//BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_SRV_VAL),
#endif
		BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_data, sizeof(mfg_data)),
		//BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
	};
	int ret;

	report_cpy(mfg_data, uptime);

	ret = bt_le_ext_adv_set_data(_adv, adv_data, ARRAY_SIZE(adv_data), NULL, 0);
	if (ret)
		LOG_ERR("bluetooth failed to set advertisement data : %d", ret);

	return ret;
}

static void bt_connected(struct bt_conn *conn, uint8_t err)
{
        if (err) {
                LOG_ERR("bt Connection failed, err 0x%02x %s", err, bt_hci_err_to_str(err));
		_bt_conn = NULL;
        } else {
                LOG_DBG("bt Connected");
		_bt_conn = conn;
        }
}

static void bt_disconnected(struct bt_conn *conn, uint8_t reason)
{
        LOG_DBG("bt Disconnected, reason 0x%02x %s", reason, bt_hci_err_to_str(reason));
        k_work_submit(&_bt_adv_work);
	_bt_conn = NULL;
}

static void bt_on_conn_recycled(void)
{
        LOG_DBG("bt Recycled");
	_bt_conn = NULL;
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
        .connected = bt_connected,
        .disconnected = bt_disconnected,
        .recycled = bt_on_conn_recycled,
};

#endif /* CONFIG_BT */

/*
 * Wifi functions
 */

#ifdef CONFIG_WIFI
static int
wifi_report(int64_t uptime, int reports)
{
	char sbuf[REPORT_SIZE];

	report_cpy(sbuf, uptime);
	return http_req(HTTP_POST, "/sensor/report", (const char *)&sbuf, REPORT_SIZE);
}

static void
wifi_report_cb(struct http_response *rsp, enum http_final_call final_data, void *user_data)
{
	LOG_DBG("wifi received response, status %s", rsp->http_status);
}

static void
wifi_event(struct net_mgmt_event_callback *cb, uint32_t mgmt_event, struct net_if *iface)
{
	switch (mgmt_event) {
	case NET_EVENT_WIFI_CONNECT_RESULT:
		LOG_DBG("wifi connected");
		_wifi_conn = true;
		break;
	case NET_EVENT_WIFI_DISCONNECT_RESULT:
		LOG_DBG("wifi disconnected");
		_wifi_conn = false;
		break;
	default:
		LOG_DBG("wifi unknown event: %d", mgmt_event);
		break;
	}
}

static int
http_req(int method, const char *url, const char *payload, int payload_len)
{
	struct sockaddr_in addr_in;
	struct sockaddr *addr = (struct sockaddr *)&addr_in;
	int sock = -1;
	const char *dst_addr = CONFIG_BKK_WIFI_SERVER_ADDR;
	int dst_port = CONFIG_BKK_WIFI_SERVER_PORT;
	struct http_request req;
	int ret;

	net_sin(addr)->sin_family = AF_INET;
	net_sin(addr)->sin_port = htons(dst_port);
	inet_pton(AF_INET, dst_addr, &net_sin(addr)->sin_addr);
	sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (sock < 0) {
		LOG_ERR("failed to create socket: %d", -errno);
	}
	ret = connect(sock, addr, sizeof(addr_in));
	if (ret < 0) {
		LOG_ERR("cannot connect to server %s port %d: %d", dst_addr, dst_port, -errno);
		close(sock);
		return -1;
	}

	memset(&req, 0, sizeof(req));
	req.method = method;
	req.url = url;
	req.host = dst_addr;
	req.protocol = "HTTP/1.1";
	req.payload = payload;
	req.payload_len = payload_len;
	req.response = wifi_report_cb;
	req.recv_buf = _wifi_rbuf;
	req.recv_buf_len = sizeof(_wifi_rbuf);

	ret = http_client_req(sock, &req, 3*MSEC_PER_SEC, NULL);
	close(sock);

	if (ret <= 0)
		return ret;
	return 0;
}
#endif /* CONFIG_WIFI */
