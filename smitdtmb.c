// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * SMIT/Ronghe DTMB USB stick driver.
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/workqueue.h>

#include <media/demux.h>
#include <media/dmxdev.h>
#include <media/dvb_demux.h>
#include <media/dvb_frontend.h>
#include <media/dvbdev.h>

#define SMITDTMB_DRV_NAME       "smitdtmb"
#define SMITDTMB_ADAPTER_NAME   "SMIT/Ronghe DTMB USB"
#define SMITDTMB_FREQ_MIN_HZ    115000000U
#define SMITDTMB_FREQ_MAX_HZ    858000000U

#define USB_VID_SMIT            0x29df
#define USB_PID_SMIT_DONGLE     0x0001
#define USB_VID_FILTER_EXTRA    0x2012
#define USB_PID_FILTER_EXTRA    0x0813

#define EP_CMD_OUT_ADDR         0x01
#define EP_CMD_IN_ADDR          0x82
#define EP_TS_IN_ADDR           0x84
#define EP_CMD_OUT_NUM          0x01
#define EP_CMD_IN_NUM           0x02
#define EP_TS_IN_NUM            0x04

#define CTRL_HOT_RESET_REQTYPE  0x40
#define CTRL_HOT_RESET_REQ      0xa0

#define TS_URB_COUNT            8
#define TS_URB_BUFSIZE          8192
#define TS_SYNC_BUFSIZE         (188 * 87)
#define TS_SYNC_TIMEOUT_MS      500
#define CMD_TIMEOUT_MS          5000
#define TRANSPORT_POLL_DELAY_MS 100
#define TRANSPORT_READ_TRIES    24
#define CI_STARTUP_TIMEOUT_MS   2000
#define APDU_RESPONSE_TIMEOUT_MS 15000
#define STATUS_POLL_INTERVAL_MS 2000
#define CI_SLOT                 0x00
#define CI_TCID                 0x01
#define CI_LAST_FRAGMENT        0x00
#define LPDU_TCID               CI_TCID
#define TPDU_CREATE_TC          0x82
#define TPDU_CTC_REPLY          0x83
#define TPDU_T_RCV              0x81
#define TPDU_T_SB               0x80
#define TPDU_DATA_LAST          0xa0
#define TPDU_POLL_ENCODED       0xa0
#define TPDU_STATUS_DATA_AVAIL  0x80
#define TPDU_STATUS_NO_DATA     0x00
#define SPDU_OPEN_SESSION_REQ   0x91
#define SPDU_OPEN_SESSION_RSP   0x92
#define SPDU_SESSION_NUMBER     0x90
#define SAS_APDU_TAG_CONNECT    0x9f9a00
#define SAS_APDU_TAG_DATA       0x9f9a07
#define RM_APDU_PROFILE_ENQ     0x9f8010
#define RM_APDU_PROFILE_REPLY   0x9f8011
#define RM_APDU_PROFILE_CHANGED 0x9f8012
#define RES_RESOURCE_MANAGER    0x00010041
#define RES_APP_INFO            0x00020041
#define RES_PRIVATE_SAS         0x00961001
#define AI_APDU_APP_INFO_ENQ    0x9f8020
#define AI_APDU_APP_INFO        0x9f8021

#define SAS_CMD_RESET           0x0001
#define SAS_CMD_TUNE            0x0003
#define SAS_CMD_STATUS          0x0005
#define SAS_RSP_RESET           0x00040002
#define SAS_RSP_TUNE            0x00040004
#define SAS_RSP_STATUS          0x0006

DVB_DEFINE_MOD_OPT_ADAPTER_NR(adapter_nr);

static bool do_hot_reset;
module_param(do_hot_reset, bool, 0644);
MODULE_PARM_DESC(do_hot_reset,
		 "send vendor USB hot-reset request 0x40/0xa0 before command reset");

static bool do_cmd_reset;
module_param(do_cmd_reset, bool, 0644);
MODULE_PARM_DESC(do_cmd_reset,
		 "send recovered REST APDU during frontend init");

static bool clear_cmd_halts = true;
module_param(clear_cmd_halts, bool, 0644);
MODULE_PARM_DESC(clear_cmd_halts,
		 "clear command bulk endpoint halts before CI buffer negotiation");

static bool init_on_probe;
module_param(init_on_probe, bool, 0644);
MODULE_PARM_DESC(init_on_probe,
		 "run CI buffer negotiation immediately after USB probe");

static bool set_interface_on_probe;
module_param(set_interface_on_probe, bool, 0644);
MODULE_PARM_DESC(set_interface_on_probe,
		 "force SET_INTERFACE(0) during probe before endpoint checks");

static bool ts_sync_reader;
module_param(ts_sync_reader, bool, 0644);
MODULE_PARM_DESC(ts_sync_reader,
		 "read TS endpoint with synchronous bulk reads instead of async URBs");

static bool poll_status = true;
module_param(poll_status, bool, 0644);
MODULE_PARM_DESC(poll_status,
		 "periodically poll tuner STATUS APDU from frontend read callbacks");

struct smitdtmb_dev {
	struct usb_device *udev;
	struct usb_interface *intf;
	struct mutex cmd_lock;
	spinlock_t feed_lock;

	struct dvb_adapter adapter;
	struct dvb_frontend fe;
	struct dvb_demux demux;
	struct dmxdev dmxdev;
	struct dmx_frontend hw_frontend;
	struct dmx_frontend mem_frontend;

	struct urb *urbs[TS_URB_COUNT];
	u8 *urb_bufs[TS_URB_COUNT];
	dma_addr_t urb_dma[TS_URB_COUNT];
	struct task_struct *ts_thread;
	struct delayed_work status_work;
	int feed_count;
	bool streaming;
	bool disconnected;
	bool buffer_ready;
	bool ci_ready;

	u32 frequency_hz;
	u32 symbol_rate;
	u32 modulation;
	u32 bandwidth_mhz;
	u16 ssnb;
	u16 rm_ssnb;
	u16 ai_ssnb;
	u16 sas_ssnb;
	u16 next_ssnb;
	u8 sas_seq;
	bool sas_connect_sent;
	bool sas_connected;
	bool sas_late_open_seen;
	u16 strength;
	u16 snr;
	bool locked;
	u32 ts_packets;
	u32 ts_errors;
	u32 ts_timeouts;
	u32 feed_logs;
};

static void put_be16(u8 *p, u16 v)
{
	p[0] = v >> 8;
	p[1] = v;
}

static void put_be32(u8 *p, u32 v)
{
	p[0] = v >> 24;
	p[1] = v >> 16;
	p[2] = v >> 8;
	p[3] = v;
}

static u16 get_be16(const u8 *p)
{
	return ((u16)p[0] << 8) | p[1];
}

static u32 get_be32(const u8 *p)
{
	return ((u32)p[0] << 24) | ((u32)p[1] << 16) | ((u32)p[2] << 8) | p[3];
}

static void put_le16(u8 *p, u16 v)
{
	p[0] = v;
	p[1] = v >> 8;
}

static void put_le32(u8 *p, u32 v)
{
	p[0] = v;
	p[1] = v >> 8;
	p[2] = v >> 16;
	p[3] = v >> 24;
}

static u16 get_le16(const u8 *p)
{
	return ((u16)p[1] << 8) | p[0];
}

static u32 get_le32(const u8 *p)
{
	return ((u32)p[3] << 24) | ((u32)p[2] << 16) | ((u32)p[1] << 8) | p[0];
}

static int smitdtmb_send_raw_locked(struct smitdtmb_dev *d, const u8 *buf,
				    int len, const char *what);
static int smitdtmb_start_streaming(struct smitdtmb_dev *d);
static void smitdtmb_status_work(struct work_struct *work);

static int smitdtmb_bulk_write(struct smitdtmb_dev *d, const void *buf, int len)
{
	u8 *tx;
	int actual = 0;
	int ret;

	tx = kmemdup(buf, len, GFP_KERNEL);
	if (!tx)
		return -ENOMEM;

	ret = usb_bulk_msg(d->udev, usb_sndbulkpipe(d->udev, EP_CMD_OUT_NUM),
			   tx, len, &actual, CMD_TIMEOUT_MS);
	kfree(tx);
	if (ret)
		return ret;
	if (actual != len)
		return -EIO;
	return 0;
}

static int smitdtmb_bulk_read_timeout(struct smitdtmb_dev *d, void *buf, int len,
				      int *actual, int timeout_ms)
{
	u8 *rx;
	int ret;

	rx = kmalloc(len, GFP_KERNEL);
	if (!rx)
		return -ENOMEM;

	ret = usb_bulk_msg(d->udev, usb_rcvbulkpipe(d->udev, EP_CMD_IN_NUM),
			   rx, len, actual, timeout_ms);
	if (!ret && actual && *actual > 0)
		memcpy(buf, rx, min(*actual, len));
	kfree(rx);
	return ret;
}

static int smitdtmb_check_complete_locked(struct smitdtmb_dev *d, int check_count)
{
	u8 msg[5] = { LPDU_TCID, CI_LAST_FRAGMENT, TPDU_DATA_LAST, 0x01, CI_TCID };
	u8 rx[1000];
	int actual = 0;
	int ret;
	int cnt = 0;

	while (true) {
		ret = smitdtmb_bulk_write(d, msg, sizeof(msg));
		if (ret)
			return ret;

		ret = smitdtmb_bulk_read_timeout(d, rx, sizeof(rx), &actual, 500);
		if (ret)
			return ret;
		if (actual < 6 || rx[0] != LPDU_TCID)
			return -EIO;
		if (rx[5] == TPDU_STATUS_DATA_AVAIL)
			return 0;
		if (rx[5])
			return -EIO;
		cnt++;
		if (check_count <= 0)
			return 0;
		if (cnt > check_count)
			return -ETIMEDOUT;
		msleep(cnt / 2 + 3);
	}
}

static int smitdtmb_set_frame_complete_locked(struct smitdtmb_dev *d,
					     const u8 *frame, int frame_len,
					     int need_complete_flag,
					     const char *what)
{
	u8 rx[1000];
	int actual = 0;
	int ret;

	ret = smitdtmb_send_raw_locked(d, frame, frame_len, what);
	if (ret)
		return ret;

	ret = smitdtmb_bulk_read_timeout(d, rx, sizeof(rx), &actual, 500);
	if (ret == -ETIMEDOUT || ret == -EAGAIN) {
		dev_dbg(&d->intf->dev,
			"%s completion read timed out; continuing like iCast\n",
			what);
		return 0;
	}
	if (ret)
		return ret;
	dev_dbg(&d->intf->dev, "%s completion rx len=%d data=%*ph\n",
		what, actual, min(actual, 64), rx);

	if (actual < 4 || rx[0] != LPDU_TCID)
		return -EIO;
	if (actual >= 6 && rx[2] == TPDU_T_SB && rx[5] == TPDU_STATUS_DATA_AVAIL)
		return 0;
	if (actual > rx[3] + 7 &&
	    rx[4 + rx[3]] == TPDU_STATUS_DATA_AVAIL &&
	    rx[7 + rx[3]] == TPDU_STATUS_DATA_AVAIL)
		return 0;
	if (need_complete_flag) {
		msleep(100);
		ret = smitdtmb_check_complete_locked(d, need_complete_flag);
		if (ret == -ETIMEDOUT) {
			dev_dbg(&d->intf->dev,
				"%s completion check timed out; continuing like iCast\n",
				what);
			return 0;
		}
		return ret;
	}

	return 0;
}

static int smitdtmb_tuner_stat_locked(struct smitdtmb_dev *d, u8 *out,
				      int out_len, int check_count,
				      const char *what)
{
	u8 msg[5] = { LPDU_TCID, CI_LAST_FRAGMENT, TPDU_T_RCV, 0x01, CI_TCID };
	u8 rx[900];
	int actual = 0;
	int data_len;
	int copy_len;
	int available;
	int ret;

	ret = smitdtmb_bulk_write(d, msg, sizeof(msg));
	if (ret)
		return ret;

	ret = smitdtmb_bulk_read_timeout(d, rx, sizeof(rx), &actual, 400);
	if (ret)
		return ret;
	dev_dbg(&d->intf->dev, "%s stat rx len=%d data=%*ph\n",
		what, actual, min(actual, 64), rx);

	if (actual < 4 || rx[0] != LPDU_TCID)
		return -EIO;

	data_len = rx[3];
	available = actual > 4 ? actual - 4 : 0;
	copy_len = min(data_len, available);
	copy_len = min(copy_len, out_len);
	memcpy(out, rx + 4, copy_len);

	if ((data_len + 7 >= actual ||
	     rx[4 + data_len] != TPDU_STATUS_DATA_AVAIL ||
	     rx[7 + data_len] != TPDU_STATUS_DATA_AVAIL) && check_count > 0)
		smitdtmb_check_complete_locked(d, check_count);

	return data_len;
}

static void smitdtmb_clear_cmd_halts(struct smitdtmb_dev *d)
{
	int ret;

	if (!clear_cmd_halts)
		return;

	ret = usb_clear_halt(d->udev, usb_sndbulkpipe(d->udev, EP_CMD_OUT_NUM));
	if (ret)
		dev_info(&d->intf->dev, "clear cmd-out halt failed: %d\n", ret);

	ret = usb_clear_halt(d->udev, usb_rcvbulkpipe(d->udev, EP_CMD_IN_NUM));
	if (ret)
		dev_info(&d->intf->dev, "clear cmd-in halt failed: %d\n", ret);
}

static int smitdtmb_asn_len(u8 *p, int len)
{
	if (len <= 0x7f) {
		p[0] = len;
		return 1;
	}

	p[0] = 0x81;
	p[1] = len;
	return 2;
}

static int smitdtmb_asn_read_len(const u8 *p, int len, int *value, int *used)
{
	int n, i, v = 0;

	if (len <= 0)
		return -EINVAL;

	if (!(p[0] & 0x80)) {
		*value = p[0];
		*used = 1;
		return 0;
	}

	n = p[0] & 0x7f;
	if (n < 1 || n > 2 || len < n + 1)
		return -EINVAL;

	for (i = 0; i < n; i++)
		v = (v << 8) | p[1 + i];

	*value = v;
	*used = n + 1;
	return 0;
}

static int smitdtmb_send_raw_locked(struct smitdtmb_dev *d, const u8 *buf,
				    int len, const char *what);
static int smitdtmb_wrap_spdu(struct smitdtmb_dev *d, const u8 *spdu, int spdu_len,
			      u8 *out, int out_len);
static int smitdtmb_send_sas_connect_locked(struct smitdtmb_dev *d);

static int smitdtmb_reply_open_session_locked(struct smitdtmb_dev *d,
					     u32 resource_id, bool prefer_existing)
{
	u8 open_rsp[9];
	u8 frame[96];
	u16 ssnb = 0;
	int frame_len;
	int ret;

	if (prefer_existing) {
		if (resource_id == RES_RESOURCE_MANAGER)
			ssnb = d->rm_ssnb;
		else if (resource_id == RES_APP_INFO)
			ssnb = d->ai_ssnb;
		else if (resource_id == RES_PRIVATE_SAS)
			ssnb = d->sas_ssnb;
	}

	if (!ssnb) {
		if (!d->next_ssnb)
			d->next_ssnb = 1;
		ssnb = d->next_ssnb++;
		if (resource_id == RES_RESOURCE_MANAGER && !d->rm_ssnb)
			d->rm_ssnb = ssnb;
		else if (resource_id == RES_APP_INFO && !d->ai_ssnb)
			d->ai_ssnb = ssnb;
		else if (resource_id == RES_PRIVATE_SAS &&
			 (!d->sas_ssnb || !d->sas_connect_sent))
			d->sas_ssnb = ssnb;
	}

	open_rsp[0] = SPDU_OPEN_SESSION_RSP;
	open_rsp[1] = 7;
	open_rsp[2] = 0;
	put_be32(open_rsp + 3, resource_id);
	put_be16(open_rsp + 7, ssnb);

	frame_len = smitdtmb_wrap_spdu(d, open_rsp, sizeof(open_rsp),
				       frame, sizeof(frame));
	if (frame_len < 0)
		return frame_len;

	ret = smitdtmb_send_raw_locked(d, frame, frame_len,
				       "open_session_response");
	if (ret)
		return ret;

	dev_dbg(&d->intf->dev,
		"CI session opened resource=0x%08x ssnb=%u%s\n",
		resource_id, ssnb, prefer_existing ? " existing-ok" : "");
	if (resource_id == RES_PRIVATE_SAS && d->sas_ssnb == ssnb)
		d->ssnb = d->sas_ssnb;
	else if (resource_id == RES_PRIVATE_SAS && d->sas_connect_sent)
		d->sas_late_open_seen = true;
	return ssnb;
}

static int smitdtmb_negotiate_buffer(struct smitdtmb_dev *d)
{
	static const u8 cmd_template[4] = { 0xfe, 0x00, 0x10, 0x00 };
	u8 *cmd;
	u8 *rsp;
	int actual = 0;
	int ret;

	if (d->buffer_ready)
		return 0;

	cmd = kmemdup(cmd_template, sizeof(cmd_template), GFP_KERNEL);
	if (!cmd)
		return -ENOMEM;

	rsp = kmalloc(64, GFP_KERNEL);
	if (!rsp) {
		kfree(cmd);
		return -ENOMEM;
	}

	smitdtmb_clear_cmd_halts(d);

	ret = smitdtmb_bulk_write(d, cmd, sizeof(cmd_template));
	if (ret) {
		dev_info(&d->intf->dev, "buffer negotiation write failed: %d\n", ret);
		goto out;
	}

	ret = smitdtmb_bulk_read_timeout(d, rsp, 64, &actual, 1000);
	if (ret == -ETIMEDOUT) {
		dev_dbg(&d->intf->dev,
			"buffer negotiation response timed out; assuming physical layer is already ready\n");
		d->buffer_ready = true;
		ret = 0;
		goto out;
	}
	if (ret) {
		dev_info(&d->intf->dev, "buffer negotiation read failed: %d\n", ret);
		goto out;
	}

	dev_dbg(&d->intf->dev, "buffer negotiation response len=%d data=%*ph\n",
		actual, actual, rsp);
	if (actual >= 4 && rsp[0] == 0xff) {
		d->buffer_ready = true;
		ret = 0;
	} else if (actual >= 5 && rsp[0] == LPDU_TCID) {
		dev_dbg(&d->intf->dev,
			"buffer negotiation saw TPDU; assuming buffer is already ready\n");
		d->buffer_ready = true;
		ret = 0;
	} else {
		ret = -EIO;
	}

out:
	kfree(rsp);
	kfree(cmd);
	return ret;
}

static int smitdtmb_wrap_spdu(struct smitdtmb_dev *d, const u8 *spdu, int spdu_len,
			      u8 *out, int out_len)
{
	int pos = 0;

	(void)d;

	if (spdu_len + 8 > out_len)
		return -EINVAL;

	out[pos++] = LPDU_TCID;
	out[pos++] = CI_LAST_FRAGMENT;
	out[pos++] = TPDU_DATA_LAST;
	pos += smitdtmb_asn_len(out + pos, spdu_len + 1);
	out[pos++] = CI_TCID;
	memcpy(out + pos, spdu, spdu_len);
	pos += spdu_len;

	return pos;
}

static int smitdtmb_wrap_session_apdu(struct smitdtmb_dev *d, const u8 *apdu,
				      int apdu_len, u8 *out, int out_len)
{
	u8 spdu[72];
	int spdu_len;

	if (apdu_len + 4 > sizeof(spdu))
		return -EINVAL;

	spdu[0] = SPDU_SESSION_NUMBER;
	spdu[1] = 0x02;
	put_be16(spdu + 2, d->ssnb ? d->ssnb : 1);
	memcpy(spdu + 4, apdu, apdu_len);
	spdu_len = apdu_len + 4;

	return smitdtmb_wrap_spdu(d, spdu, spdu_len, out, out_len);
}

static int smitdtmb_wrap_sas_apdu(struct smitdtmb_dev *d, const u8 *apdu,
				  int apdu_len, u8 *out, int out_len)
{
	u8 sas[64];
	int sas_len = 0;
	int len_len;

	if (apdu_len + 8 > sizeof(sas))
		return -EINVAL;

	sas[sas_len++] = 0x9f;
	sas[sas_len++] = 0x9a;
	sas[sas_len++] = 0x07;
	len_len = smitdtmb_asn_len(sas + sas_len, apdu_len + 3);
	sas_len += len_len;
	sas[sas_len++] = d->sas_seq++;
	put_be16(sas + sas_len, apdu_len);
	sas_len += 2;
	memcpy(sas + sas_len, apdu, apdu_len);
	sas_len += apdu_len;

	return smitdtmb_wrap_session_apdu(d, sas, sas_len, out, out_len);
}

static int smitdtmb_send_raw_locked(struct smitdtmb_dev *d, const u8 *buf,
				    int len, const char *what)
{
	u8 *tx;
	int ret;

	tx = kmemdup(buf, len, GFP_KERNEL);
	if (!tx)
		return -ENOMEM;

	dev_dbg(&d->intf->dev, "%s tx len=%d data=%*ph\n", what, len, len, tx);
	ret = smitdtmb_bulk_write(d, tx, len);
	if (ret)
		dev_info(&d->intf->dev, "%s write failed: %d\n", what, ret);
	kfree(tx);
	return ret;
}

static int smitdtmb_send_transport_ack_locked(struct smitdtmb_dev *d, u8 status)
{
	u8 msg[] = {
		LPDU_TCID, CI_LAST_FRAGMENT,
		status == TPDU_STATUS_DATA_AVAIL ? TPDU_T_RCV : TPDU_POLL_ENCODED,
		0x01, CI_TCID
	};

	return smitdtmb_send_raw_locked(d, msg, sizeof(msg),
					status == TPDU_STATUS_DATA_AVAIL ?
					"t_rcv" : "t_poll");
}

static int smitdtmb_send_status_locked(struct smitdtmb_dev *d, u8 status)
{
	u8 msg[] = {
		LPDU_TCID, CI_LAST_FRAGMENT,
		TPDU_T_SB, 0x02, CI_TCID, status
	};

	return smitdtmb_send_raw_locked(d, msg, sizeof(msg),
					status == TPDU_STATUS_DATA_AVAIL ?
					"t_sb(data)" : "t_sb(no data)");
}

static int smitdtmb_decode_tpdu(const u8 *lpdu, int lpdu_len, u8 *tag,
				const u8 **payload, int *payload_len)
{
	const u8 *tpdu;
	int len_value, len_used;

	if (lpdu_len < 5 || lpdu[0] != LPDU_TCID)
		return -EINVAL;

	tpdu = lpdu + 2;
	*tag = tpdu[0];

	if (*tag >= 0x80 && *tag <= 0x88) {
		if (lpdu_len < 5 || tpdu[1] < 1 || tpdu[2] != CI_TCID)
			return -EINVAL;
		*payload = tpdu + 3;
		*payload_len = tpdu[1] - 1;
		return 0;
	}

	if (*tag != TPDU_DATA_LAST && *tag != 0xa1)
		return -EINVAL;

	if (smitdtmb_asn_read_len(tpdu + 1, lpdu_len - 3, &len_value, &len_used))
		return -EINVAL;
	if (len_value < 1 || lpdu_len < 2 + 1 + len_used + len_value)
		return -EINVAL;
	if (tpdu[1 + len_used] != CI_TCID)
		return -EINVAL;

	*payload = tpdu + 1 + len_used + 1;
	*payload_len = len_value - 1;
	return 0;
}

static int smitdtmb_read_data_tpdu_locked(struct smitdtmb_dev *d, u8 *rx,
					  int rx_len, int *actual,
					  const char *what)
{
	const u8 *payload;
	int payload_len;
	int ret, i;
	u8 tag;

	if (actual)
		*actual = 0;

	for (i = 0; i < TRANSPORT_READ_TRIES; i++) {
		ret = smitdtmb_bulk_read_timeout(d, rx, rx_len, actual, 1200);
		if (ret == -ETIMEDOUT || ret == -EAGAIN) {
			dev_dbg(&d->intf->dev, "%s response timed out\n", what);
			if (actual)
				*actual = 0;
			return 0;
		}
		if (ret)
			return ret;

		dev_dbg(&d->intf->dev, "%s rx len=%d data=%*ph\n",
			what, *actual, min(*actual, 64), rx);

		ret = smitdtmb_decode_tpdu(rx, *actual, &tag, &payload,
					   &payload_len);
		if (ret)
			return 0;

		if (tag == TPDU_DATA_LAST && payload_len >= 6 &&
		    payload[0] == SPDU_OPEN_SESSION_REQ && payload[1] == 4) {
			u32 resource_id = get_be32(payload + 2);
			int ssnb;

			ssnb = smitdtmb_reply_open_session_locked(d, resource_id,
								  false);
				if (ssnb < 0)
					return ssnb;
				if (resource_id == RES_PRIVATE_SAS &&
				    !d->sas_connect_sent) {
					d->sas_connect_sent = true;
					ret = smitdtmb_send_sas_connect_locked(d);
					if (ret)
						return ret;
					d->sas_connected = true;
				} else if (resource_id == RES_PRIVATE_SAS &&
					   d->sas_connect_sent) {
					if (actual)
						*actual = 0;
					return 0;
				}
				continue;
			}

		if (tag == TPDU_DATA_LAST && payload_len > 0)
			return 0;

		if (tag == TPDU_DATA_LAST && payload_len == 0) {
			ret = smitdtmb_send_status_locked(d, TPDU_STATUS_NO_DATA);
			if (ret)
				return ret;

			if (*actual >= 9 && rx[5] == TPDU_T_SB && rx[6] == 0x02 &&
			    rx[7] == CI_TCID) {
				if (rx[8] != TPDU_STATUS_DATA_AVAIL)
					msleep(TRANSPORT_POLL_DELAY_MS);
				ret = smitdtmb_send_transport_ack_locked(d, rx[8]);
				if (ret)
					return ret;
			}
			continue;
		}

		if (tag == TPDU_T_SB && payload_len >= 1) {
			if (payload[0] != TPDU_STATUS_DATA_AVAIL)
				msleep(TRANSPORT_POLL_DELAY_MS);
			ret = smitdtmb_send_transport_ack_locked(d, payload[0]);
			if (ret)
				return ret;
			continue;
		}

		if (tag == TPDU_CTC_REPLY &&
		    *actual >= 9 && rx[5] == TPDU_T_SB && rx[6] == 0x02 &&
		    rx[7] == CI_TCID) {
			if (rx[8] != TPDU_STATUS_DATA_AVAIL)
				msleep(TRANSPORT_POLL_DELAY_MS);
			ret = smitdtmb_send_transport_ack_locked(d, rx[8]);
			if (ret)
				return ret;
			continue;
		}

		return 0;
	}

	return -ETIMEDOUT;
}

static int smitdtmb_drain_pending_locked(struct smitdtmb_dev *d, u8 *rx,
					 int rx_len)
{
	int actual = 0;
	int ret, i;

	for (i = 0; i < 8; i++) {
		ret = smitdtmb_read_data_tpdu_locked(d, rx, rx_len, &actual,
						     "CI drain");
		if (ret == -ETIMEDOUT)
			return 0;
		if (ret)
			return ret;
		if (!actual)
			return 0;
	}

	return 0;
}

static int smitdtmb_send_sas_connect_locked(struct smitdtmb_dev *d)
{
	static const u8 sas_app_id[8] = { 'S', 'M', 'i', 'T', 'Z', 'B', 'J', 'L' };
	u8 apdu[16];
	u8 frame[96];
	int apdu_len = 0;
	int frame_len;

	apdu[apdu_len++] = 0x9f;
	apdu[apdu_len++] = 0x9a;
	apdu[apdu_len++] = 0x00;
	apdu_len += smitdtmb_asn_len(apdu + apdu_len, sizeof(sas_app_id));
	memcpy(apdu + apdu_len, sas_app_id, sizeof(sas_app_id));
	apdu_len += sizeof(sas_app_id);

	frame_len = smitdtmb_wrap_session_apdu(d, apdu, apdu_len, frame, sizeof(frame));
	if (frame_len < 0)
		return frame_len;

	return smitdtmb_send_raw_locked(d, frame, frame_len, "SAS connect");
}

static int smitdtmb_send_rm_profile_enquiry_locked(struct smitdtmb_dev *d)
{
	u8 apdu[4];
	u8 frame[32];
	int frame_len;

	apdu[0] = 0x9f;
	apdu[1] = 0x80;
	apdu[2] = 0x10;
	apdu[3] = 0x00;

	frame_len = smitdtmb_wrap_session_apdu(d, apdu, sizeof(apdu),
					       frame, sizeof(frame));
	if (frame_len < 0)
		return frame_len;

	return smitdtmb_send_raw_locked(d, frame, frame_len,
					"RM profile_enquiry");
}

static int smitdtmb_send_rm_profile_changed_locked(struct smitdtmb_dev *d)
{
	u8 apdu[4];
	u8 frame[32];
	int frame_len;

	apdu[0] = 0x9f;
	apdu[1] = 0x80;
	apdu[2] = 0x12;
	apdu[3] = 0x00;

	frame_len = smitdtmb_wrap_session_apdu(d, apdu, sizeof(apdu),
					       frame, sizeof(frame));
	if (frame_len < 0)
		return frame_len;

	return smitdtmb_send_raw_locked(d, frame, frame_len,
					"RM profile_changed");
}

static int smitdtmb_send_rm_profile_reply_locked(struct smitdtmb_dev *d)
{
	static const u32 resources[] = {
		0x00010041, 0x00020041, 0x00030041, 0x00240041,
		0x00400041, 0x00961001, 0x00200041,
	};
	u8 apdu[3 + 2 + sizeof(resources)];
	u8 frame[96];
	int apdu_len = 0;
	int frame_len;
	int i;

	apdu[apdu_len++] = 0x9f;
	apdu[apdu_len++] = 0x80;
	apdu[apdu_len++] = 0x11;
	apdu_len += smitdtmb_asn_len(apdu + apdu_len, sizeof(resources));
	for (i = 0; i < ARRAY_SIZE(resources); i++) {
		put_be32(apdu + apdu_len, resources[i]);
		apdu_len += 4;
	}

	frame_len = smitdtmb_wrap_session_apdu(d, apdu, apdu_len,
					       frame, sizeof(frame));
	if (frame_len < 0)
		return frame_len;

	return smitdtmb_send_raw_locked(d, frame, frame_len,
					"RM profile_reply(host)");
}

static int smitdtmb_send_ai_enquiry_locked(struct smitdtmb_dev *d)
{
	u8 apdu[4];
	u8 frame[32];
	int frame_len;

	apdu[0] = 0x9f;
	apdu[1] = 0x80;
	apdu[2] = 0x20;
	apdu[3] = 0x00;

	frame_len = smitdtmb_wrap_session_apdu(d, apdu, sizeof(apdu),
					       frame, sizeof(frame));
	if (frame_len < 0)
		return frame_len;

	return smitdtmb_send_raw_locked(d, frame, frame_len,
					"AI application_info_enq");
}

static int smitdtmb_get_session_apdu_tag(const u8 *rx, int rx_len, u32 *tag)
{
	const u8 *payload;
	const u8 *apdu;
	int payload_len;
	u8 tpdu_tag;

	if (smitdtmb_decode_tpdu(rx, rx_len, &tpdu_tag, &payload, &payload_len))
		return -EINVAL;
	if (tpdu_tag != TPDU_DATA_LAST || payload_len < 8)
		return -EINVAL;
	if (payload[0] != SPDU_SESSION_NUMBER || payload[1] != 2)
		return -EINVAL;

	apdu = payload + 4;
	payload_len -= 4;
	if (payload_len < 3)
		return -EINVAL;

	*tag = ((u32)apdu[0] << 16) | ((u32)apdu[1] << 8) | apdu[2];
	return 0;
}

static void smitdtmb_log_rm_profile(struct smitdtmb_dev *d,
				    const u8 *rx, int rx_len)
{
	const u8 *payload;
	const u8 *apdu;
	int payload_len;
	int len_value;
	int len_used;
	u32 tag;
	int i;
	u8 tpdu_tag;

	if (smitdtmb_decode_tpdu(rx, rx_len, &tpdu_tag, &payload, &payload_len))
		return;
	if (tpdu_tag != TPDU_DATA_LAST || payload_len < 8)
		return;
	if (payload[0] != SPDU_SESSION_NUMBER || payload[1] != 2)
		return;

	apdu = payload + 4;
	payload_len -= 4;
	if (payload_len < 4)
		return;

	tag = ((u32)apdu[0] << 16) | ((u32)apdu[1] << 8) | apdu[2];
	if (smitdtmb_asn_read_len(apdu + 3, payload_len - 3,
				  &len_value, &len_used))
		return;

	apdu += 3 + len_used;
	payload_len -= 3 + len_used;
	if (len_value > payload_len)
		return;

	dev_dbg(&d->intf->dev, "RM APDU tag=0x%06x len=%d\n",
		tag, len_value);
	if (tag != RM_APDU_PROFILE_REPLY)
		return;

	for (i = 0; i + 3 < len_value; i += 4)
		dev_dbg(&d->intf->dev, "RM profile resource[%d]=0x%08x\n",
			i / 4, get_be32(apdu + i));
}

static int smitdtmb_ci_startup_locked(struct smitdtmb_dev *d)
{
	u8 create_tc[] = { LPDU_TCID, CI_LAST_FRAGMENT, TPDU_CREATE_TC, 0x01, CI_TCID };
	u8 t_rcv[] = { LPDU_TCID, CI_LAST_FRAGMENT, TPDU_T_RCV, 0x01, CI_TCID };
	u8 t_poll[] = { LPDU_TCID, CI_LAST_FRAGMENT, TPDU_POLL_ENCODED, 0x01, CI_TCID };
	u8 open_rsp[9];
	u8 frame[96];
	u8 *rx;
	const u8 *payload;
	u32 resource_id;
	u16 ssnb;
	u32 apdu_tag;
	unsigned long deadline;
	int payload_len;
	int frame_len;
	int actual = 0;
	int ret, i;
	u8 tag;

	if (d->ci_ready)
		return 0;
	if (!d->next_ssnb)
		d->next_ssnb = 1;

	ret = smitdtmb_negotiate_buffer(d);
	if (ret)
		return ret;

	ret = smitdtmb_send_raw_locked(d, create_tc, sizeof(create_tc), "create_t_c");
	if (ret)
		return ret;

	rx = kmalloc(256, GFP_KERNEL);
	if (!rx)
		return -ENOMEM;

	ret = smitdtmb_send_raw_locked(d, t_rcv, sizeof(t_rcv), "t_rcv(startup)");
	if (ret)
		goto out_free_rx;

	deadline = jiffies + msecs_to_jiffies(CI_STARTUP_TIMEOUT_MS);
	for (i = 0; time_before(jiffies, deadline); i++) {
		ret = smitdtmb_bulk_read_timeout(d, rx, 256, &actual, 1200);
		if (ret == -ETIMEDOUT || ret == -EAGAIN) {
			ret = smitdtmb_send_raw_locked(d, t_rcv, sizeof(t_rcv),
						       "t_rcv(startup retry)");
			if (ret)
				goto out_free_rx;
			msleep(TRANSPORT_POLL_DELAY_MS);
			continue;
		}
		if (ret) {
			dev_info(&d->intf->dev, "CI startup read failed at step %d: %d\n",
				 i, ret);
			goto out_free_rx;
		}

		dev_dbg(&d->intf->dev, "CI startup rx len=%d data=%*ph\n",
			actual, min(actual, 64), rx);

		ret = smitdtmb_decode_tpdu(rx, actual, &tag, &payload, &payload_len);
		if (ret)
			continue;

		if (tag == TPDU_CTC_REPLY) {
			if (actual >= 9 && rx[5] == TPDU_T_SB && rx[6] == 0x02 &&
			    rx[7] == CI_TCID) {
				if (rx[8] != TPDU_STATUS_DATA_AVAIL)
					msleep(TRANSPORT_POLL_DELAY_MS);
				ret = smitdtmb_send_raw_locked(d,
					rx[8] == TPDU_STATUS_DATA_AVAIL ? t_rcv : t_poll,
					sizeof(t_rcv),
					rx[8] == TPDU_STATUS_DATA_AVAIL ? "t_rcv" : "t_poll");
				if (ret)
					goto out_free_rx;
			}
			continue;
		}

		if (tag == TPDU_T_SB && payload_len >= 1) {
			if (payload[0] != TPDU_STATUS_DATA_AVAIL)
				msleep(TRANSPORT_POLL_DELAY_MS);
			ret = smitdtmb_send_raw_locked(d,
				payload[0] == TPDU_STATUS_DATA_AVAIL ? t_rcv : t_poll,
				sizeof(t_rcv),
				payload[0] == TPDU_STATUS_DATA_AVAIL ? "t_rcv" : "t_poll");
			if (ret)
				goto out_free_rx;
			continue;
		}

		if (tag != TPDU_DATA_LAST || payload_len < 2)
			continue;

		if (payload[0] != SPDU_OPEN_SESSION_REQ || payload[1] != 4 ||
		    payload_len < 6)
			continue;

		resource_id = get_be32(payload + 2);
		if (resource_id == RES_RESOURCE_MANAGER && !d->rm_ssnb)
			ssnb = d->rm_ssnb = d->next_ssnb++;
		else if (resource_id == RES_APP_INFO && !d->ai_ssnb)
			ssnb = d->ai_ssnb = d->next_ssnb++;
		else if (resource_id == RES_PRIVATE_SAS && !d->sas_ssnb)
			ssnb = d->sas_ssnb = d->next_ssnb++;
		else
			ssnb = d->next_ssnb++;
		d->ssnb = ssnb;

		open_rsp[0] = SPDU_OPEN_SESSION_RSP;
		open_rsp[1] = 7;
		open_rsp[2] = 0;
		put_be32(open_rsp + 3, resource_id);
		put_be16(open_rsp + 7, ssnb);

		frame_len = smitdtmb_wrap_spdu(d, open_rsp, sizeof(open_rsp),
					       frame, sizeof(frame));
		if (frame_len < 0) {
			ret = frame_len;
			goto out_free_rx;
		}

		ret = smitdtmb_send_raw_locked(d, frame, frame_len,
					       "open_session_response");
		if (ret)
			goto out_free_rx;

		if (resource_id != RES_RESOURCE_MANAGER) {
			dev_dbg(&d->intf->dev,
				"CI session opened resource=0x%08x ssnb=%u\n",
				resource_id, ssnb);
			if (resource_id == RES_APP_INFO) {
				d->ssnb = d->ai_ssnb;
				ret = smitdtmb_send_ai_enquiry_locked(d);
				if (ret)
					goto out_free_rx;
				ret = smitdtmb_read_data_tpdu_locked(d, rx, 256,
								     &actual,
								     "AI application_info");
				if (ret)
					goto out_free_rx;
				if (actual &&
				    !smitdtmb_get_session_apdu_tag(rx, actual,
								   &apdu_tag))
					dev_dbg(&d->intf->dev,
						"AI APDU tag=0x%06x\n",
						apdu_tag);
				continue;
			}

			if (resource_id == RES_PRIVATE_SAS) {
				d->ssnb = d->sas_ssnb;
				d->sas_connect_sent = true;
				ret = smitdtmb_send_sas_connect_locked(d);
				if (ret)
					goto out_free_rx;
				d->sas_connected = true;
				ret = smitdtmb_read_data_tpdu_locked(d, rx, 256,
								     &actual,
								     "SAS connect");
				if (ret)
					goto out_free_rx;
				deadline = jiffies + msecs_to_jiffies(3000);
				while (!d->sas_late_open_seen &&
				       time_before(jiffies, deadline)) {
					ret = smitdtmb_read_data_tpdu_locked(d, rx, 256,
									     &actual,
									     "SAS late_open");
					if (ret == -ETIMEDOUT && d->sas_late_open_seen)
						break;
					if (ret)
						goto out_free_rx;
					if (!actual)
						msleep(TRANSPORT_POLL_DELAY_MS);
				}
				if (d->sas_late_open_seen)
					dev_dbg(&d->intf->dev,
						"CI/SAS late private open completed\n");
				d->ci_ready = true;
				dev_dbg(&d->intf->dev,
					"CI/SAS startup completed ssnb=%u\n",
					d->sas_ssnb);
				ret = 0;
				goto out_free_rx;
			}
			continue;
		}

		ret = smitdtmb_send_rm_profile_enquiry_locked(d);
		if (ret)
			goto out_free_rx;

		ret = smitdtmb_read_data_tpdu_locked(d, rx, 256, &actual,
						     "RM profile_reply");
		if (ret)
			goto out_free_rx;
		if (!actual) {
			ret = -ETIMEDOUT;
			goto out_free_rx;
		}
		smitdtmb_log_rm_profile(d, rx, actual);

		ret = smitdtmb_send_rm_profile_changed_locked(d);
		if (ret)
			goto out_free_rx;

		ret = smitdtmb_read_data_tpdu_locked(d, rx, 256, &actual,
						     "RM module_profile_enquiry");
		if (ret)
			goto out_free_rx;
		if (actual &&
		    !smitdtmb_get_session_apdu_tag(rx, actual, &apdu_tag) &&
		    apdu_tag == RM_APDU_PROFILE_ENQ) {
			d->ssnb = d->rm_ssnb;
			ret = smitdtmb_send_rm_profile_reply_locked(d);
			if (ret)
				goto out_free_rx;
		}

		dev_dbg(&d->intf->dev,
			"CI/RM startup completed resource=0x%08x ssnb=%u\n",
			resource_id, d->rm_ssnb);
		continue;
	}

	if (!d->sas_ssnb) {
		d->sas_ssnb = 3;
		if (d->next_ssnb <= d->sas_ssnb)
			d->next_ssnb = d->sas_ssnb + 1;
	}
	d->ssnb = d->sas_ssnb;
	d->ci_ready = true;
	ret = 0;
	dev_dbg(&d->intf->dev,
		"CI/SAS startup timed out after %d ms; using existing SAS session %u\n",
		CI_STARTUP_TIMEOUT_MS, d->sas_ssnb);

out_free_rx:
	kfree(rx);
	return ret;
}

static int smitdtmb_extract_sas_payload(const u8 *rx, int rx_len,
					u8 *rsp, int rsp_len, int *actual)
{
	const u8 *payload;
	const u8 *apdu;
	int payload_len;
	int len_value;
	int len_used;
	int copy_len;
	u8 tag;

	if (actual)
		*actual = 0;

	if (smitdtmb_decode_tpdu(rx, rx_len, &tag, &payload, &payload_len))
		return -EINVAL;
	if (tag != TPDU_DATA_LAST || payload_len < 8)
		return -EINVAL;
	if (payload[0] != SPDU_SESSION_NUMBER || payload[1] != 2)
		return -EINVAL;

	apdu = payload + 4;
	payload_len -= 4;
	if (payload_len < 7 || apdu[0] != 0x9f || apdu[1] != 0x9a)
		return -EINVAL;

	if (apdu[2] != 0x07) {
		copy_len = min(payload_len, rsp_len);
		memcpy(rsp, apdu, copy_len);
		if (actual)
			*actual = copy_len;
		return 0;
	}

	if (smitdtmb_asn_read_len(apdu + 3, payload_len - 3, &len_value, &len_used))
		return -EINVAL;
	apdu += 3 + len_used;
	payload_len -= 3 + len_used;
	if (len_value > payload_len || len_value < 3)
		return -EINVAL;

	apdu += 3;
	payload_len = len_value - 3;
	copy_len = min(payload_len, rsp_len);
	memcpy(rsp, apdu, copy_len);
	if (actual)
		*actual = copy_len;

	return 0;
}

static bool smitdtmb_sas_payload_matches(const u8 *rsp, int actual,
					 u32 expected_id, bool expected_u16)
{
	if (!expected_id)
		return true;
	if (expected_u16)
		return actual >= 2 && get_be16(rsp) == expected_id;
	return actual >= 4 && get_be32(rsp) == expected_id;
}

static int smitdtmb_send_apdu_expect(struct smitdtmb_dev *d,
				     const u8 *cmd, int cmd_len,
				     u8 *rsp, int rsp_len, int *actual,
				     u32 expected_id, bool expected_u16)
{
	u8 *rx = NULL;
	u8 frame[96];
	int rx_len = 0;
	int frame_len = 0;
	unsigned long deadline;
	int ret;

	if (rsp && rsp_len) {
		rx_len = max(rsp_len + 64, 256);
		rx = kmalloc(rx_len, GFP_KERNEL);
		if (!rx)
			return -ENOMEM;
	}

	mutex_lock(&d->cmd_lock);
	ret = smitdtmb_ci_startup_locked(d);
	if (!ret && rx)
		ret = smitdtmb_drain_pending_locked(d, rx, rx_len);
	if (!ret) {
		frame_len = smitdtmb_wrap_sas_apdu(d, cmd, cmd_len,
						   frame, sizeof(frame));
		if (frame_len < 0)
			ret = frame_len;
		else
			ret = smitdtmb_send_raw_locked(d, frame, frame_len,
						       "APDU frame");
	}
	if (ret)
		dev_info(&d->intf->dev, "APDU frame write failed len=%d ret=%d\n",
			 frame_len, ret);

	if (!ret && rx) {
		bool matched = false;

		if (actual)
			*actual = 0;

		deadline = jiffies + msecs_to_jiffies(APDU_RESPONSE_TIMEOUT_MS);
		while (time_before(jiffies, deadline)) {
			int rx_actual = 0;
			int sas_actual = 0;

			ret = smitdtmb_read_data_tpdu_locked(d, rx, rx_len,
							     &rx_actual,
							     "APDU response");
			if (ret == -ETIMEDOUT || ret == -EAGAIN) {
				ret = 0;
				msleep(TRANSPORT_POLL_DELAY_MS);
				continue;
			}
			if (ret)
				break;
			if (!rx_actual) {
				msleep(TRANSPORT_POLL_DELAY_MS);
				continue;
			}

			ret = smitdtmb_extract_sas_payload(rx, rx_actual,
							   rsp, rsp_len,
							   &sas_actual);
			if (ret) {
				dev_dbg(&d->intf->dev,
					"failed to unwrap APDU response: %d\n",
					ret);
				ret = 0;
				msleep(TRANSPORT_POLL_DELAY_MS);
				continue;
			}
			if (!sas_actual) {
				msleep(TRANSPORT_POLL_DELAY_MS);
				continue;
			}

			if (!smitdtmb_sas_payload_matches(rsp, sas_actual,
							  expected_id,
							  expected_u16)) {
				dev_dbg(&d->intf->dev,
					"ignoring SAS payload len=%d id16=0x%04x id32=0x%08x data=%*ph\n",
					sas_actual,
					sas_actual >= 2 ? get_be16(rsp) : 0,
					sas_actual >= 4 ? get_be32(rsp) : 0,
					min(sas_actual, 32), rsp);
				msleep(TRANSPORT_POLL_DELAY_MS);
				continue;
			}

			if (actual)
				*actual = sas_actual;
			matched = true;
			break;
		}
		if (!matched) {
			dev_dbg(&d->intf->dev,
				"APDU response timeout expected_id=0x%08x after %d ms\n",
				expected_id, APDU_RESPONSE_TIMEOUT_MS);
			ret = -ETIMEDOUT;
		}
	}
	mutex_unlock(&d->cmd_lock);

	kfree(rx);
	return ret;
}

static int smitdtmb_send_apdu(struct smitdtmb_dev *d, const u8 *cmd, int cmd_len,
			      u8 *rsp, int rsp_len, int *actual)
{
	return smitdtmb_send_apdu_expect(d, cmd, cmd_len, rsp, rsp_len,
					actual, 0, false);
}

static int smitdtmb_send_apdu_set(struct smitdtmb_dev *d, const u8 *cmd,
				  int cmd_len, int complete_count,
				  u8 *stat, int stat_len, int *stat_actual,
				  int stat_check_count, const char *what)
{
	u8 *rx = NULL;
	u8 frame[96];
	int rx_len = 256;
	int frame_len;
	int ret;

	if (stat_actual)
		*stat_actual = 0;
	rx = kmalloc(rx_len, GFP_KERNEL);
	if (!rx)
		return -ENOMEM;

	mutex_lock(&d->cmd_lock);
	ret = smitdtmb_ci_startup_locked(d);
	if (!ret)
		ret = smitdtmb_drain_pending_locked(d, rx, rx_len);
	if (!ret) {
		frame_len = smitdtmb_wrap_sas_apdu(d, cmd, cmd_len,
						   frame, sizeof(frame));
		if (frame_len < 0)
			ret = frame_len;
		else
			ret = smitdtmb_set_frame_complete_locked(d, frame, frame_len,
								 complete_count,
								 what);
	}
	if (!ret && stat && stat_len) {
		ret = smitdtmb_tuner_stat_locked(d, stat, stat_len,
						 stat_check_count, what);
		if (ret >= 0) {
			if (stat_actual)
				*stat_actual = ret;
			ret = 0;
		}
	}
	mutex_unlock(&d->cmd_lock);

	kfree(rx);
	return ret;
}

static int smitdtmb_send_apdu_set_expect(struct smitdtmb_dev *d,
					 const u8 *cmd, int cmd_len,
					 int complete_count,
					 u8 *rsp, int rsp_len, int *actual,
					 u32 expected_id, const char *what)
{
	u8 *rx = NULL;
	u8 frame[96];
	int rx_len = max(rsp_len + 64, 256);
	int frame_len;
	unsigned long deadline;
	int ret;

	if (actual)
		*actual = 0;

	rx = kmalloc(rx_len, GFP_KERNEL);
	if (!rx)
		return -ENOMEM;

	mutex_lock(&d->cmd_lock);
	ret = smitdtmb_ci_startup_locked(d);
	if (!ret)
		ret = smitdtmb_drain_pending_locked(d, rx, rx_len);
	if (!ret) {
		frame_len = smitdtmb_wrap_sas_apdu(d, cmd, cmd_len,
						   frame, sizeof(frame));
		if (frame_len < 0)
			ret = frame_len;
		else
			ret = smitdtmb_set_frame_complete_locked(d, frame, frame_len,
								 complete_count,
								 what);
	}
	if (!ret) {
		bool matched = false;

		deadline = jiffies + msecs_to_jiffies(APDU_RESPONSE_TIMEOUT_MS);
		while (time_before(jiffies, deadline)) {
			int rx_actual = 0;
			int sas_actual = 0;

			ret = smitdtmb_read_data_tpdu_locked(d, rx, rx_len,
							     &rx_actual,
							     what);
			if (ret == -ETIMEDOUT || ret == -EAGAIN) {
				ret = 0;
				msleep(TRANSPORT_POLL_DELAY_MS);
				continue;
			}
			if (ret)
				break;
			if (!rx_actual) {
				msleep(TRANSPORT_POLL_DELAY_MS);
				continue;
			}

			ret = smitdtmb_extract_sas_payload(rx, rx_actual, rsp,
							   rsp_len, &sas_actual);
			if (ret) {
				ret = 0;
				msleep(TRANSPORT_POLL_DELAY_MS);
				continue;
			}
			if (!sas_actual) {
				msleep(TRANSPORT_POLL_DELAY_MS);
				continue;
			}
			if (!smitdtmb_sas_payload_matches(rsp, sas_actual,
							  expected_id, false)) {
				dev_dbg(&d->intf->dev,
					"%s ignoring SAS payload len=%d id32=0x%08x data=%*ph\n",
					what, sas_actual,
					sas_actual >= 4 ? get_be32(rsp) : 0,
					min(sas_actual, 32), rsp);
				msleep(TRANSPORT_POLL_DELAY_MS);
				continue;
			}

			if (actual)
				*actual = sas_actual;
			matched = true;
			break;
		}
		if (!matched) {
			dev_dbg(&d->intf->dev,
				"%s response timeout expected_id=0x%08x after %d ms\n",
				what, expected_id, APDU_RESPONSE_TIMEOUT_MS);
			ret = -ETIMEDOUT;
		}
	}
	mutex_unlock(&d->cmd_lock);

	kfree(rx);
	return ret;
}

static int smitdtmb_hot_reset(struct smitdtmb_dev *d)
{
	return usb_control_msg(d->udev, usb_sndctrlpipe(d->udev, 0),
			       CTRL_HOT_RESET_REQ, CTRL_HOT_RESET_REQTYPE,
			       0, 0, NULL, 0, 100);
}

static int smitdtmb_cmd_reset(struct smitdtmb_dev *d)
{
	u8 cmd[8] = { 0 };
	u8 rsp[32];
	int actual = 0;
	int ret;

	put_be16(cmd, SAS_CMD_RESET);
	put_be16(cmd + 2, 4);
	memcpy(cmd + 4, "REST", 4);

	ret = smitdtmb_send_apdu_expect(d, cmd, sizeof(cmd), rsp, sizeof(rsp),
					&actual, SAS_RSP_RESET, false);
	if (ret) {
		dev_info(&d->intf->dev, "REST command failed: %d\n", ret);
		return ret;
	}

	if (actual >= 8 && get_be32(rsp) == SAS_RSP_RESET) {
		ret = (int)get_be32(rsp + 4);
		dev_info(&d->intf->dev, "REST response len=%d code=%d\n", actual, ret);
		return ret;
	}

	dev_info(&d->intf->dev, "unexpected REST response len=%d id=0x%08x\n",
		 actual, actual >= 4 ? get_be32(rsp) : 0);
	return 0;
}

static int smitdtmb_cmd_tune(struct smitdtmb_dev *d, u32 freq_hz,
			     u32 symbol_rate, u32 modulation, u32 bandwidth_mhz)
{
	u8 cmd[20] = { 0 };
	u8 stat[900];
	int stat_actual = 0;
	int ret;

	put_be16(cmd, SAS_CMD_TUNE);
	put_be16(cmd + 2, 16);
	put_le32(cmd + 4, freq_hz / 1000);
	put_le32(cmd + 8, symbol_rate);
	put_le32(cmd + 12, modulation);
	put_le32(cmd + 16, bandwidth_mhz);

	dev_dbg(&d->intf->dev,
		"TUNE request freq=%uHz symbol=%u modulation=%u bandwidth=%uMHz\n",
		freq_hz, symbol_rate, modulation, bandwidth_mhz);

	d->frequency_hz = freq_hz;
	d->symbol_rate = symbol_rate;
	d->modulation = modulation;
	d->bandwidth_mhz = bandwidth_mhz;
	d->locked = true;

	/*
	 * Match the original iCast flow: submit TUNE, wait only for the short
	 * transport completion/status path, then let TS flow. Waiting for the
	 * SAS TUNE response blocks scans for 15 seconds on this stick.
	 */
	ret = smitdtmb_send_apdu_set(d, cmd, sizeof(cmd), 100, stat,
				     sizeof(stat), &stat_actual, 2, "TUNE");
	if (ret)
		dev_dbg(&d->intf->dev,
			"TUNE submit failed: %d; allowing TS read\n",
			ret);
	else
		dev_dbg(&d->intf->dev,
			"TUNE command submitted stat_len=%d\n", stat_actual);

	if (d->feed_count > 0 && !d->streaming)
		smitdtmb_start_streaming(d);
	return 0;
}

static int smitdtmb_cmd_status(struct smitdtmb_dev *d)
{
	u8 cmd[8] = { 0 };
	u8 rsp[64];
	int actual = 0;
	int ret;

	put_be16(cmd, SAS_CMD_STATUS);
	put_be16(cmd + 2, 4);
	memcpy(cmd + 4, "GSTA", 4);

	ret = smitdtmb_send_apdu_set(d, cmd, sizeof(cmd), 50, rsp, sizeof(rsp),
				     &actual, 5, "STATUS");
	if (ret)
		return ret;

	if (actual == 32) {
		const u8 *p = rsp;

		d->frequency_hz = get_le32(p + 16) * 1000;
		d->locked = !!p[29];
		d->snr = p[30] * 0x0101;
		d->strength = p[31] * 0x0101;
		dev_dbg(&d->intf->dev,
			"STATUS lock=%u snr=%u strength=%u freq=%u\n",
			d->locked, d->snr, d->strength, d->frequency_hz);
		return 0;
	}

	dev_dbg(&d->intf->dev, "unexpected status response, len=%d\n", actual);
	return 0;
}

static void smitdtmb_status_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct smitdtmb_dev *d = container_of(dwork, struct smitdtmb_dev,
					      status_work);
	int ret;

	if (d->disconnected || !d->streaming || !poll_status || !d->frequency_hz)
		return;

	ret = smitdtmb_cmd_status(d);
	if (ret)
		dev_dbg(&d->intf->dev, "STATUS poll failed: %d\n", ret);

	if (!d->disconnected && d->streaming && poll_status)
		schedule_delayed_work(&d->status_work,
				      msecs_to_jiffies(STATUS_POLL_INTERVAL_MS));
}

static void smitdtmb_urb_complete(struct urb *urb)
{
	struct smitdtmb_dev *d = urb->context;
	int ret;

	if (!d || d->disconnected || !d->streaming)
		return;

	if (urb->status == 0 && urb->actual_length > 0) {
		if (d->ts_packets < 8)
			dev_dbg(&d->intf->dev,
				"TS URB len=%d sync=%s first=%*ph\n",
				urb->actual_length,
				((u8 *)urb->transfer_buffer)[0] == 0x47 ? "yes" : "no",
				min(urb->actual_length, 16),
				urb->transfer_buffer);
		d->ts_packets++;
		dvb_dmx_swfilter(&d->demux, urb->transfer_buffer, urb->actual_length);
	} else if (urb->status && d->ts_errors < 8) {
		d->ts_errors++;
		dev_info(&d->intf->dev, "TS URB status=%d actual=%d\n",
			 urb->status, urb->actual_length);
	}

	if (urb->status == -ENOENT || urb->status == -ESHUTDOWN ||
	    urb->status == -ECONNRESET)
		return;

	usb_mark_last_busy(d->udev);
	ret = usb_submit_urb(urb, GFP_ATOMIC);
	if (ret)
		dev_dbg(&d->intf->dev, "resubmit TS URB failed: %d\n", ret);
}

static int smitdtmb_find_ts_sync(const u8 *buf, int len)
{
	int i, packets;

	for (i = 0; i < 188 && i < len; i++) {
		if (buf[i] != 0x47)
			continue;
		for (packets = 1; i + packets * 188 < len; packets++) {
			if (buf[i + packets * 188] != 0x47)
				break;
		}
		if (packets >= 3)
			return i;
	}

	return -1;
}

static int smitdtmb_ts_sync_thread(void *arg)
{
	struct smitdtmb_dev *d = arg;
	u8 *buf;
	int tail_len = 0;
	u32 read_count = 0;
	u32 error_count = 0;

	buf = kmalloc(TS_SYNC_BUFSIZE + 187, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	while (!kthread_should_stop() && !d->disconnected && d->streaming) {
		int actual = 0;
		int ret;

		ret = usb_bulk_msg(d->udev, usb_rcvbulkpipe(d->udev, EP_TS_IN_NUM),
				   buf + tail_len, TS_SYNC_BUFSIZE, &actual,
				   TS_SYNC_TIMEOUT_MS);
		if ((ret == 0 || ret == -EOVERFLOW) && actual > 0) {
			int total = tail_len + actual;
			int offset = smitdtmb_find_ts_sync(buf, total);
			int aligned = 0;
			int used = 0;

			if (offset >= 0)
				aligned = ((total - offset) / 188) * 188;
			if (read_count < 8)
				dev_dbg(&d->intf->dev,
					"TS sync read ret=%d len=%d total=%d offset=%d aligned=%d tail=%d first=%*ph\n",
					ret, actual, total, offset, aligned,
					tail_len, min(actual, 16),
					buf + tail_len);
			read_count++;
			if (aligned > 0) {
				dvb_dmx_swfilter(&d->demux, buf + offset, aligned);
				used = offset + aligned;
			} else if (total > 187) {
				used = total - 187;
			}
			tail_len = total - used;
			if (tail_len > 0 && used > 0)
				memmove(buf, buf + used, tail_len);
			continue;
		}

		if (ret && ret != -ETIMEDOUT && error_count < 8) {
			error_count++;
			dev_dbg(&d->intf->dev, "TS sync read ret=%d actual=%d\n",
				ret, actual);
		} else if (ret == -ETIMEDOUT && d->ts_timeouts < 8) {
			d->ts_timeouts++;
			dev_dbg(&d->intf->dev, "TS sync read timeout\n");
		}

		if (actual == 0)
			msleep(10);
	}

	kfree(buf);
	return 0;
}

static int smitdtmb_start_streaming(struct smitdtmb_dev *d)
{
	int i, ret;

	if (d->streaming)
		return 0;

	dev_dbg(&d->intf->dev, "starting TS streaming (%s)\n",
		ts_sync_reader ? "sync bulk" : "urb");
	ret = usb_clear_halt(d->udev, usb_rcvbulkpipe(d->udev, EP_TS_IN_NUM));
	if (ret)
		dev_info(&d->intf->dev, "clear ts-in halt failed: %d\n", ret);
	d->streaming = true;
	if (poll_status)
		schedule_delayed_work(&d->status_work,
				      msecs_to_jiffies(STATUS_POLL_INTERVAL_MS));
	if (ts_sync_reader) {
		d->ts_thread = kthread_run(smitdtmb_ts_sync_thread, d,
					   "smitdtmb-ts");
		if (IS_ERR(d->ts_thread)) {
			ret = PTR_ERR(d->ts_thread);
			d->ts_thread = NULL;
			d->streaming = false;
			dev_err(&d->intf->dev, "start TS thread failed: %d\n", ret);
			return ret;
		}
		return 0;
	}

	for (i = 0; i < TS_URB_COUNT; i++) {
		ret = usb_submit_urb(d->urbs[i], GFP_KERNEL);
		if (ret) {
			dev_err(&d->intf->dev, "submit TS URB %d failed: %d\n", i, ret);
			while (--i >= 0)
				usb_kill_urb(d->urbs[i]);
			d->streaming = false;
			return ret;
		}
	}

	return 0;
}

static void smitdtmb_stop_streaming(struct smitdtmb_dev *d)
{
	int i;

	if (!d->streaming)
		return;

	dev_dbg(&d->intf->dev, "stopping TS streaming\n");
	d->streaming = false;
	cancel_delayed_work(&d->status_work);
	if (d->ts_thread) {
		kthread_stop(d->ts_thread);
		d->ts_thread = NULL;
		return;
	}

	for (i = 0; i < TS_URB_COUNT; i++)
		usb_kill_urb(d->urbs[i]);
}

static int smitdtmb_start_feed(struct dvb_demux_feed *feed)
{
	struct smitdtmb_dev *d = feed->demux->priv;
	unsigned long flags;
	bool first_feed = false;
	bool log_feed = false;
	int feed_count;
	int ret = 0;

	spin_lock_irqsave(&d->feed_lock, flags);
	if (d->feed_count++ == 0) {
		first_feed = true;
		feed_count = d->feed_count;
		if (d->feed_logs < 16) {
			d->feed_logs++;
			log_feed = true;
		}
		spin_unlock_irqrestore(&d->feed_lock, flags);
			if (log_feed)
				dev_dbg(&d->intf->dev,
					"start feed pid=0x%04x type=%u pes=%u count=%d first feed\n",
					feed->pid, feed->type, feed->pes_type,
					feed_count);
			if (!d->locked) {
				dev_dbg(&d->intf->dev,
					"deferring TS streaming until tune completes\n");
				return 0;
			}
			ret = smitdtmb_start_streaming(d);
		if (ret) {
			spin_lock_irqsave(&d->feed_lock, flags);
			d->feed_count--;
			spin_unlock_irqrestore(&d->feed_lock, flags);
		}
		return ret;
	}
	if (d->feed_logs < 16) {
		d->feed_logs++;
		log_feed = true;
	}
	feed_count = d->feed_count;
	spin_unlock_irqrestore(&d->feed_lock, flags);

	if (log_feed)
		dev_dbg(&d->intf->dev,
			"start feed pid=0x%04x type=%u pes=%u count=%d%s\n",
			feed->pid, feed->type, feed->pes_type, feed_count,
			first_feed ? " first feed" : "");

	return 0;
}

static int smitdtmb_stop_feed(struct dvb_demux_feed *feed)
{
	struct smitdtmb_dev *d = feed->demux->priv;
	unsigned long flags;
	bool stop = false;

	spin_lock_irqsave(&d->feed_lock, flags);
	if (d->feed_count > 0 && --d->feed_count == 0)
		stop = true;
	spin_unlock_irqrestore(&d->feed_lock, flags);

	if (stop)
		smitdtmb_stop_streaming(d);

	return 0;
}

static int smitdtmb_fe_init(struct dvb_frontend *fe)
{
	struct smitdtmb_dev *d = fe->demodulator_priv;
	int ret;

	if (do_hot_reset) {
		smitdtmb_hot_reset(d);
		msleep(200);
	}
	dev_dbg(&d->intf->dev, "frontend init\n");
	if (!do_cmd_reset) {
		dev_dbg(&d->intf->dev, "frontend init skipped REST APDU\n");
		return 0;
	}
	ret = smitdtmb_cmd_reset(d);
	dev_dbg(&d->intf->dev, "frontend init result=%d\n", ret);
	return ret;
}

static int smitdtmb_fe_set_frontend(struct dvb_frontend *fe)
{
	struct smitdtmb_dev *d = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	u32 bandwidth_mhz = 8;
	u32 modulation = 64;
	u32 symbol_rate = c->symbol_rate;

	if (c->bandwidth_hz)
		bandwidth_mhz = c->bandwidth_hz / 1000000;

	if (c->delivery_system == SYS_DTMB || c->delivery_system == SYS_DVBT) {
		symbol_rate = 0;
		modulation = 0;
	} else {
		if (symbol_rate > 100000)
			symbol_rate /= 1000;
		switch (c->modulation) {
		case QAM_16:
			modulation = 16;
			break;
		case QAM_32:
			modulation = 32;
			break;
		case QAM_64:
			modulation = 64;
			break;
		case QAM_128:
			modulation = 128;
			break;
		case QAM_256:
			modulation = 256;
			break;
		case QAM_AUTO:
		default:
			modulation = 64;
			break;
		}
	}

	dev_dbg(&d->intf->dev, "set_frontend delsys=%u frequency=%u bandwidth_hz=%u modulation=%u\n",
		c->delivery_system, c->frequency, c->bandwidth_hz, c->modulation);

	return smitdtmb_cmd_tune(d, c->frequency, symbol_rate,
				 modulation, bandwidth_mhz);
}

static int smitdtmb_fe_get_frontend(struct dvb_frontend *fe,
				    struct dtv_frontend_properties *c)
{
	struct smitdtmb_dev *d = fe->demodulator_priv;

	c->frequency = d->frequency_hz;
	c->symbol_rate = d->symbol_rate;
	c->modulation = d->modulation;
	c->bandwidth_hz = d->bandwidth_mhz * 1000000;
	return 0;
}

static int smitdtmb_fe_read_status(struct dvb_frontend *fe, enum fe_status *status)
{
	struct smitdtmb_dev *d = fe->demodulator_priv;

	*status = d->locked ? (FE_HAS_SIGNAL | FE_HAS_CARRIER | FE_HAS_VITERBI |
			       FE_HAS_SYNC | FE_HAS_LOCK) : 0;
	return 0;
}

static int smitdtmb_fe_read_signal_strength(struct dvb_frontend *fe, u16 *strength)
{
	struct smitdtmb_dev *d = fe->demodulator_priv;

	*strength = d->strength;
	return 0;
}

static int smitdtmb_fe_read_snr(struct dvb_frontend *fe, u16 *snr)
{
	struct smitdtmb_dev *d = fe->demodulator_priv;

	*snr = d->snr;
	return 0;
}

static int smitdtmb_fe_read_ber(struct dvb_frontend *fe, u32 *ber)
{
	*ber = 0;
	return 0;
}

static int smitdtmb_fe_read_ucblocks(struct dvb_frontend *fe, u32 *ucblocks)
{
	*ucblocks = 0;
	return 0;
}

static void smitdtmb_fe_release(struct dvb_frontend *fe)
{
}

static const struct dvb_frontend_ops smitdtmb_fe_ops = {
	.delsys = { SYS_DVBT, SYS_DVBC_ANNEX_A },
	.info = {
		.name = "SMIT DTMB/DVBC frontend",
		.frequency_min_hz = SMITDTMB_FREQ_MIN_HZ,
		.frequency_max_hz = SMITDTMB_FREQ_MAX_HZ,
		.frequency_stepsize_hz = 1000,
		.caps = FE_CAN_QAM_64 | FE_CAN_FEC_AUTO | FE_CAN_TRANSMISSION_MODE_AUTO |
			FE_CAN_GUARD_INTERVAL_AUTO | FE_CAN_RECOVER,
	},
	.release = smitdtmb_fe_release,
	.init = smitdtmb_fe_init,
	.set_frontend = smitdtmb_fe_set_frontend,
	.get_frontend = smitdtmb_fe_get_frontend,
	.read_status = smitdtmb_fe_read_status,
	.read_signal_strength = smitdtmb_fe_read_signal_strength,
	.read_snr = smitdtmb_fe_read_snr,
	.read_ber = smitdtmb_fe_read_ber,
	.read_ucblocks = smitdtmb_fe_read_ucblocks,
};

static int smitdtmb_alloc_urbs(struct smitdtmb_dev *d)
{
	int i;

	for (i = 0; i < TS_URB_COUNT; i++) {
		d->urbs[i] = usb_alloc_urb(0, GFP_KERNEL);
		if (!d->urbs[i])
			return -ENOMEM;

		d->urb_bufs[i] = usb_alloc_coherent(d->udev, TS_URB_BUFSIZE,
						    GFP_KERNEL, &d->urb_dma[i]);
		if (!d->urb_bufs[i])
			return -ENOMEM;

		usb_fill_bulk_urb(d->urbs[i], d->udev,
				  usb_rcvbulkpipe(d->udev, EP_TS_IN_NUM),
				  d->urb_bufs[i], TS_URB_BUFSIZE,
				  smitdtmb_urb_complete, d);
		d->urbs[i]->transfer_dma = d->urb_dma[i];
		d->urbs[i]->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	}

	return 0;
}

static bool smitdtmb_has_endpoint(struct usb_host_interface *alt, u8 addr)
{
	int i;

	for (i = 0; i < alt->desc.bNumEndpoints; i++) {
		if (alt->endpoint[i].desc.bEndpointAddress == addr)
			return true;
	}

	return false;
}

static int smitdtmb_check_endpoints(struct usb_interface *intf)
{
	struct usb_host_interface *alt = intf->cur_altsetting;

	if (!smitdtmb_has_endpoint(alt, EP_CMD_OUT_ADDR) ||
	    !smitdtmb_has_endpoint(alt, EP_CMD_IN_ADDR) ||
	    !smitdtmb_has_endpoint(alt, EP_TS_IN_ADDR)) {
		dev_err(&intf->dev,
			"required endpoints missing, need cmd-out 0x%02x cmd-in 0x%02x ts-in 0x%02x\n",
			EP_CMD_OUT_ADDR, EP_CMD_IN_ADDR, EP_TS_IN_ADDR);
		return -ENODEV;
	}

	return 0;
}

static void smitdtmb_free_urbs(struct smitdtmb_dev *d)
{
	int i;

	for (i = 0; i < TS_URB_COUNT; i++) {
		if (d->urbs[i]) {
			usb_kill_urb(d->urbs[i]);
			usb_free_urb(d->urbs[i]);
			d->urbs[i] = NULL;
		}
		if (d->urb_bufs[i]) {
			usb_free_coherent(d->udev, TS_URB_BUFSIZE, d->urb_bufs[i],
					  d->urb_dma[i]);
			d->urb_bufs[i] = NULL;
		}
	}
}

static int smitdtmb_register_dvb(struct smitdtmb_dev *d)
{
	int ret;

	ret = dvb_register_adapter(&d->adapter, SMITDTMB_ADAPTER_NAME, THIS_MODULE,
				   &d->intf->dev, adapter_nr);
	if (ret < 0)
		return ret;

	d->demux.dmx.capabilities = DMX_TS_FILTERING | DMX_SECTION_FILTERING |
				    DMX_MEMORY_BASED_FILTERING;
	d->demux.priv = d;
	d->demux.filternum = 256;
	d->demux.feednum = 256;
	d->demux.start_feed = smitdtmb_start_feed;
	d->demux.stop_feed = smitdtmb_stop_feed;

	ret = dvb_dmx_init(&d->demux);
	if (ret)
		goto err_adapter;

	d->dmxdev.filternum = 256;
	d->dmxdev.demux = &d->demux.dmx;
	d->dmxdev.capabilities = 0;
	ret = dvb_dmxdev_init(&d->dmxdev, &d->adapter);
	if (ret)
		goto err_demux;

	d->hw_frontend.source = DMX_FRONTEND_0;
	ret = d->demux.dmx.add_frontend(&d->demux.dmx, &d->hw_frontend);
	if (ret)
		goto err_dmxdev;

	d->mem_frontend.source = DMX_MEMORY_FE;
	ret = d->demux.dmx.add_frontend(&d->demux.dmx, &d->mem_frontend);
	if (ret)
		goto err_remove_hw_fe;

	ret = d->demux.dmx.connect_frontend(&d->demux.dmx, &d->hw_frontend);
	if (ret)
		goto err_remove_mem_fe;

	memcpy(&d->fe.ops, &smitdtmb_fe_ops, sizeof(d->fe.ops));
	d->fe.demodulator_priv = d;
	ret = dvb_register_frontend(&d->adapter, &d->fe);
	if (ret)
		goto err_disconnect_fe;

	return 0;

err_disconnect_fe:
	d->demux.dmx.close(&d->demux.dmx);
err_remove_mem_fe:
	d->demux.dmx.remove_frontend(&d->demux.dmx, &d->mem_frontend);
err_remove_hw_fe:
	d->demux.dmx.remove_frontend(&d->demux.dmx, &d->hw_frontend);
err_dmxdev:
	dvb_dmxdev_release(&d->dmxdev);
err_demux:
	dvb_dmx_release(&d->demux);
err_adapter:
	dvb_unregister_adapter(&d->adapter);
	return ret;
}

static void smitdtmb_unregister_dvb(struct smitdtmb_dev *d)
{
	dvb_unregister_frontend(&d->fe);
	d->demux.dmx.close(&d->demux.dmx);
	d->demux.dmx.remove_frontend(&d->demux.dmx, &d->mem_frontend);
	d->demux.dmx.remove_frontend(&d->demux.dmx, &d->hw_frontend);
	dvb_dmxdev_release(&d->dmxdev);
	dvb_dmx_release(&d->demux);
	dvb_unregister_adapter(&d->adapter);
}

static int smitdtmb_probe(struct usb_interface *intf,
			  const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(intf);
	struct smitdtmb_dev *d;
	int ret;

	d = kzalloc(sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;

	d->udev = usb_get_dev(udev);
	d->intf = intf;
	d->sas_seq = 0;
	mutex_init(&d->cmd_lock);
	spin_lock_init(&d->feed_lock);
	INIT_DELAYED_WORK(&d->status_work, smitdtmb_status_work);
	usb_set_intfdata(intf, d);

	if (set_interface_on_probe) {
		ret = usb_set_interface(udev, intf->cur_altsetting->desc.bInterfaceNumber, 0);
		if (ret)
			dev_info(&intf->dev, "usb_set_interface failed: %d\n", ret);
	}

	ret = smitdtmb_check_endpoints(intf);
	if (ret)
		goto err;

	if (init_on_probe) {
		mutex_lock(&d->cmd_lock);
		ret = smitdtmb_ci_startup_locked(d);
		mutex_unlock(&d->cmd_lock);
		if (ret) {
			dev_info(&intf->dev, "probe CI/SAS startup failed: %d\n", ret);
			goto err;
		}
	}

	ret = smitdtmb_alloc_urbs(d);
	if (ret) {
		smitdtmb_free_urbs(d);
		goto err;
	}

	ret = smitdtmb_register_dvb(d);
	if (ret)
		goto err_urbs;

	if (do_hot_reset &&
	    le16_to_cpu(udev->descriptor.idVendor) == USB_VID_SMIT &&
	    le16_to_cpu(udev->descriptor.idProduct) == USB_PID_SMIT_DONGLE) {
		ret = smitdtmb_hot_reset(d);
		if (ret < 0)
			dev_dbg(&intf->dev, "hot reset command failed: %d\n", ret);
	}

	dev_info(&intf->dev, "registered %s on %04x:%04x\n", SMITDTMB_ADAPTER_NAME,
		 le16_to_cpu(udev->descriptor.idVendor),
		 le16_to_cpu(udev->descriptor.idProduct));
	return 0;

err_urbs:
	smitdtmb_free_urbs(d);
err:
	usb_set_intfdata(intf, NULL);
	usb_put_dev(d->udev);
	kfree(d);
	return ret;
}

static void smitdtmb_disconnect(struct usb_interface *intf)
{
	struct smitdtmb_dev *d = usb_get_intfdata(intf);

	if (!d)
		return;

	usb_set_intfdata(intf, NULL);
	d->disconnected = true;
	smitdtmb_stop_streaming(d);
	cancel_delayed_work_sync(&d->status_work);
	smitdtmb_unregister_dvb(d);
	smitdtmb_free_urbs(d);
	usb_put_dev(d->udev);
	kfree(d);
}

static const struct usb_device_id smitdtmb_id_table[] = {
	{ USB_DEVICE(USB_VID_SMIT, USB_PID_SMIT_DONGLE) },
	{ USB_DEVICE(USB_VID_FILTER_EXTRA, USB_PID_FILTER_EXTRA) },
	{ }
};
MODULE_DEVICE_TABLE(usb, smitdtmb_id_table);

static struct usb_driver smitdtmb_usb_driver = {
	.name = SMITDTMB_DRV_NAME,
	.probe = smitdtmb_probe,
	.disconnect = smitdtmb_disconnect,
	.id_table = smitdtmb_id_table,
};

module_usb_driver(smitdtmb_usb_driver);

MODULE_DESCRIPTION("SMIT/Ronghe DTMB USB DVB driver");
MODULE_AUTHOR("Billtv");
MODULE_LICENSE("GPL");
