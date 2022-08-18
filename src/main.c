/*
 * Copyright (c) 2018 Henrik Brix Andersen <henrik@brixandersen.dk>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <host/id.h>
#include <host/hci_core.h>
#include <zephyr/sys/byteorder.h>

#ifndef IBEACON_RSSI
#define IBEACON_RSSI 0xc8
#endif

#ifndef IBEACON_RSSI_2
#define IBEACON_RSSI_2 0x64
#endif

LOG_MODULE_REGISTER(ibeacon_demo, LOG_LEVEL_DBG);

/*
 * Set iBeacon demo advertisement data. These values are for
 * demonstration only and must be changed for production environments!
 *
 * UUID:  18ee1516-016b-4bec-ad96-bcb96d166e97
 * Major: 0
 * Minor: 0
 * RSSI:  -56 dBm
 */
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
	BT_DATA_BYTES(BT_DATA_MANUFACTURER_DATA,
		      0x4c, 0x00, /* Apple */
		      0x02, 0x15, /* iBeacon */
		      0x18, 0xee, 0x15, 0x16, /* UUID[15..12] */
		      0x01, 0x6b, /* UUID[11..10] */
		      0x4b, 0xec, /* UUID[9..8] */
		      0xad, 0x96, /* UUID[7..6] */
		      0xbc, 0xb9, 0x6d, 0x16, 0x6e, 0x97, /* UUID[5..0] */
		      0x00, 0x00, /* Major */
		      0x00, 0x00, /* Minor */
		      IBEACON_RSSI) /* Calibrated RSSI @ 1m */
};

static const struct bt_data ad_second[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
	BT_DATA_BYTES(BT_DATA_MANUFACTURER_DATA,
		      0x2e, 0x01, /* Assa abloy */
		      0x02, 0x15, /* iBeacon */
		      0x18, 0xee, 0x15, 0x16, /* UUID[15..12] */
		      0x01, 0x6b, /* UUID[11..10] */
		      0x4b, 0xec, /* UUID[9..8] */
		      0xad, 0x96, /* UUID[7..6] */
		      0xbc, 0xb9, 0x6d, 0x16, 0x6e, 0x97, /* UUID[5..0] */
		      0x00, 0x00, /* Major */
		      0x00, 0x00, /* Minor */
		      IBEACON_RSSI_2) /* Calibrated RSSI @ 1m */
};

static struct bt_le_adv_param adv_param1=BT_LE_ADV_PARAM_INIT(
	BT_LE_ADV_OPT_USE_IDENTITY, BT_GAP_ADV_FAST_INT_MIN_2, BT_GAP_ADV_FAST_INT_MAX_2, NULL);
static struct bt_le_adv_param adv_param2=BT_LE_ADV_PARAM_INIT(
	BT_LE_ADV_OPT_USE_IDENTITY, BT_GAP_ADV_FAST_INT_MIN_2, BT_GAP_ADV_FAST_INT_MAX_2, NULL);

static const struct bt_data* adv_array[]={ad_second,ad};
//static struct bt_le_adv_param* adv_param_array[CONFIG_BT_ID_MAX];
static const size_t adv_lengths[]={ARRAY_SIZE(ad_second),ARRAY_SIZE(ad)};

static bt_addr_t pubBtAddr={.val={0x5a,0xa5,0xbe,0x2e,0x01,0x00}};
static bt_addr_le_t iBeacon_adr={ 
								  .type=BT_ADDR_LE_RANDOM,
								  .a.val={0xcc,0xbb,0xaa,0x23,0x01,0x00}};

static bt_addr_le_t iBeacon_adr2={ 
								  .type=BT_ADDR_LE_RANDOM,
								  .a.val={0xcc,0xbb,0xaa,0x23,0x02,0x00}};


struct k_timer my_timer;
static uint8_t beacon_cnt=0;

static struct bt_le_ext_adv *adv_set1;
static struct bt_le_ext_adv *adv_set2;

static struct bt_le_ext_adv_start_param ext_adv_start_param = {
	.timeout = 0, //N*10ms
	.num_events = 0, 
};

static struct bt_le_ext_adv_start_param ext_adv_start_param_timeout = {
	.timeout = (BT_GAP_ADV_FAST_INT_MAX_2)/10, //N*10ms
	.num_events = 0, 
};

static struct bt_le_ext_adv_start_param ext_adv_start_param_one_event = {
	.timeout = 0, //N*10ms
	.num_events = 1, 
};

void my_expiry_function(struct k_timer *timer_id){
	ARG_UNUSED(timer_id);
}

/****** CHANGE PUBLIC ADRESS ****/ 
struct bt_aci_hal_write_config_data_bd_addr {
	uint8_t offset; //must be 0x00 for pubAddr_offset
	uint8_t len;
	bt_addr_t bdaddr;
} __packed;

int bt_setup_public_id_addr(void); //Definition in /host/id.c but not predefined in id.h 

static void set_ble_public_address(bt_addr_t *addr){

	struct net_buf *buf, *rsp = NULL;
	char addr_s[BT_ADDR_LE_STR_LEN];

	struct bt_aci_hal_write_config_data_bd_addr *cp;

	int err;
	/* Uses  ACI_HAL_WRITE_CONFIG_DATA opcode = 0xFC0C
	*/
	uint16_t opcode=0xFC0C;
	buf = bt_hci_cmd_create(opcode, sizeof(*cp));
	if (!buf) {
		LOG_INF("Unable to allocate command buffer\n");
		return;
	}

	cp = net_buf_add(buf, sizeof(*cp));
	cp->offset=0;
	cp->len=6;
	memcpy(&cp->bdaddr,addr,sizeof(bt_addr_t));

	err = bt_hci_cmd_send_sync(opcode,
				   buf, &rsp);

	if (err){
		LOG_INF("Command ACI_HAL_WRITE_CONFIG_DATA to PUBADDR_OFFSET not executed properly\n");
		return;
	}

	bt_addr_to_str(addr,addr_s,sizeof(addr_s));
	LOG_INF("Set public address to %s\n",addr_s);

	bt_setup_public_id_addr();

	net_buf_unref(rsp);

}
/****** CHANGE PUBLIC ADRESS STM32 ****/ 
extern int update_params_legacy(const struct bt_le_adv_param *param);

void function_to_update_ble_ad(void){
	if (bt_is_ready()){
		int err = bt_le_adv_update_data(adv_array[beacon_cnt], adv_lengths[beacon_cnt],
			      NULL, 0);
		//update_params_legacy(adv_param_array[beacon_cnt]);

		if (err) {
			LOG_ERR("Bluetooth update failed (err %x)", err);
		}
		else{
			LOG_INF("Bluetooth updated");
		}
		
	}
	
	beacon_cnt++;
	if (beacon_cnt>=ARRAY_SIZE(adv_array)){
		beacon_cnt=0;
	}
}

static void adv_sent_cb(struct bt_le_ext_adv *adv, struct bt_le_ext_adv_sent_info *info)
{
	int err;
	if (beacon_cnt==0){
		//err = bt_le_ext_adv_start(adv_set1, &ext_adv_start_param);
		err = bt_le_ext_adv_start(adv_set1, &ext_adv_start_param_timeout);
		beacon_cnt++;
		printk("advertising 2 done \n");
	}
	else if(beacon_cnt==1){
		//err = bt_le_ext_adv_start(adv_set2, &ext_adv_start_param);
		err = bt_le_ext_adv_start(adv_set2, &ext_adv_start_param_timeout);
		beacon_cnt=0;
		printk("advertising 1 done \n");
	}
	
}
	
static struct bt_le_ext_adv_cb adv_callbacks = {
	.sent = adv_sent_cb,
};

/* Read supported simultaneous adv sets */
static int ble_read_num_adv_sets(int *numsets)
{
	struct bt_hci_rp_le_read_num_adv_sets *rp;
	struct net_buf *rsp = NULL;
	int err;

	*numsets = 0;

	/* Read Bluetooth Address */
	err = bt_hci_cmd_send_sync(BT_HCI_OP_LE_READ_NUM_ADV_SETS, NULL, &rsp);
	if (err) {
		uint8_t status = ((struct bt_hci_rp_le_read_num_adv_sets *)rsp->data)->status;
		LOG_ERR("ACI tone stop error: %d (status 0x%02x)", err, status);
		return err;
	}

	rp = (struct bt_hci_rp_le_read_num_adv_sets *)rsp->data;

	*numsets = rp->num_sets;

	net_buf_unref(rsp);

	return 0;
}


/* Mockup to start simultaneously two advertisings in extended mode - this is not supported yet in zephyr*/
static int bt_le_adv_set_enable_two(struct bt_le_ext_adv *adv1,
			 struct bt_le_ext_adv *adv2,
			 bool enable,
			 const struct bt_le_ext_adv_start_param *param1,
			 const struct bt_le_ext_adv_start_param *param2)
{
	struct net_buf *buf;
	struct bt_hci_cmd_state_set state;
	int err;

	/* 2 constant bytes, 4 bytes per adv set */
	buf = bt_hci_cmd_create(BT_HCI_OP_LE_SET_EXT_ADV_ENABLE, 10);
	if (!buf) {
		return -ENOBUFS;
	}

	if (enable) {
		net_buf_add_u8(buf, BT_HCI_LE_ADV_ENABLE);
	} else {
		net_buf_add_u8(buf, BT_HCI_LE_ADV_DISABLE);
	}

	/* two sets to enable */
	net_buf_add_u8(buf, 2);

	/* first set */
	net_buf_add_u8(buf, adv1->handle);
	net_buf_add_le16(buf, param1 ? sys_cpu_to_le16(param1->timeout) : 0);
	net_buf_add_u8(buf, param1 ? param1->num_events : 0);
	bt_hci_cmd_state_set_init(buf, &state, adv1->flags, BT_ADV_ENABLED, enable);
	/* second set */
	net_buf_add_u8(buf, adv2->handle);
	net_buf_add_le16(buf, param2 ? sys_cpu_to_le16(param2->timeout) : 0);
	net_buf_add_u8(buf, param2 ? param2->num_events : 0);
	bt_hci_cmd_state_set_init(buf, &state, adv2->flags, BT_ADV_ENABLED, enable);

	err = bt_hci_cmd_send_sync(BT_HCI_OP_LE_SET_EXT_ADV_ENABLE, buf, NULL);
	if (err) {
		return err;
	}

	return 0;
}

static void bt_ready(int err)
{
	k_timer_init(&my_timer, my_expiry_function, NULL);
	//k_timer_start(&my_timer, K_MSEC(BT_GAP_ADV_FAST_INT_MIN_2), K_MSEC(BT_GAP_ADV_FAST_INT_MIN_2));
	k_timer_start(&my_timer, K_MSEC(2000), K_MSEC(2000));

	//bt_addr_le_t addr = { 0 };
	int id=0;

	if (err) {
		LOG_ERR("Bluetooth init failed (err %x)", err);
		return;
	}

	int ext_numsets=0;
	ble_read_num_adv_sets(&ext_numsets);
	LOG_INF("Bluetooth ext adv numsets simultaneous: %d", ext_numsets);

	/* Set adress as static */
	BT_ADDR_SET_STATIC((&iBeacon_adr.a)); 
	BT_ADDR_SET_STATIC((&iBeacon_adr2.a)); 
	set_ble_public_address(&pubBtAddr);

	LOG_INF("Bluetooth initialized");
	id = bt_id_create(&iBeacon_adr, NULL);

	/* Set param id 1 to Default public address */
	adv_param1.sid = 0;
	//adv_param1.id=BT_ID_DEFAULT;
	adv_param1.id=id;

	/* Set param id 2 to Ibeacon  address */
	id = bt_id_create(&iBeacon_adr2, NULL);
	adv_param2.sid = 1;
	adv_param2.id=id;

	err = bt_le_ext_adv_create(&adv_param1, &adv_callbacks, &adv_set1);
	if (err) {
		LOG_ERR("failed (err %x)\n", err);
		return;
	}

	err = bt_le_ext_adv_create(&adv_param2, &adv_callbacks, &adv_set2);
	if (err) {
		LOG_ERR("failed (err %x)\n", err);
		return;
	}

	/* Write data sets*/
	err = bt_le_ext_adv_set_data(adv_set1, adv_array[0], adv_lengths[0], NULL,
				     0);
	if (err) {
		LOG_ERR("failed (err %x)\n", err);
		return;
	}  

	err = bt_le_ext_adv_set_data(adv_set2, adv_array[1], adv_lengths[1], NULL,
				     0);
	if (err) {
		LOG_ERR("failed (err %x)\n", err);
		return;
	}  

	err = bt_le_ext_adv_start(adv_set2, &ext_adv_start_param);
	if (err) {
		LOG_ERR("failed 1 adv set start (err %x)\n", err);
		return;
	}
	err = bt_le_ext_adv_start(adv_set2, &ext_adv_start_param);

	if (err) {
		LOG_ERR("failed 2 adv set start (err %x)\n", err);
		return;
	}

	LOG_INF("Extended advertising enable...");

	LOG_INF("Beacons started\n");

}

void main(void)
{
	int err;

	LOG_INF("Starting iBeacon Demo\n");

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(bt_ready);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %x)\n", err);
	}

	while(1){
		k_timer_status_sync(&my_timer);
		//function_to_update_ble_ad();
		//LOG_INF("Timer tick");
	}
	
}
