/*
 * lmic.c
 *
 *  Created on: 17 sep. 2016
 *      Author: Jaap Braam
 */
#include "module.h"
#include "lauxlib.h"
#include "platform.h"
#include "hw_timer.h"
#include "cpu_esp8266.h"
#include "gpio.h"
#include "lmic.h"
#include "hal.h"
#include "rtc/rtctime.h"

#include "c_types.h"
#include "c_string.h"

unsigned char LMIC_DEBUG_LEVEL=0;

static u32 nss_pin=GPIO_ID_NONE;
static u32 rst_pin=GPIO_ID_NONE;
static u32 rxtx_pin=GPIO_ID_NONE;

static u32 dio0_pin=GPIO_ID_NONE;
static u32 dio0_mask;
static u32 dio1_pin=GPIO_ID_NONE;
static u32 dio1_mask;
static u32 dio2_pin=GPIO_ID_NONE;
static u32 dio2_mask;

extern void radio_irq_handler(u8 dio);

static uint32_t  dio_hook(uint32_t  ret_gpio_status) {
	if (ret_gpio_status & dio0_mask){
		radio_irq_handler(0);
		//c_printf("dio %d\n",0);
		GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, ret_gpio_status & dio0_mask);
		ret_gpio_status&=!dio0_mask;
	}
	if (ret_gpio_status & dio1_mask){
		radio_irq_handler(1);
		//c_printf("dio %d\n",1);
		GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, ret_gpio_status & dio1_mask);
		ret_gpio_status&=!dio1_mask;
	}
	if (ret_gpio_status & dio2_mask){
		radio_irq_handler(2);
		//c_printf("dio %d\n",2);
		GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, ret_gpio_status & dio2_mask);
		ret_gpio_status&=!dio2_mask;
	}
	return ret_gpio_status;
}

static const os_param_t LMIC_OWNER = 0x6C6D6963; // "lmic"

static void ICACHE_RAM_ATTR lmic_cb(os_param_t p) {
  os_runloop_once();
  platform_hw_timer_arm_us(LMIC_OWNER, 10);
}
//===============================================================================================
// LMIC HAL interface
//===============================================================================================

/*
 * initialize hardware (IO, SPI, TIMER, IRQ).
 */
void hal_init(void) {
	// IO
	if (rxtx_pin != GPIO_ID_NONE)
		platform_gpio_mode(rxtx_pin, PLATFORM_GPIO_OUTPUT, PLATFORM_GPIO_FLOAT);
	if (rst_pin != GPIO_ID_NONE)
		platform_gpio_mode(rst_pin, PLATFORM_GPIO_OUTPUT, PLATFORM_GPIO_FLOAT);

	// SPI
	if (nss_pin != GPIO_ID_NONE)
		platform_gpio_mode(nss_pin, PLATFORM_GPIO_OUTPUT, PLATFORM_GPIO_FLOAT);
	platform_spi_setup(1, PLATFORM_SPI_MASTER, PLATFORM_SPI_CPOL_LOW,PLATFORM_SPI_CPHA_LOW, 3);

	// TIMER
	struct rtc_timeval tv;
	rtctime_gettimeofday(&tv);
	rtctime_settimeofday(&tv);

	// IRQ
	u32 mask=0;
	if (dio0_pin != GPIO_ID_NONE) {
		platform_gpio_mode(dio0_pin, PLATFORM_GPIO_INT, PLATFORM_GPIO_FLOAT);
		platform_gpio_intr_init(dio0_pin,GPIO_PIN_INTR_POSEDGE);
		dio0_mask=BIT(pin_num[dio0_pin]);
		mask|=dio0_mask;
	}
	if (dio1_pin != GPIO_ID_NONE){
		platform_gpio_mode(dio1_pin, PLATFORM_GPIO_INT, PLATFORM_GPIO_FLOAT);
		platform_gpio_intr_init(dio1_pin,GPIO_PIN_INTR_POSEDGE);
		dio1_mask=BIT(pin_num[dio1_pin]);
		mask|=dio1_mask;
	}
	if (dio2_pin != GPIO_ID_NONE){
		platform_gpio_mode(dio2_pin, PLATFORM_GPIO_INT, PLATFORM_GPIO_FLOAT);
		platform_gpio_intr_init(dio2_pin,GPIO_PIN_INTR_POSEDGE);
		dio2_mask=BIT(pin_num[dio2_pin]);
		mask|=dio2_mask;
	}
	if (mask){
		platform_gpio_register_intr_hook(mask, dio_hook);
	}
	// lmic callback
	platform_hw_timer_init(LMIC_OWNER, FRC1_SOURCE, FALSE);
	platform_hw_timer_set_func(LMIC_OWNER,lmic_cb,0);
    platform_hw_timer_arm_us(LMIC_OWNER, 10);
}

/*
 * drive radio NSS pin (0=low, 1=high).
 */
void hal_pin_nss(u1_t val) {
	platform_gpio_write(nss_pin, val);
}

/*
 * drive radio RX/TX pins (0=rx, 1=tx).
 */
void hal_pin_rxtx(u1_t val) {
	if (rxtx_pin == GPIO_ID_NONE) return;
	platform_gpio_write(rxtx_pin, val);
}

/*
 * control radio RST pin (0=low, 1=high, 2=floating)
 */
void hal_pin_rst(u1_t val) {
	if (rst_pin == GPIO_ID_NONE) return;
	if (val == 2)
		platform_gpio_mode(rst_pin, PLATFORM_GPIO_INPUT, PLATFORM_GPIO_FLOAT);
	else
		platform_gpio_write(rst_pin, val);
}

/*
 * perform 8-bit SPI transaction with radio.
 *   - write given byte 'outval'
 *   - read byte and return value
 */
u1_t hal_spi (u1_t outval){
	return platform_spi_send_recv(1, 8, outval);
}

/*
 * disable all CPU interrupts.
 *   - might be invoked nested
 *   - will be followed by matching call to hal_enableIRQs()
 */
void hal_disableIRQs (void){
	ets_intr_lock();
}

/*
 * enable CPU interrupts.
 */
void hal_enableIRQs (void){
	ets_intr_unlock();
}

/*
 * put system and CPU in low-power mode, sleep until interrupt.
 */
void hal_sleep (void){
	// not implemented yet
}

/*
 * return 32-bit system time in ticks.
 */
u4_t hal_ticks (void){
	struct rtc_timeval tv;
	rtctime_gettimeofday(&tv);
	u64 time=tv.tv_sec*1000000+tv.tv_usec;
	return (time>>4)&0xFFFFFFFF;
}

/*
 * busy-wait until specified timestamp (in ticks) is reached.
 */
void hal_waitUntil (u4_t time){
	s4_t wait=time-hal_ticks();
	if (wait <= 0) return;
	wait<<=4;
	while (wait > 1000){
		os_delay_us(1000);
		wait-=1000;
	}
	os_delay_us(wait);
}

/*
 * check and rewind timer for target time.
 *   - return 1 if target time is close
 *   - otherwise rewind timer for target time or full period and return 0
 */
u1_t hal_checkTimer (u4_t targettime){
	s4_t wait=targettime-hal_ticks();
	return (wait <= 1);
}

/*
 * perform fatal failure action.
 *   - called by assertions
 *   - action could be HALT or reboot
 */
void hal_failed (void){
	c_printf("hal_failed...");
	system_restart();
}

//===============================================================================================
// KEYS and OS
//===============================================================================================

u8 devKey[16];
void os_getDevKey (xref2u1_t buf){
	strncpy(buf,devKey,16);
}

u8 artEui[8];
void os_getArtEui (xref2u1_t buf){
	strncpy(buf,artEui,8);
}

u8 devEui[8];
void os_getDevEui (xref2u1_t buf){
	strncpy(buf,devEui,8);
}


static int cb_onevent_ref=LUA_NOREF;
void onEvent(ev_t e){
	if (cb_onevent_ref != LUA_NOREF) {
		lua_State *L = lua_getstate();
		lua_rawgeti(L, LUA_REGISTRYINDEX, cb_onevent_ref);
		lua_pushinteger(L, e);
		lua_call(L, 1, 0);
	}
}


//===============================================================================================
// LUA interface
//===============================================================================================

// lmic.onEvent(cb(evt))
static int lmic_onEvent(lua_State *L) {
	luaL_argcheck(L,lua_isfunction(L,1),1,"function expected");
	lua_pushvalue(L, 1);
	int old_ref=cb_onevent_ref;
	cb_onevent_ref = luaL_ref(L, LUA_REGISTRYINDEX);
	if (old_ref != LUA_NOREF && cb_onevent_ref != old_ref) {
		luaL_unref(L,LUA_REGISTRYINDEX,old_ref);
	}
	return 0;
}
// lmic.setupChannel(channel,freq,drmap,band)
static int lmic_setupChannel(lua_State *L) {
	u1_t channel = luaL_checkinteger(L, 1);
	u4_t freq = luaL_checkinteger(L, 2);
	u2_t drmap = luaL_checkinteger(L, 3);
	s1_t band = luaL_checkinteger(L, 4);
	lua_pushboolean(L, LMIC_setupChannel(channel, freq, drmap, band));
	return 1;
}
// lmic.disableChannel(channel)
static int lmic_disableChannel(lua_State *L) {
	u1_t channel = luaL_checkinteger(L, 1);
	LMIC_disableChannel(channel);
	return 0;
}
// lmic.setDrTxpow(dr,txpow)
static int lmic_setDrTxpow(lua_State *L) {
	dr_t dr = luaL_checkinteger(L, 1);
	s1_t txpow = luaL_checkinteger(L, 2);
	LMIC_setDrTxpow(dr,txpow);  // set default/start DR/txpow
	return 0;
}
// lmic.setAdrMode(true/false)
static int lmic_setAdrMode(lua_State *L) {
	bit_t enabled=luaL_checkinteger(L,1);
	LMIC_setAdrMode(enabled);        // set ADR mode (if mobile turn off)
	return 0;
}
// lmic.startJoining()
static int lmic_startJoining(lua_State *L) {
	lua_pushboolean(L, LMIC_startJoining());
	return 1;
}
// lmic.shutdown()
static int lmic_shutdown(lua_State *L) {
	LMIC_shutdown();
	return 0;
}
// lua lmic.init(nss,dio0,dio1,dio2,rst,rxtx)
static int lmic_init(lua_State *L) {
	nss_pin = luaL_checkinteger(L, 1);
	luaL_argcheck(L, platform_gpio_exists(nss_pin), 1, "Invalid pin");
	dio0_pin = luaL_checkinteger(L, 2);
	luaL_argcheck(L, platform_gpio_exists(dio0_pin) && dio0_pin > 0, 2, "Invalid pin");
	dio1_pin = luaL_checkinteger(L, 3);
	luaL_argcheck(L, platform_gpio_exists(dio1_pin) && dio1_pin > 0, 3, "Invalid pin");
	dio2_pin = luaL_optinteger(L, 4,GPIO_ID_NONE);
	if (dio2_pin != GPIO_ID_NONE)
		luaL_argcheck(L, platform_gpio_exists(dio2_pin) && dio2_pin > 0, 4, "Invalid pin");
	rst_pin = luaL_optinteger(L, 5,GPIO_ID_NONE);
	if (rst_pin != GPIO_ID_NONE)
		luaL_argcheck(L, platform_gpio_exists(rst_pin) , 5, "Invalid pin");
	rxtx_pin = luaL_optinteger(L, 6,GPIO_ID_NONE);
	if (rxtx_pin != GPIO_ID_NONE)
		luaL_argcheck(L, platform_gpio_exists(rxtx_pin) , 5, "Invalid pin");
    //
	os_init();
	LMIC_init();
	return 0;
}
// lmic.reset()
static int lmic_reset(lua_State *L) {
	if (dio0_pin != GPIO_ID_NONE) {
		platform_gpio_mode(dio0_pin, PLATFORM_GPIO_INT, PLATFORM_GPIO_FLOAT);
	}
	if (dio1_pin != GPIO_ID_NONE) {
		platform_gpio_mode(dio1_pin, PLATFORM_GPIO_INT, PLATFORM_GPIO_FLOAT);
	}
	if (dio2_pin != GPIO_ID_NONE) {
		platform_gpio_mode(dio2_pin, PLATFORM_GPIO_INT, PLATFORM_GPIO_FLOAT);
	}
	LMIC_reset();
	return 0;
}
// lmic.clrTxData()
static int lmic_clrTxData(lua_State *L) {
	LMIC_clrTxData();
	return 0;
}
// lmic.setTxData()
static int lmic_setTxData(lua_State *L) {
	LMIC_setTxData();
	return 0;
}
u8 dataBuf[256];
// lmic.setTxData2(port,data,confirmed)
static int lmic_setTxData2(lua_State *L) {
	u1_t port=luaL_checkinteger(L,1);
	u1_t confirmed=luaL_optint(L,3,0);
	size_t size;
	const char *dk=luaL_checklstring(L,2,&size);
	strncpy(dataBuf,(xref2u1_t)dk,size);
	lua_pushboolean(L, LMIC_setTxData2(port,dataBuf,size,confirmed));
	return 1;
}
// lmic.sendAlive()
static int lmic_sendAlive(lua_State *L) {
	LMIC_sendAlive();
	return 0;
}
// lmic.enableTracking(tryBcnInfo)
static int lmic_enableTracking(lua_State *L) {
	u1_t tryBcnInfo=luaL_checkinteger(L,1);
	lua_pushboolean(L, LMIC_enableTracking  (tryBcnInfo));
	return 1;
}
// lmic.disableTracking()
static int lmic_disableTracking(lua_State *L) {
	LMIC_disableTracking();
	return 0;
}
// lmic.stopPingable()
static int lmic_stopPingable(lua_State *L) {
	LMIC_stopPingable();
	return 0;
}
// lmic.setPingable(intvExp)
static int lmic_setPingable(lua_State *L) {
	u1_t intvExp=luaL_checkinteger(L,1);
	LMIC_setPingable(intvExp);
	return 0;
}
// lmic.tryRejoin()
static int lmic_tryRejoin(lua_State *L) {
	LMIC_tryRejoin();
	return 0;
}
// lmic.setOTAKeys(devKey,ArtEui,devEui)
static int lmic_setOTAKeys(lua_State *L) {
	size_t size;
	const char *dk=luaL_checklstring(L,1,&size);
	strncpy(devKey,(xref2u1_t)dk,16);
	const char *ae=luaL_checklstring(L,2,&size);
	strncpy(artEui,(xref2u1_t)ae,8);
	const char *de=luaL_checklstring(L,3,&size);
	strncpy(devEui,(xref2u1_t)de,8);
	return 0;
}
// lmic.setSession(netid,devaddr,nwkKey,artKey)
static int lmic_setSession(lua_State *L) {
	u4_t netid=luaL_checkinteger(L,1);
	devaddr_t devaddr=luaL_checkinteger(L,2);
	size_t size;
	const char *nwkKey=luaL_checklstring(L,3,&size);
	const char *artKey=luaL_checklstring(L,4,&size);
	LMIC_setSession (netid,devaddr,(xref2u1_t)nwkKey,(xref2u1_t)artKey);
	return 0;
}
// lmic.setLinkCheckMode(enabled)
static int lmic_setLinkCheckMode(lua_State *L) {
	bit_t enabled=luaL_checkinteger(L,1);
	LMIC_setLinkCheckMode (enabled);
	return 0;
}
static int lmic_debugLevel(lua_State *L) {
	LMIC_DEBUG_LEVEL=luaL_checkinteger(L,1);
	return 0;
}
// Module function map
static const LUA_REG_TYPE lmic_map[] = {
	{ LSTRKEY( "onEvent" ), LFUNCVAL( lmic_onEvent)},
	{ LSTRKEY( "init" ), LFUNCVAL( lmic_init)},
	{ LSTRKEY( "setupChannel" ), LFUNCVAL( lmic_setupChannel)},
	{ LSTRKEY( "disableChannel" ), LFUNCVAL( lmic_disableChannel)},
	{ LSTRKEY( "setDrTxpow" ), LFUNCVAL( lmic_setDrTxpow)},
	{ LSTRKEY( "setAdrMode" ), LFUNCVAL( lmic_setAdrMode)},
	{ LSTRKEY( "startJoining" ), LFUNCVAL( lmic_startJoining)},
	{ LSTRKEY( "shutdown" ), LFUNCVAL( lmic_shutdown)},
	{ LSTRKEY( "reset" ), LFUNCVAL( lmic_reset)},
	{ LSTRKEY( "clrTxData" ), LFUNCVAL( lmic_clrTxData)},
	{ LSTRKEY( "setTxData" ), LFUNCVAL( lmic_setTxData)},
	{ LSTRKEY( "setTxData2" ), LFUNCVAL( lmic_setTxData2)},
	{ LSTRKEY( "sendAlive" ), LFUNCVAL( lmic_sendAlive)},
	{ LSTRKEY( "enableTracking" ), LFUNCVAL( lmic_enableTracking)},
	{ LSTRKEY( "disableTracking" ), LFUNCVAL( lmic_disableTracking)},
	{ LSTRKEY( "stopPingable" ), LFUNCVAL( lmic_stopPingable)},
	{ LSTRKEY( "setPingable" ), LFUNCVAL( lmic_setPingable)},
	{ LSTRKEY( "tryRejoin" ), LFUNCVAL( lmic_tryRejoin)},
	{ LSTRKEY( "setSession" ), LFUNCVAL( lmic_setSession)},
	{ LSTRKEY( "setOTAKeys" ), LFUNCVAL( lmic_setOTAKeys)},
	{ LSTRKEY( "setLinkCheckMode" ), LFUNCVAL( lmic_setLinkCheckMode)},
	{ LSTRKEY( "debugLevel" ), LFUNCVAL( lmic_debugLevel)},

	{ LSTRKEY( "SCAN_TIMEOUT" ), LNUMVAL( EV_SCAN_TIMEOUT ) },
	{ LSTRKEY( "BEACON_FOUND" ), LNUMVAL( EV_BEACON_FOUND ) },
	{ LSTRKEY( "BEACON_MISSED" ), LNUMVAL( EV_BEACON_MISSED ) },
	{ LSTRKEY( "BEACON_TRACKED" ), LNUMVAL( EV_BEACON_TRACKED ) },
	{ LSTRKEY( "JOINING" ), LNUMVAL( EV_JOINING ) },
	{ LSTRKEY( "JOINED" ), LNUMVAL( EV_JOINED ) },
	{ LSTRKEY( "RFU1" ), LNUMVAL( EV_RFU1 ) },
	{ LSTRKEY( "JOIN_FAILED" ), LNUMVAL( EV_JOIN_FAILED ) },
	{ LSTRKEY( "REJOIN_FAILED" ), LNUMVAL( EV_REJOIN_FAILED ) },
	{ LSTRKEY( "TXCOMPLETE" ), LNUMVAL( EV_TXCOMPLETE ) },
	{ LSTRKEY( "LOST_TSYNC" ), LNUMVAL( EV_LOST_TSYNC ) },
	{ LSTRKEY( "RESET" ), LNUMVAL( EV_RESET ) },
	{ LSTRKEY( "RXCOMPLETE" ), LNUMVAL( EV_RXCOMPLETE ) },
	{ LSTRKEY( "LINK_DEAD" ), LNUMVAL( EV_LINK_DEAD ) },
	{ LSTRKEY( "LINK_ALIVE" ), LNUMVAL( EV_LINK_ALIVE ) },

	{ LNILKEY, LNILVAL }
};

NODEMCU_MODULE(LMIC, "lmic", lmic_map, NULL);
