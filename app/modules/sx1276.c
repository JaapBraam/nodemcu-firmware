/*
The MIT License (MIT)

Copyright (c) 2017 Jaap Braam

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "module.h"
#include "lauxlib.h"
#include "lmem.h"
#include "platform.h"
#include "c_types.h"
#include "c_string.h"
#include "cpu_esp8266.h"
#include "gpio.h"
#include "sx1276.h"
#include "hw_timer.h"

#ifdef LUA_USE_MODULES_SX1276
#if !defined(GPIO_INTERRUPT_ENABLE) || !defined(GPIO_INTERRUPT_HOOK_ENABLE)
#error Must have GPIO_INTERRUPT and GPIO_INTERRUPT_HOOK if using SX1276 module
#endif
#endif

static u32 timestamp;

static u8 nss_pin;
static u8 dio0_pin;
static u32 dio0_mask;
static u8 dio1_pin;
static u32 dio1_mask;

static void write(u8 reg, u8 val) {
	platform_gpio_write(nss_pin, PLATFORM_GPIO_LOW);
	platform_spi_send(1, 8, reg | 0x80);
	platform_spi_send(1, 8, val);
	platform_gpio_write(nss_pin, PLATFORM_GPIO_HIGH);
}

static u8 read(u8 reg) {
	platform_gpio_write(nss_pin, PLATFORM_GPIO_LOW);
	platform_spi_send(1, 8, reg & 0x7F);
	spi_data_type b = platform_spi_send_recv(1, 8, 0);
	platform_gpio_write(nss_pin, PLATFORM_GPIO_HIGH);
	return b & 0xFF;
}

static u8 rxbuf[256];
static void readBuffer(u8 reg, u8 size) {
	platform_gpio_write(nss_pin, PLATFORM_GPIO_LOW);
	platform_spi_send(1, 8, reg & 0x7F);
	u8 i;
	for (i = 0; i < size; i++) {
		rxbuf[i] = platform_spi_send_recv(1, 8, 0);
	}
	platform_gpio_write(nss_pin, PLATFORM_GPIO_HIGH);
}

static void loraOpMode(u8 mode) {
	write(RegOpMode, mode | OPMODE_LORA);
}

static u32 frf;
static void setFreq(u32 freq) {
	frf = (freq / 1000 << 11) / 125;
	write(RegFrfMsb, (frf >> 16) & 0xFF);
	write(RegFrfMid, (frf >> 8) & 0xFF);
	write(RegFrfLsb, (frf >> 0) & 0xFF);
}

volatile u16 symbol_length;
static void setDR(u8 sf, u8 bw, u8 cr, u8 crc, u8 iiq) {
	write(LORARegModemConfig1, bw | cr);
	write(LORARegModemConfig2, sf | crc);
	write(LORARegModemConfig3, (sf < MC2_SF11) ? 0x04 : 0x0C);
	write(LORARegSymbTimeoutLsb, (sf < MC2_SF10) ? 0x08 : 0x05);
	write(LORARegInvertIQ, iiq);
	symbol_length = (bw == MC1_BW_125) ? 8 << (sf >> 4) : (bw == MC1_BW_250) ? 4 << (sf >> 4) : 2 << (sf >> 4);
}

static int cb_rxdone_ref = LUA_NOREF;
static int cb_rxtimeout_ref = LUA_NOREF;
static int cb_caddone_ref = LUA_NOREF;
static int cb_caddetected_ref = LUA_NOREF;
static int cb_txdone_ref = LUA_NOREF;

static void cad_detected(u32 tmst) {
	write(LORARegIrqFlags, 0xff);
	if (cb_caddetected_ref != LUA_NOREF) {
		lua_State *L = lua_getstate();
		lua_rawgeti(L, LUA_REGISTRYINDEX, cb_caddetected_ref);
		lua_pushinteger(L, tmst);
		lua_call(L, 1, 0);
	}
}
static void cad_done(u32 tmst) {
	write(LORARegIrqFlags, 0xff);
	if (cb_caddone_ref != LUA_NOREF) {
		lua_State *L = lua_getstate();
		lua_rawgeti(L, LUA_REGISTRYINDEX, cb_caddone_ref);
		lua_pushinteger(L, tmst);
		lua_call(L, 1, 0);
	}
}

static void rx_timeout(u32 tmst) {
	write(LORARegIrqFlags, 0xff);
	if (cb_rxtimeout_ref != LUA_NOREF) {
		lua_State *L = lua_getstate();
		lua_rawgeti(L, LUA_REGISTRYINDEX, cb_rxtimeout_ref);
		lua_pushinteger(L, tmst);
		lua_call(L, 1, 0);
	}
}

static void rx_done(u32 tmst) {
	write(LORARegIrqFlags, IRQ_LORA_RXDONE_MASK);
	u8 irqflags = read(LORARegIrqFlags);
	s8 stat = 0;
	if (irqflags & IRQ_LORA_CRCERR_MASK == IRQ_LORA_CRCERR_MASK) {
		write(LORARegIrqFlags, IRQ_LORA_CRCERR_MASK);
		stat = -1;
		if (cb_rxdone_ref != LUA_NOREF) {
			lua_State *L = lua_getstate();
			lua_rawgeti(L, LUA_REGISTRYINDEX, cb_rxdone_ref);
			lua_createtable(L, 0, 2);
			lua_pushstring(L, "tmst");
			lua_pushinteger(L, tmst);
			lua_settable(L, -3);

			lua_pushstring(L, "stat");
			lua_pushinteger(L, stat);
			lua_settable(L, -3);

			lua_call(L, 1, 0);
		}
	} else {
		u8 rhc = read(LORARegHopChannel);
		if (rhc & 0x40 == 0x40) {
			stat = 1;
		} else {
			stat = 0;
		}
		u8 chan = read(LORARegHopChannel) & 0x1F;
		u32 freq = (125000 * read(RegFrfMsb) << 5) + (125000 * read(RegFrfMid) >> 3) + (125000 * read(RegFrfLsb) >> 11);
		s16 rssi = -157 + read(LORARegPktRssiValue);
		s8 snr = read(LORARegPktSnrValue);

		u8 sf = read(LORARegModemConfig2) & 0xF0;
		u8 bw = read(LORARegModemConfig1) & 0xF0;
		u8 cr = read(LORARegModemConfig1) & 0x0E;

		u8 curr = read(LORARegFifoRxCurrentAddr);
		u8 size = read(LORARegRxNbBytes);
		write(LORARegFifoAddrPtr, curr);
		readBuffer(0x00, size);
		write(LORARegIrqFlags, 0xff);
		// callback
		if (cb_rxdone_ref != LUA_NOREF) {
			lua_State *L = lua_getstate();
			lua_rawgeti(L, LUA_REGISTRYINDEX, cb_rxdone_ref);
			lua_createtable(L, 0, 10);
			lua_pushstring(L, "tmst");
			lua_pushinteger(L, tmst);
			lua_settable(L, -3);

			lua_pushstring(L, "stat");
			lua_pushinteger(L, stat);
			lua_settable(L, -3);

			lua_pushstring(L, "chan");
			lua_pushinteger(L, chan);
			lua_settable(L, -3);

			lua_pushstring(L, "freq");
			lua_pushfstring(L, "%d.%d", freq / 1000000, ((freq + 500) / 1000) % 1000);
			lua_settable(L, -3);

			lua_pushstring(L, "rssi");
			lua_pushinteger(L, rssi);
			lua_settable(L, -3);

			lua_pushstring(L, "snr");
			lua_pushfstring(L, "%d.%d", snr / 4, (snr * 25) % 100);
			//lua_pushinteger(L, snr);
			lua_settable(L, -3);

			lua_pushstring(L, "datr");
			lua_pushfstring(L, "SF%dBW%d", sf >> 4, (bw == MC1_BW_125) ? 125 : (bw == MC1_BW_250) ? 250 : 500);
			lua_settable(L, -3);

			lua_pushstring(L, "codr");
			lua_pushfstring(L, "4/%d", 4 + (cr >> 1));
			lua_settable(L, -3);

			lua_pushstring(L, "size");
			lua_pushinteger(L, size);
			lua_settable(L, -3);

			lua_pushstring(L, "data");
			lua_pushlstring(L, rxbuf, size);
			lua_settable(L, -3);

			lua_call(L, 1, 0);
		}
	}
}

static void tx_done(u32 tmst) {
	write(LORARegIrqFlags, 0xff);
	if (cb_txdone_ref != LUA_NOREF) {
		lua_State *L = lua_getstate();
		lua_rawgeti(L, LUA_REGISTRYINDEX, cb_txdone_ref);
		lua_pushinteger(L, tmst);
		lua_call(L, 1, 0);
	}
}

static void sleep() {
	loraOpMode(OPMODE_SLEEP);
}

static void standby() {
	loraOpMode(OPMODE_STANDBY);
}

volatile u32 cad_start;
static void cad() {
	loraOpMode(OPMODE_CAD);
	cad_start = system_get_time();
	u8 mask = (IRQ_LORA_CDDONE_MASK | IRQ_LORA_CDDETD_MASK);
	write(LORARegIrqFlagsMask, ~mask);
	write(RegDioMapping1, MAP_DIO0_LORA_CADONE | MAP_DIO1_LORA_CADDET);
}

static void rx() {
	loraOpMode(OPMODE_RX);
	u8 mask = (IRQ_LORA_RXDONE_MASK | IRQ_LORA_CRCERR_MASK);
	write(LORARegIrqFlagsMask, ~mask);
	write(RegDioMapping1, MAP_DIO0_LORA_RXDONE | MAP_DIO1_LORA_NOP);
}

static void rx_single() {
	loraOpMode(OPMODE_RX_SINGLE);
	u8 mask = (IRQ_LORA_RXTOUT_MASK | IRQ_LORA_RXDONE_MASK | IRQ_LORA_CRCERR_MASK);
	write(LORARegIrqFlagsMask, ~mask);
	write(RegDioMapping1, MAP_DIO0_LORA_RXDONE | MAP_DIO1_LORA_RXTOUT);
}

static void tx() {
	loraOpMode(OPMODE_TX);
	u8 mask = (IRQ_LORA_TXDONE_MASK);
	write(LORARegIrqFlagsMask, ~mask);
	write(RegDioMapping1, MAP_DIO0_LORA_TXDONE | MAP_DIO1_LORA_NOP);
}

static void dio1Handler() {
	u8 flags = read(LORARegIrqFlags);
	if (flags & IRQ_LORA_CDDETD_MASK) {
		cad_detected(timestamp);
	};
	if (flags & IRQ_LORA_RXTOUT_MASK) {
		rx_timeout(timestamp);
	};
}

static void dio0Handler() {
	u8 flags = read(LORARegIrqFlags);
	if (flags & IRQ_LORA_CDDONE_MASK) {
		cad_done(timestamp);
	};
	if (flags & IRQ_LORA_RXDONE_MASK) {
		rx_done(timestamp);
	};
	if (flags & IRQ_LORA_TXDONE_MASK) {
		tx_done(timestamp);
	}
}

static uint32_t dio_hook(uint32_t ret_gpio_status) {
	timestamp = system_get_time();
	if (ret_gpio_status & dio1_mask) {
		dio1Handler();
		if (dio0_pin == dio1_pin) {
			dio0Handler();
		}
		GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, ret_gpio_status & dio1_mask);
		ret_gpio_status &= ~dio1_mask;
	}
	if (dio0_pin != dio1_pin) {
		if (ret_gpio_status & dio0_mask) {
			dio0Handler();
			GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, ret_gpio_status & dio0_mask);
			ret_gpio_status &= ~dio0_mask;
		}
	}
	return ret_gpio_status;
}

static void setupIO(u8 dio0, u8 dio1) {

	if (dio0 == 0xFF)
		return;

// setup i/o
	dio0_pin = dio0;
	dio0_mask = BIT(pin_num[dio0]);
	dio1_pin = dio1;
	dio1_mask = BIT(pin_num[dio1]);

	platform_gpio_mode(dio0_pin, PLATFORM_GPIO_INT, PLATFORM_GPIO_PULLUP);
	platform_gpio_intr_init(dio0_pin, GPIO_PIN_INTR_POSEDGE);

	if (dio0 != dio1) {
		platform_gpio_mode(dio1_pin, PLATFORM_GPIO_INT, PLATFORM_GPIO_PULLUP);
		platform_gpio_intr_init(dio1_pin, GPIO_PIN_INTR_POSEDGE);
	}

	platform_gpio_register_intr_hook(dio0_mask | dio1_mask, dio_hook);
}

static void setupSPI(u8 nss) {
	nss_pin = nss;
	platform_gpio_mode(nss_pin, PLATFORM_GPIO_OUTPUT, PLATFORM_GPIO_FLOAT);
	platform_gpio_write(nss_pin, 1);
	platform_spi_setup(1, PLATFORM_SPI_MASTER, PLATFORM_SPI_CPOL_LOW,
	PLATFORM_SPI_CPHA_LOW, 3);
}

typedef enum {
	idle_state, start_state, scan_state, cad_state, rx_single_state, tx_state
} State;
volatile State state = idle_state;

static const os_param_t SX1276_TIMER_OWNER = 0x4c6f5261; // LoRa

#define SAMPLES 15
#define CAD_WAIT 240+SAMPLES*8
#define RSSI_TRESHOLD 46
#define CHANNELS 3

volatile u32 freqs[] = { 868100000, 868300000, 868500000, 867100000, 867300000, 867500000, 867700000, 867900000 };
volatile u8 hopper_freq = 0;
volatile u8 flags, hopper_ch, hopper_best, rssi, hopper_cad, modemstat, cads;

static void ICACHE_RAM_ATTR hopper(os_param_t p) {
	timestamp = system_get_time();
	u16 delay = 10;
	switch (state) {
	case start_state:
		setFreq(freqs[hopper_freq]);
		setDR(MC2_SF7, MC1_BW_125, MC1_CR_4_5, MC2_RX_PAYLOAD_CRCON, LORA_IQ_NORMAL);
		cad();
		hopper_cad = MC2_SF7 | MC2_RX_PAYLOAD_CRCON;
		hopper_ch = 0xff;
		hopper_best = 0;
		delay = CAD_WAIT;
		state = scan_state;
		break;
	case scan_state:
		rssi = read(LORARegRssiValue);
		if (rssi > RSSI_TRESHOLD) {
			if (rssi > hopper_best) {
				hopper_best = rssi;
				hopper_ch = hopper_freq;
			}
		}
		standby();
		hopper_freq = (hopper_freq + 1) % CHANNELS;
		setFreq(freqs[hopper_freq]);
		cad();
		delay = CAD_WAIT;
		if (hopper_ch == hopper_freq) {
			state = cad_state;
		}
		break;
	case cad_state:
		flags = read(LORARegIrqFlags);
		if (flags & IRQ_LORA_CDDETD_MASK) {
			rx_single();
			state = rx_single_state;
			delay = 2000;
			cad_detected(timestamp);
		} else if (flags & IRQ_LORA_CDDONE_MASK) {
			if (hopper_cad < MC2_SF12) {
				hopper_cad += 0x10;
				write(LORARegModemConfig2, hopper_cad);
				if (hopper_cad & MC2_SF11 == MC2_SF11) {
					write(LORARegModemConfig3, MC3_AGCAUTO | MC3_LOW_DATA_RATE_OPTIMIZE);
				}
				if (hopper_cad & MC2_SF10 == MC2_SF10) {
					write(LORARegSymbTimeoutLsb, 0x05);
				}
				cad();
				delay = CAD_WAIT;
				symbol_length <<= 1;
				cad_done(timestamp);
			} else {
				state = start_state;
				cad_done(timestamp);
			}
		} else {
			if (timestamp < cad_start + symbol_length) { // marge 240 us starting up
				rssi = read(LORARegRssiValue);
				if (rssi < RSSI_TRESHOLD - 3) {
					state = start_state;
					cad_done(timestamp);
				} else {
					delay = 128;
				}
			}
		}
		break;
	case rx_single_state:
		flags = read(LORARegIrqFlags);
		if (flags & IRQ_LORA_RXTOUT_MASK) {
			state = start_state;
			rx_timeout(timestamp);
		} else if (flags & IRQ_LORA_RXDONE_MASK) {
			state = start_state;
			rx_done(timestamp);
		} else {
			rssi = read(LORARegRssiValue);
			if (rssi < RSSI_TRESHOLD - 3) {
				state = start_state;
				rx_timeout(timestamp);
			} else {
				delay = 128;
			}
		}
		break;
	case tx_state:
		flags = read(LORARegIrqFlags);
		if (flags & IRQ_LORA_TXDONE_MASK) {
			state = start_state;
			tx_done(timestamp);
		}
		break;
	case idle_state:
		delay = 0;
	}
	if (delay) {
		platform_hw_timer_arm_us(SX1276_TIMER_OWNER, delay);
	} else {
		platform_hw_timer_close(SX1276_TIMER_OWNER);
	}
}

static void startHopper() {
	state = start_state;
	platform_hw_timer_init(SX1276_TIMER_OWNER, FRC1_SOURCE, false);
	platform_hw_timer_set_func(SX1276_TIMER_OWNER, hopper, 0);
	platform_hw_timer_arm_us(SX1276_TIMER_OWNER, 10);
}

static void stopHopper() {
	state = idle_state;
}

static int sx1276_on(lua_State *L) {
	int *refptr = NULL;
	const char *name = luaL_checkstring(L, 1);
	if (!name)
		return luaL_error(L, "need callback name");
	if (strcmp("cadDetected", name) == 0) {
		refptr = &cb_caddetected_ref;
	}
	if (strcmp("cadDone", name) == 0) {
		refptr = &cb_caddone_ref;
	}
	if (strcmp("rxDone", name) == 0) {
		refptr = &cb_rxdone_ref;
	}
	if (strcmp("rxTimeout", name) == 0) {
		refptr = &cb_rxtimeout_ref;
	}
	if (strcmp("txDone", name) == 0) {
		refptr = &cb_txdone_ref;
	}
	if (refptr == NULL)
		return luaL_error(L, "invalid callback name");
	if (lua_isfunction(L, 2) || lua_islightfunction(L, 2)) {
		lua_pushvalue(L, 2);
		luaL_unref(L, LUA_REGISTRYINDEX, *refptr);
		*refptr = luaL_ref(L, LUA_REGISTRYINDEX);
	} else if (lua_isnil(L, 2)) {
		luaL_unref(L, LUA_REGISTRYINDEX, *refptr);
		*refptr = LUA_NOREF;
	} else {
		return luaL_error(L, "invalid callback function");
	}
	return 0;
}

static int sx1276_sleep(lua_State *L) {
	sleep();
	return 0;
}
static int sx1276_standby(lua_State *L) {
	standby();
	return 0;
}
static int sx1276_cad(lua_State *L) {
	cad();
	return 0;
}

static int sx1276_rx(lua_State *L) {
	rx();
	return 0;
}

static int sx1276_rxSingle(lua_State *L) {
	rx_single();
	return 0;
}

static int sx1276_tx(lua_State *L) {
	tx();
	return 0;
}

//sx1276.frequency(freqHz)
static int sx1276_frequency(lua_State *L) {
	u32 freqHz = luaL_checkinteger(L, 1);
	setFreq(freqHz);
	return 0;
}

//sx1276.dataRate(sf,bw,cr,crc,iiq,powe)
static int sx1276_dataRate(lua_State *L) {
	u8 sf = luaL_optinteger(L, 1, MC2_SF7);
	luaL_argcheck(L,
			(sf==MC2_SF6 || sf==MC2_SF7|| sf==MC2_SF8|| sf==MC2_SF9|| sf==MC2_SF10|| sf==MC2_SF11|| sf==MC2_SF12|| sf==MC2_FSK),
			1, "invalid SF");
	u8 bw = luaL_optinteger(L, 2, MC1_BW_125);
	luaL_argcheck(L, (bw==MC1_BW_125 || bw==MC1_BW_250|| bw==MC1_BW_500), 2, "invalid BW");
	u8 cr = luaL_optinteger(L, 3, MC1_CR_4_5);
	luaL_argcheck(L, (cr==MC1_CR_4_5 || cr==MC1_CR_4_6|| cr==MC1_CR_4_7|| cr==MC1_CR_4_8), 3, "invalid CR");
	u8 crc = luaL_optinteger(L, 4, MC2_RX_PAYLOAD_CRCON);
	luaL_argcheck(L, (crc==MC2_RX_PAYLOAD_CRCOFF ||crc==MC2_RX_PAYLOAD_CRCON), 4, "invalid CRC");
	u8 iiq = luaL_optinteger(L, 5, LORA_IQ_NORMAL);
	luaL_argcheck(L, (iiq==LORA_IQ_NORMAL ||iiq==LORA_IQ_INVERTED), 5, "invalid IQ");
	setDR(sf, bw, cr, crc, iiq);

	return 0;
}

static int sx1276_init(lua_State *L) {
	u8 version = read(RegVersion);
	if (version == 0x12) {
		loraOpMode(OPMODE_SLEEP);
		write(LORARegSyncWord, LORA_MAC_PREAMBLE);
		write(RegLna, 0x23);
		write(LORARegPayloadMaxLength, 0x23);
		write(LORARegPayloadLength, 0x40);
		write(LORARegPreambleLsb, 0x08);
		write(RegPaRamp, (read(RegPaRamp) & 0xF0) | 0x08);
		write(RegPaDac, read(RegPaDac) | 0x04);
		write(LORARegDetectionThreshold, 0x0A);
	}
	lua_pushboolean(L, version == 0x12);
	return 1;
}

static int sx1276_hopper(lua_State *L) {
	startHopper();
	return 0;
}

static int sx1276_version(lua_State *L) {
	u8 version = read(RegVersion);
	lua_pushinteger(L, (lua_Integer) (version));
	return 1;
}

static int sx1276_rssi(lua_State *L) {
	u8 rssi = read(LORARegRssiValue);
	lua_pushinteger(L, (lua_Integer) (rssi));
	return 1;
}

static int sx1276_setup(lua_State *L) {
	unsigned nss = luaL_checkinteger(L, 1);
	luaL_argcheck(L, platform_gpio_exists(nss), 2, "Unknown pin");
	unsigned dio0 = luaL_optinteger(L, 2, 0xFF);
	if (dio0 != 0xFF) {
		luaL_argcheck(L, platform_gpio_exists(dio0) && dio0 > 0, 2, "Invalid interrupt pin");
		MOD_CHECK_ID(gpio, dio0);
	}
	unsigned dio1 = luaL_optinteger(L, 3, 0xFF);
	if (dio1 != 0xFF) {
		luaL_argcheck(L, platform_gpio_exists(dio1) && dio1 > 0, 3, "Invalid interrupt pin");
		MOD_CHECK_ID(gpio, dio1);
	} else {
		dio1 = dio0;
	}

	MOD_CHECK_ID(gpio, nss);

	setupIO(dio0, dio1);
	setupSPI(nss);

	return 0;
}

// Module function map
static const LUA_REG_TYPE sx1276_map[] = {
	{ LSTRKEY("setup"), LFUNCVAL(sx1276_setup) },
	{ LSTRKEY("version"), LFUNCVAL(	sx1276_version) },
	{ LSTRKEY("rssi"), LFUNCVAL(sx1276_rssi) },
	{ LSTRKEY("init"), LFUNCVAL(sx1276_init) },
	{ LSTRKEY("dataRate"), LFUNCVAL(sx1276_dataRate) },
	{ LSTRKEY("frequency"), LFUNCVAL(sx1276_frequency) },
	{ LSTRKEY("hopper"),LFUNCVAL(sx1276_hopper) },
	{ LSTRKEY("sleep"), LFUNCVAL(sx1276_sleep) },
	{ LSTRKEY("standby"), LFUNCVAL(	sx1276_standby) },
	{ LSTRKEY("cad"), LFUNCVAL(sx1276_cad) },
	{ LSTRKEY("rx"), LFUNCVAL(sx1276_rx) },
	{ LSTRKEY("rxSingle"), LFUNCVAL(sx1276_rxSingle) },
	{ LSTRKEY("tx"), LFUNCVAL(sx1276_tx) },
	{ LSTRKEY("on"), LFUNCVAL(sx1276_on) },
	{ LSTRKEY("SF6"), LNUMVAL( MC2_SF6) },
	{ LSTRKEY("SF7"), LNUMVAL( MC2_SF7) },
	{ LSTRKEY("SF8"), LNUMVAL( MC2_SF8) },
	{ LSTRKEY("SF9"), LNUMVAL( MC2_SF9) },
	{ LSTRKEY("SF10"), LNUMVAL( MC2_SF10) },
	{ LSTRKEY("SF11"), LNUMVAL( MC2_SF11) },
	{ LSTRKEY("SF12"), LNUMVAL( MC2_SF12) },
	{ LSTRKEY("FSK"), LNUMVAL( MC2_FSK) },
	{ LSTRKEY("BW125"),	LNUMVAL(MC1_BW_125) },
	{ LSTRKEY("BW250"), LNUMVAL( MC1_BW_250) },
	{ LSTRKEY("BW500"), LNUMVAL( MC1_BW_500) },
	{ LSTRKEY("CR4_5"),	LNUMVAL(MC1_CR_4_5) },
	{ LSTRKEY("CR4_6"), LNUMVAL( MC1_CR_4_6) },
	{ LSTRKEY("CR4_7"), LNUMVAL( MC1_CR_4_7) },
	{ LSTRKEY("CR4_8"),	LNUMVAL(MC1_CR_4_8) },
	{ LSTRKEY("CRC"), LNUMVAL( MC2_RX_PAYLOAD_CRCON) },
	{ LSTRKEY("NOCRC"), LNUMVAL(MC2_RX_PAYLOAD_CRCOFF) },
	{ LSTRKEY("IQ"), LNUMVAL( LORA_IQ_NORMAL) },
	{ LSTRKEY("IQ_INV"), LNUMVAL(LORA_IQ_INVERTED) },
	{ LNILKEY, LNILVAL } };

NODEMCU_MODULE(SX1276, "sx1276", sx1276_map, NULL);
