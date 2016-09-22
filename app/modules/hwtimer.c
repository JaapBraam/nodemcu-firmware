#include "module.h"
#include "lauxlib.h"
#include "platform.h"
#include "c_types.h"
#include "user_interface.h"
#include "hw_timer.h"

static const os_param_t HW_TIMER_OWNER = 0x68775f74; // hw_t

static int cb_interval_ref = LUA_NOREF;

static u32 count;
static void ICACHE_RAM_ATTR interval_async_cb(os_param_t p) {
	count+=1;
	if (cb_interval_ref != LUA_NOREF) {
		lua_State *L = lua_getstate();
		lua_rawgeti(L, LUA_REGISTRYINDEX, cb_interval_ref);
		lua_call(L, 0, 0);
	}
}

// hw_tmr.interval(microseconds,callback)
int hwtimer_interval(lua_State *L) {

	uint32_t microseconds = luaL_checkinteger(L, 1);

	if (lua_isfunction(L,2)){
		lua_pushvalue(L, 2);
		cb_interval_ref = luaL_ref(L, LUA_REGISTRYINDEX);
	}
	if (!platform_hw_timer_init(HW_TIMER_OWNER, FRC1_SOURCE, true)) {
		luaL_error(L, "Unable to initialize timer");
	}
	if (!platform_hw_timer_set_func(HW_TIMER_OWNER, interval_async_cb,0)){
		luaL_error(L, "Unable to set function");
	}
	if (!platform_hw_timer_arm_us(HW_TIMER_OWNER, microseconds)){
		luaL_error(L, "Unable to arm timer");
	}

	return 0;
}
// hw_tmr.interval(microseconds,callback)
int hwtimer_count(lua_State *L) {
	lua_pushinteger(L,count);
	count=0;
	return 1;
}
static const LUA_REG_TYPE hwtimer_map[] = {
	{ LSTRKEY("interval"), LFUNCVAL(hwtimer_interval) },
	{ LSTRKEY("count"), LFUNCVAL(hwtimer_count) },
	{ LNILKEY, LNILVAL }
};

NODEMCU_MODULE(HWTIMER, "hwtimer", hwtimer_map, NULL);
