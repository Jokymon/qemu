/*
 * Arm PrimeCell PL061 General Purpose IO based special IOs (bbv)
 *
 * Copyright (c) 2007 CodeSourcery.
 * Written by Paul Brook
 *
 * Copyright (c) 2013 bbv Software Services AG.
 * Written by Mario Konrad
 *
 * This code is licensed under the GPL.
 */

#include "sysbus.h"
#include "lauxlib.h"
#include "lualib.h"

static const uint8_t pl061_id[12] =
  { 0x00, 0x00, 0x00, 0x00, 0x61, 0x10, 0x04, 0x00, 0x0d, 0xf0, 0x05, 0xb1 };

typedef struct {
    SysBusDevice busdev;
    MemoryRegion iomem;
    uint32_t locked;
    uint32_t data;
    uint32_t old_data;
    uint32_t dir;
    uint32_t isense;
    uint32_t ibe;
    uint32_t iev;
    uint32_t im;
    uint32_t istate;
    uint32_t afsel;
    uint32_t dr2r;
    uint32_t dr4r;
    uint32_t dr8r;
    uint32_t odr;
    uint32_t pur;
    uint32_t pdr;
    uint32_t slr;
    uint32_t den;
    uint32_t cr;
    uint32_t float_high;
    uint32_t amsel;
    qemu_irq irq;
    qemu_irq out[8];
    const unsigned char *id;
	lua_State * lua;
	char * lua_script;
} pl061bbv_state;

static Property pl061bbv_properties[] =
{
	DEFINE_PROP_STRING("lua_script", pl061bbv_state, lua_script),
    DEFINE_PROP_END_OF_LIST(),
};

static const VMStateDescription vmstate_pl061bbv = {
    .name = "pl061bbv",
    .version_id = 2,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(locked, pl061bbv_state),
        VMSTATE_UINT32(data, pl061bbv_state),
        VMSTATE_UINT32(old_data, pl061bbv_state),
        VMSTATE_UINT32(dir, pl061bbv_state),
        VMSTATE_UINT32(isense, pl061bbv_state),
        VMSTATE_UINT32(ibe, pl061bbv_state),
        VMSTATE_UINT32(iev, pl061bbv_state),
        VMSTATE_UINT32(im, pl061bbv_state),
        VMSTATE_UINT32(istate, pl061bbv_state),
        VMSTATE_UINT32(afsel, pl061bbv_state),
        VMSTATE_UINT32(dr2r, pl061bbv_state),
        VMSTATE_UINT32(dr4r, pl061bbv_state),
        VMSTATE_UINT32(dr8r, pl061bbv_state),
        VMSTATE_UINT32(odr, pl061bbv_state),
        VMSTATE_UINT32(pur, pl061bbv_state),
        VMSTATE_UINT32(pdr, pl061bbv_state),
        VMSTATE_UINT32(slr, pl061bbv_state),
        VMSTATE_UINT32(den, pl061bbv_state),
        VMSTATE_UINT32(cr, pl061bbv_state),
        VMSTATE_UINT32(float_high, pl061bbv_state),
        VMSTATE_UINT32_V(amsel, pl061bbv_state, 2),
        VMSTATE_END_OF_LIST()
    }
};

static int pl061bbv_lua_init(pl061bbv_state * s)
{
	s->lua = luaL_newstate();
	if (s->lua == NULL) {
		printf("pl061bbv: unable to create lua state\n");
		return -1;
	}

	luaopen_base(s->lua);
	luaopen_table(s->lua);
	luaopen_string(s->lua);
	luaopen_bit32(s->lua);
	luaopen_math(s->lua);
	luaopen_debug(s->lua);

	if (luaL_dofile(s->lua, (s->lua_script) ? s->lua_script : "src/gpio.lua")) {
		printf("pl061bbv: unable to execute script\n");
		return -1;
	}

	return 0;
}

static void pl061bbv_lua_write(pl061bbv_state * s, uint32_t id, uint32_t val)
{
	if (s->lua == NULL) return;

	/* TODO: exception handling */
	lua_getglobal(s->lua, "gpio_set");
	lua_pushinteger(s->lua, id);
	lua_pushinteger(s->lua, val);
	lua_pcall(s->lua, 2, 0, 0);
}

static uint32_t pl061bbv_lua_read(pl061bbv_state * s, uint32_t id)
{
	int rc;

	/* TODO: exception handling */
	lua_getglobal(s->lua, "gpio_get");
	lua_pushinteger(s->lua, id);
	lua_pcall(s->lua, 1, 1, 0);
	rc = lua_tointeger(s->lua, -1);
	lua_pop(s->lua, 1);
	return rc;
}

static void pl061bbv_update(pl061bbv_state *s)
{
    uint8_t changed;
    uint8_t mask;
    uint8_t out;
    int i;

    /* Outputs float high.  */
    /* FIXME: This is board dependent.  */
    out = (s->data & s->dir) | ~s->dir;
    changed = s->old_data ^ out;
    if (!changed)
        return;

    s->old_data = out;
    for (i = 0; i < 8; i++) {
        mask = 1 << i;
        if (changed & mask) {
			pl061bbv_lua_write(s, i, (out & mask) != 0);
            qemu_set_irq(s->out[i], (out & mask) != 0);
        }
    }

    /* FIXME: Implement input interrupts.  */
}

static uint64_t pl061bbv_read(void *opaque, hwaddr offset,
                           unsigned size)
{
	uint32_t i;
    pl061bbv_state *s = (pl061bbv_state *)opaque;

    if (offset >= 0xfd0 && offset < 0x1000) {
		printf("%s:%d: TODO\n", __FUNCTION__, __LINE__);
        return s->id[(offset - 0xfd0) >> 2];
    }
    if (offset < 0x400) {
		if (s->lua) {
			offset >>= 3;
			for (i = 1; offset > 0; offset >>= 1, ++i);
			return pl061bbv_lua_read(s, i);
		} else {
			return s->data & (offset >> 2);
		}
    }
    switch (offset) {
    case 0x400: /* Direction */
        return s->dir;
    case 0x404: /* Interrupt sense */
        return s->isense;
    case 0x408: /* Interrupt both edges */
        return s->ibe;
    case 0x40c: /* Interrupt event */
        return s->iev;
    case 0x410: /* Interrupt mask */
        return s->im;
    case 0x414: /* Raw interrupt status */
        return s->istate;
    case 0x418: /* Masked interrupt status */
        return s->istate | s->im;
    case 0x420: /* Alternate function select */
        return s->afsel;
    case 0x500: /* 2mA drive */
        return s->dr2r;
    case 0x504: /* 4mA drive */
        return s->dr4r;
    case 0x508: /* 8mA drive */
        return s->dr8r;
    case 0x50c: /* Open drain */
        return s->odr;
    case 0x510: /* Pull-up */
        return s->pur;
    case 0x514: /* Pull-down */
        return s->pdr;
    case 0x518: /* Slew rate control */
        return s->slr;
    case 0x51c: /* Digital enable */
        return s->den;
    case 0x520: /* Lock */
        return s->locked;
    case 0x524: /* Commit */
        return s->cr;
    case 0x528: /* Analog mode select */
        return s->amsel;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "pl061bbv_read: Bad offset %x\n", (int)offset);
        return 0;
    }
}

static void pl061bbv_write(void *opaque, hwaddr offset,
                        uint64_t value, unsigned size)
{
    pl061bbv_state *s = (pl061bbv_state *)opaque;
    uint8_t mask;

    if (offset < 0x400) {
        mask = (offset >> 2) & s->dir;
        s->data = (s->data & ~mask) | (value & mask);
        pl061bbv_update(s);
        return;
    }
    switch (offset) {
    case 0x400: /* Direction */
        s->dir = value & 0xff;
        break;
    case 0x404: /* Interrupt sense */
        s->isense = value & 0xff;
        break;
    case 0x408: /* Interrupt both edges */
        s->ibe = value & 0xff;
        break;
    case 0x40c: /* Interrupt event */
        s->iev = value & 0xff;
        break;
    case 0x410: /* Interrupt mask */
        s->im = value & 0xff;
        break;
    case 0x41c: /* Interrupt clear */
        s->istate &= ~value;
        break;
    case 0x420: /* Alternate function select */
        mask = s->cr;
        s->afsel = (s->afsel & ~mask) | (value & mask);
        break;
    case 0x500: /* 2mA drive */
        s->dr2r = value & 0xff;
        break;
    case 0x504: /* 4mA drive */
        s->dr4r = value & 0xff;
        break;
    case 0x508: /* 8mA drive */
        s->dr8r = value & 0xff;
        break;
    case 0x50c: /* Open drain */
        s->odr = value & 0xff;
        break;
    case 0x510: /* Pull-up */
        s->pur = value & 0xff;
        break;
    case 0x514: /* Pull-down */
        s->pdr = value & 0xff;
        break;
    case 0x518: /* Slew rate control */
        s->slr = value & 0xff;
        break;
    case 0x51c: /* Digital enable */
        s->den = value & 0xff;
        break;
    case 0x520: /* Lock */
        s->locked = (value != 0xacce551);
        break;
    case 0x524: /* Commit */
        if (!s->locked)
            s->cr = value & 0xff;
        break;
    case 0x528:
        s->amsel = value & 0xff;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "pl061bbv_write: Bad offset %x\n", (int)offset);
    }
    pl061bbv_update(s);
}

static void pl061bbv_reset(pl061bbv_state *s)
{
  s->locked = 1;
  s->cr = 0xff;
}

static void pl061bbv_set_irq(void * opaque, int irq, int level)
{
    pl061bbv_state *s = (pl061bbv_state *)opaque;
    uint8_t mask;

    mask = 1 << irq;
    if ((s->dir & mask) == 0) {
        s->data &= ~mask;
        if (level)
            s->data |= mask;
        pl061bbv_update(s);
    }
}

static const MemoryRegionOps pl061bbv_ops = {
    .read = pl061bbv_read,
    .write = pl061bbv_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static int pl061bbv_init(SysBusDevice *dev, const unsigned char *id)
{
    pl061bbv_state *s = FROM_SYSBUS(pl061bbv_state, dev);
	printf("pl061bbv: %s : prop:lua_script='%s'\n", __FUNCTION__, s->lua_script);
    s->id = id;

	if (pl061bbv_lua_init(s) < 0) return -1;

    memory_region_init_io(&s->iomem, &pl061bbv_ops, s, "pl061bbv", 0x1000);
    sysbus_init_mmio(dev, &s->iomem);
    sysbus_init_irq(dev, &s->irq);
    qdev_init_gpio_in(&dev->qdev, pl061bbv_set_irq, 8);
    qdev_init_gpio_out(&dev->qdev, s->out, 8);
    pl061bbv_reset(s);
    return 0;
}

static int pl061bbv_init_arm(SysBusDevice *dev)
{
    return pl061bbv_init(dev, pl061_id);
}

static void pl061bbv_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = pl061bbv_init_arm;
    dc->vmsd = &vmstate_pl061bbv;
	dc->props = pl061bbv_properties;
}

static const TypeInfo pl061bbv_info = {
    .name          = "pl061bbv",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(pl061bbv_state),
    .class_init    = pl061bbv_class_init,
};

static void pl061bbv_register_types(void)
{
    type_register_static(&pl061bbv_info);
}

type_init(pl061bbv_register_types)
