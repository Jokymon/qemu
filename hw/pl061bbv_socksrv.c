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
#include <sys/select.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <qemu/thread.h>

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

	int sock;
	struct sockaddr_in addr;
	QemuThread thread_id;
	QemuMutex mtx;
	int client_sock;
	struct sockaddr_in client_addr;
	uint8_t cache;
} pl061bbv_state;

static Property properties[] =
{
    DEFINE_PROP_END_OF_LIST(),
};

static const VMStateDescription vmstate_pl061bbv = {
    .name = "pl061bbv_socksrv",
    .version_id = 2,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(locked,     pl061bbv_state),
        VMSTATE_UINT32(data,       pl061bbv_state),
        VMSTATE_UINT32(old_data,   pl061bbv_state),
        VMSTATE_UINT32(dir,        pl061bbv_state),
        VMSTATE_UINT32(isense,     pl061bbv_state),
        VMSTATE_UINT32(ibe,        pl061bbv_state),
        VMSTATE_UINT32(iev,        pl061bbv_state),
        VMSTATE_UINT32(im,         pl061bbv_state),
        VMSTATE_UINT32(istate,     pl061bbv_state),
        VMSTATE_UINT32(afsel,      pl061bbv_state),
        VMSTATE_UINT32(dr2r,       pl061bbv_state),
        VMSTATE_UINT32(dr4r,       pl061bbv_state),
        VMSTATE_UINT32(dr8r,       pl061bbv_state),
        VMSTATE_UINT32(odr,        pl061bbv_state),
        VMSTATE_UINT32(pur,        pl061bbv_state),
        VMSTATE_UINT32(pdr,        pl061bbv_state),
        VMSTATE_UINT32(slr,        pl061bbv_state),
        VMSTATE_UINT32(den,        pl061bbv_state),
        VMSTATE_UINT32(cr,         pl061bbv_state),
        VMSTATE_UINT32(float_high, pl061bbv_state),
        VMSTATE_UINT32_V(amsel,    pl061bbv_state, 2),
        VMSTATE_END_OF_LIST()
    }
};

static int pl061bbv_sock_write(pl061bbv_state * s, uint8_t data)
{
	qemu_mutex_lock(&s->mtx);
	s->cache = data;
	qemu_mutex_unlock(&s->mtx);
	return 0;
}

static uint64_t pl061bbv_sock_read(pl061bbv_state * s)
{
	uint64_t result;
	qemu_mutex_lock(&s->mtx);
	result = s->cache;
	qemu_mutex_unlock(&s->mtx);
	return result;
}

typedef enum { CMD_INVALID = 0, CMD_WRITE = 1, CMD_READ = 2, CMD_RESULT = 3 } cmd_t;

typedef struct { /* keep data 8 bits to avoid byte order issue, and 8bits are suffient */
	uint8_t cmd;
	uint8_t data;
} msg_t;

static void process_client(pl061bbv_state * s)
{
	int rc;
	msg_t msg;
	fd_set rfds;

	for (;;) {
		FD_ZERO(&rfds);
		FD_SET(s->client_sock, &rfds);
		rc = select(s->client_sock + 1, &rfds, NULL, NULL, NULL);
		if (rc <= 0) return;

		rc = read(s->client_sock, &msg, sizeof(msg));
		if (rc < 0) {
			perror("read");
			return;
		}
		if (rc == 0) return;
		if (!FD_ISSET(s->client_sock, &rfds)) return;

		switch ((cmd_t)msg.cmd) {
			case CMD_INVALID:
				/* ignore */
				break;
			case CMD_WRITE:
				printf("%s: write\n", __PRETTY_FUNCTION__);
				pl061bbv_sock_write(s, msg.data);
				break;
			case CMD_READ:
				msg.cmd = CMD_RESULT;
				msg.data = (uint8_t)pl061bbv_sock_read(s);
				rc = write(s->client_sock, &msg, sizeof(msg));
				if (rc < 0) {
					perror("write");
					return;
				}
				break;
			case CMD_RESULT:
				/* ignore */
				break;
		}
	}
}

static void * thread_func(void * ptr)
{
    pl061bbv_state * s = (pl061bbv_state *)ptr;
	socklen_t client_addr_len;

	if (s->sock < 0) {
		printf("%s: invalid socket\n", __PRETTY_FUNCTION__);
		qemu_thread_exit(NULL);
		return NULL;
	}

	printf("%s: thread started\n", __PRETTY_FUNCTION__);
	for (;;) {
		memset(&s->client_addr, 0, sizeof(struct sockaddr_in));
		client_addr_len = sizeof(struct sockaddr_in);
		s->client_sock = accept(s->sock, (struct sockaddr *)&s->client_addr, &client_addr_len);
		if (s->client_sock < 0) continue;
		printf("%s: connection established\n", __PRETTY_FUNCTION__);
		process_client(s);
		close(s->client_sock);
		s->client_sock = -1;
		printf("%s: connection closed\n", __PRETTY_FUNCTION__);
	}

	qemu_thread_exit(NULL);
	return NULL;
}

static int pl061bbv_sock_init(pl061bbv_state * s)
{
	int rc;
	int tmp;

	if (s->sock >= 0) return 0;

	s->sock = socket(AF_INET, SOCK_STREAM, 0);
	if (s->sock < 0) {
		perror("socket");
		return -1;
	}

	tmp = 1;
	if (setsockopt(s->sock, SOL_SOCKET, SO_REUSEADDR, &tmp, sizeof(tmp)) < 0) {
		perror("setsockopt");
		close(s->sock);
		s->sock = -1;
		return -1;
	}

	s->addr.sin_family = AF_INET;
	s->addr.sin_port = htons(5679);
	s->addr.sin_addr.s_addr = INADDR_ANY;
	rc = bind(s->sock, (const struct sockaddr *)&s->addr, sizeof(s->addr));
	if (rc < 0) {
		perror("bind");
		close(s->sock);
		s->sock = -1;
		return -1;
	}

	if (listen(s->sock, 5) < 0) {
		perror("listen");
		close(s->sock);
		s->sock = -1;
		return -1;
	}

	return 0;
}

static void pl061bbv_update(pl061bbv_state *s)
{
    uint8_t changed;
    uint8_t mask;
    uint8_t out;
    int i;
	msg_t msg;
	int rc;

    /* Outputs float high.  */
    /* FIXME: This is board dependent.  */
    out = (s->data & s->dir) | ~s->dir;
    changed = s->old_data ^ out;
    if (!changed)
        return;

    s->old_data = out;

	pl061bbv_sock_write(s, out); /* TODO: protect 's->client_sock' and 's->cache' with mtx? */
	if (s->client_sock >= 0) {
		msg.cmd = CMD_WRITE;
		msg.data = s->cache;
		rc = write(s->client_sock, &msg, sizeof(msg));
		if (rc < 0) {
			printf("%s: ", __PRETTY_FUNCTION__);
			perror("write");
		}
	}

    for (i = 0; i < 8; i++) {
        mask = 1 << i;
        if (changed & mask) {
            qemu_set_irq(s->out[i], (out & mask) != 0);
        }
    }

    /* FIXME: Implement input interrupts.  */
}

static uint64_t pl061bbv_read(void *opaque, hwaddr offset,
                           unsigned size)
{
    pl061bbv_state *s = (pl061bbv_state *)opaque;

    if (offset >= 0xfd0 && offset < 0x1000) {
		printf("%s:%s:%d: TODO\n", __FILE__, __FUNCTION__, __LINE__);
        return s->id[(offset - 0xfd0) >> 2];
    }
    if (offset < 0x400) {
		return pl061bbv_sock_read(s) & (offset >> 2);
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
                      "%s: Bad offset %x\n", __FUNCTION__, (int)offset);
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
                      "%s: Bad offset %x\n", __FUNCTION__, (int)offset);
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
    s->id = id;

	s->cache = 0xff;
	s->sock = -1;
	s->client_sock = -1;
	if (pl061bbv_sock_init(s) < 0) {
		printf("%s: unable to init listener socket\n", __FUNCTION__);
		return -1;
	}
	qemu_mutex_init(&s->mtx);
	qemu_thread_create(&s->thread_id, thread_func, s, QEMU_THREAD_JOINABLE);

    memory_region_init_io(&s->iomem, &pl061bbv_ops, s, "pl061bbv_socksrv", 0x1000);
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
	dc->props = properties;
}

static const TypeInfo pl061bbv_info = {
    .name          = "pl061bbv_socksrv",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(pl061bbv_state),
    .class_init    = pl061bbv_class_init,
};

static void pl061bbv_register_types(void)
{
    type_register_static(&pl061bbv_info);
}

type_init(pl061bbv_register_types)
