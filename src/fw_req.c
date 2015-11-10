#include <sys/ioctl.h>
#include "fw_req.h"
#include "internal.h"

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

/**
 * SECTION:fw_req
 * @Title: HinawaFwReq
 * @Short_description: A transaction executor to a FireWire unit
 *
 * A HinawaFwReq supports three types of transactions in IEEE 1212:
 *  - read
 *  - write
 *  - lock
 *
 * Any of transaction frames should be aligned to 32bit (quadlet).
 * This class is an application of Linux FireWire subsystem. All of operations
 * utilize ioctl(2) with subsystem specific request commands.
 */

/* For error handling. */
G_DEFINE_QUARK("HinawaFwReq", hinawa_fw_req)
#define raise(exception, errno)						\
	g_set_error(exception, hinawa_fw_req_quark(), errno,		\
		    "%d: %s", __LINE__, strerror(errno))

enum fw_req_type {
	FW_REQ_TYPE_WRITE = 0,
	FW_REQ_TYPE_READ,
	FW_REQ_TYPE_COMPARE_SWAP,
};

/* NOTE: This object has no properties and no signals. */
struct _HinawaFwReqPrivate {
	guint32 *frame;

	GCond cond;
};
G_DEFINE_TYPE_WITH_PRIVATE(HinawaFwReq, hinawa_fw_req, G_TYPE_OBJECT)

static void hinawa_fw_req_class_init(HinawaFwReqClass *klass)
{
	return;
}

static void hinawa_fw_req_init(HinawaFwReq *self)
{
	return;
}

static void fw_req_transact(HinawaFwReq *self, HinawaFwUnit *unit,
			    enum fw_req_type type, guint64 addr,
			    guint32 frame[], guint len, GError **exception)
{
	HinawaFwReqPrivate *priv = hinawa_fw_req_get_instance_private(self);
	struct fw_cdev_send_request req = {0};
	int tcode;

	guint64 generation;

	guint64 expiration;
	GMutex lock;

	unsigned int i;

	/* From host order to be32. */
	for (i = 0; i < len; i++)
		frame[i] = GUINT32_TO_BE(frame[i]);

	/* Setup a private structure. */
	if (type == FW_REQ_TYPE_READ) {
		priv->frame = frame;
		req.data = (guint64)NULL;
		if (len == 1)
			tcode = TCODE_READ_QUADLET_REQUEST;
		else
			tcode = TCODE_READ_BLOCK_REQUEST;
	} else if (type == FW_REQ_TYPE_WRITE) {
		priv->frame = NULL;
		req.data = (guint64)frame;
		if (len == 1)
			tcode = TCODE_WRITE_QUADLET_REQUEST;
		else
			tcode = TCODE_WRITE_BLOCK_REQUEST;
	} else if (type == FW_REQ_TYPE_COMPARE_SWAP && (len == 2 || len == 4)) {
			priv->frame = NULL;
			req.data = (guint64)frame;
			tcode = TCODE_LOCK_COMPARE_SWAP;
	} else {
		raise(exception, EINVAL);
		return;
	}

	/* Get unit properties. */
	g_object_get(G_OBJECT(unit), "generation", &generation, NULL);

	/* Setup a transaction structure. */
	req.tcode = tcode;
	req.length = len * sizeof(guint32);
	req.offset = addr;
	req.closure = (guint64)self;
	req.generation = generation;

	/* NOTE: Timeout is 10 milli-seconds. */
	expiration = g_get_monotonic_time() + 10 * G_TIME_SPAN_MILLISECOND;
	g_cond_init(&priv->cond);
	g_mutex_init(&lock);

	/* Send this transaction. */
	hinawa_fw_unit_ioctl(unit, FW_CDEV_IOC_SEND_REQUEST, &req, exception);
	/* Wait for a response with timeout, waken by a response handler. */
	if (*exception == NULL) {
		g_mutex_lock(&lock);
		if (!g_cond_wait_until(&priv->cond, &lock, expiration))
			raise(exception, ETIMEDOUT);
		g_mutex_unlock(&lock);
	}

	/* From be32 to host endian. */
	for (i = 0; i < len; i++)
		frame[i] = GUINT32_FROM_BE(frame[i]);

	g_mutex_clear(&lock);
	g_cond_clear(&priv->cond);
}

/**
 * hinawa_fw_req_write:
 * @self: A #HinawaFwReq
 * @unit: A #HinawaFwUnit
 * @addr: A destination address of target device
 * @frame: (element-type guint32) (array length=len) (in): a 32bit array
 * @len: (in): the number of elements in the array
 * @exception: A #GError
 *
 * Execute write transactions to the given unit.
 */
void hinawa_fw_req_write(HinawaFwReq *self, HinawaFwUnit *unit, guint64 addr,
			 guint32 frame[], guint len, GError **exception)
{
	g_return_if_fail(HINAWA_IS_FW_REQ(self));

	if (frame == NULL || len == 0) {
		raise(exception, EINVAL);
		return;
	}

	g_object_ref(unit);
	fw_req_transact(self, unit, FW_REQ_TYPE_WRITE, addr, frame, len,
			exception);
	g_object_unref(unit);
}

/**
 * hinawa_fw_req_read:
 * @self: A #HinawaFwReq
 * @unit: A #HinawaFwUnit
 * @addr: A destination address of target device
 * @frame: (element-type guint32) (array length=len) (out caller-allocates): a 32bit array
 * @len: (in): the number of elements in the array
 * @exception: A #GError
 *
 * Execute read transaction to the given unit.
 */
void hinawa_fw_req_read(HinawaFwReq *self, HinawaFwUnit *unit, guint64 addr,
			guint32 frame[], guint len, GError **exception)
{
	g_return_if_fail(HINAWA_IS_FW_REQ(self));

	if (frame == NULL || len == 0) {
		raise(exception, EINVAL);
		return;
	}

	g_object_ref(unit);
	fw_req_transact(self, unit, FW_REQ_TYPE_READ, addr, frame, len,
			exception);
	g_object_unref(unit);
}

/**
 * hinawa_fw_req_lock:
 * @self: A #HinawaFwReq
 * @unit: A #HinawaFwUnit
 * @addr: A destination address of target device
 * @frame: (element-type guint32) (array length=len) (inout): a 32bit array
 * @len: (in): the number of elements in the array
 * @exception: A #GError
 *
 * Execute lock transaction to the given unit.
 */
void hinawa_fw_req_lock(HinawaFwReq *self, HinawaFwUnit *unit, guint64 addr,
			guint32 *frame[], guint len, GError **exception)
{
	g_return_if_fail(HINAWA_IS_FW_REQ(self));

	if (*frame == NULL || len == 0) {
		raise(exception, EINVAL);
		return;
	}

	g_object_ref(unit);
	fw_req_transact(self, unit, FW_REQ_TYPE_COMPARE_SWAP, addr, *frame, len,
			exception);
	g_object_unref(unit);
}

/* NOTE: For HinawaFwUnit, internal. */
void hinawa_fw_req_handle_response(HinawaFwReq *self,
				   struct fw_cdev_event_response *event)
{
	HinawaFwReqPrivate *priv;

	g_return_if_fail(HINAWA_IS_FW_REQ(self));
	priv = hinawa_fw_req_get_instance_private(self);

	/* Copy transaction frame. */
	memcpy(priv->frame, event->data, event->length);

	/* Waken a thread of an user application. */
	g_cond_signal(&priv->cond);
}
