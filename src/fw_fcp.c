#include "fw_fcp.h"
#include "fw_resp.h"
#include "fw_req.h"

/**
 * SECTION:fw_fcp
 * @Title: HinawaFwFcp
 * @Short_description: A FCP transaction executor to a FireWire unit
 *
 * A HinawaFwFcp supports Function Control Protocol (FCP) in IEC 61883-1.
 * Some types of transaction in 'AV/C Digital Interface Command Set General
 * Specification Version 4.2' (Sep 1 2004, 1394TA) requires low layer support,
 * thus this class has a code for them.
 *
 * Any of transaction frames should be aligned to 8bit (byte).
 * This class is an application of #HinawaFwReq / #HinawaFwResp.
 */

/* For error handling. */
G_DEFINE_QUARK("HinawaFwFcp", hinawa_fw_fcp)
#define raise(exception, errno)						\
	g_set_error(exception, hinawa_fw_fcp_quark(), errno,		\
		    "%d: %s", __LINE__, strerror(errno))

#define FCP_MAXIMUM_FRAME_BYTES	0x200U
#define FCP_REQUEST_ADDR	0xfffff0000b00
#define FCP_RESPOND_ADDR	0xfffff0000d00

/* For your information. */
enum avc_type {
	AVC_TYPE_CONTROL		= 0x00,
	AVC_TYPE_STATUS			= 0x01,
	AVC_TYPE_SPECIFIC_INQUIRY	= 0x02,
	AVC_TYPE_NOTIFY			= 0x03,
	AVC_TYPE_GENERAL_INQUIRY	= 0x04,
	/* 0x05-0x07 are reserved. */
};
/* continue */
enum avc_status {
	AVC_STATUS_NOT_IMPLEMENTED	= 0x08,
	AVC_STATUS_ACCEPTED		= 0x09,
	AVC_STATUS_REJECTED		= 0x0a,
	AVC_STATUS_IN_TRANSITION	= 0x0b,
	AVC_STATUS_IMPLEMENTED_STABLE	= 0x0c,
	AVC_STATUS_CHANGED		= 0x0d,
	/* reserved */
	AVC_STATUS_INTERIM		= 0x0f,
};

struct fcp_transaction {
	guint32 req_frame[FCP_MAXIMUM_FRAME_BYTES];	/* Request frame */
	guint req_len;
	guint32 resp_frame[FCP_MAXIMUM_FRAME_BYTES];	/* Response frame */
	guint resp_len;
	GCond cond;
};

struct _HinawaFwFcpPrivate {
	HinawaFwUnit *unit;
	HinawaFwResp *resp;

	GList *transactions;
	GMutex lock;
};
G_DEFINE_TYPE_WITH_PRIVATE(HinawaFwFcp, hinawa_fw_fcp, G_TYPE_OBJECT)

static void hinawa_fw_fcp_finalize(GObject *obj)
{
	HinawaFwFcp *self = HINAWA_FW_FCP(obj);

	hinawa_fw_fcp_unlisten(self);

	G_OBJECT_CLASS(hinawa_fw_fcp_parent_class)->finalize(obj);
}

static void hinawa_fw_fcp_class_init(HinawaFwFcpClass *klass)
{
	GObjectClass *gobject_class = G_OBJECT_CLASS(klass);

	gobject_class->finalize = hinawa_fw_fcp_finalize;
}

static void hinawa_fw_fcp_init(HinawaFwFcp *self)
{
	return;
}

/**
 * hinawa_fw_fcp_transact:
 * @self: A #HinawaFwFcp
 * @req_frame: (element-type guint8) (array length=req_len) (in): a byte frame for request
 * @req_len: (in): the number of bytes in the request frame
 * @resp_frame: (element-type guint8) (array length=resp_len) (out caller-allocates): a byte frame for response
 * @resp_len: (in): the number of bytes in the request frame
 * @exception: A #GError
 */
void hinawa_fw_fcp_transact(HinawaFwFcp *self,
			    guint8 req_frame[], guint req_len,
			    guint8 resp_frame[], guint resp_len,
			    GError **exception)
{
	HinawaFwFcpPrivate *priv;
	HinawaFwReq *req;
	struct fcp_transaction trans;
	GMutex local_lock;
	gint64 expiration;
	guint32 *frame;
	guint i;

	g_return_if_fail(HINAWA_IS_FW_FCP(self));
	priv = hinawa_fw_fcp_get_instance_private(self);

	if (req_frame  == NULL || resp_frame == NULL ||
	    req_len > FCP_MAXIMUM_FRAME_BYTES ||
	    resp_len > FCP_MAXIMUM_FRAME_BYTES) {
		raise(exception, EINVAL);
		return;
	}

	req = g_object_new(HINAWA_TYPE_FW_REQ, NULL);

	/* Copy guint8 array to guint32 array. */
	frame = (guint32 *)req_frame;
	for (i = 0; i < req_len / 4; i++)
		trans.req_frame[i] = GUINT32_TO_BE(frame[i]);

	/* Insert this entry. */
	g_mutex_lock(&priv->lock);
	priv->transactions = g_list_prepend(priv->transactions, &trans);
	g_mutex_unlock(&priv->lock);

	/* NOTE: Timeout is 200 milli-seconds. */
	expiration = g_get_monotonic_time() + 200 * G_TIME_SPAN_MILLISECOND;
	g_cond_init(&trans.cond);
	g_mutex_init(&local_lock);

	/* Send this request frame. */
	hinawa_fw_req_write(req, priv->unit, FCP_REQUEST_ADDR, trans.req_frame,
			    trans.req_len, exception);
	if (*exception != NULL)
		goto end;
deferred:
	/*
	 * Wait corresponding response till timeout.
	 * NOTE: Timeout at bus-reset, illegally.
	 */
	g_mutex_lock(&local_lock);
	if (!g_cond_wait_until(&trans.cond, &local_lock, expiration))
		raise(exception, ETIMEDOUT);
	g_mutex_unlock(&local_lock);

	/* Error happened. */
	if (*exception != NULL)
		goto end;

	/* It's a deffered transaction, wait 200 milli-seconds again. */
	if (GUINT32_FROM_BE(trans.resp_frame[0]) >> 24 == AVC_STATUS_INTERIM) {
		expiration = g_get_monotonic_time() +
			     200 * G_TIME_SPAN_MILLISECOND;
		goto deferred;
	}

	/* Convert guint32 array to guint8 array. */
	/* TODO: check length */
	frame = (guint32 *)resp_frame;
	for (i = 0; i < trans.resp_len / 4; i++)
		frame[i] = GUINT32_FROM_BE(trans.resp_frame[i]);
end:
	/* Remove this entry. */
	g_mutex_lock(&priv->lock);
	priv->transactions =
			g_list_remove(priv->transactions, (gpointer *)&trans);
	g_mutex_unlock(&priv->lock);

	g_mutex_clear(&local_lock);
	g_clear_object(&req);
}

static GArray *handle_response(HinawaFwResp *self, gint tcode,
			       guint32 *resp_frame, guint resp_len,
			       gpointer user_data)
{
	HinawaFwFcp *fcp = (HinawaFwFcp *)user_data;
	HinawaFwFcpPrivate *priv = hinawa_fw_fcp_get_instance_private(fcp);
	struct fcp_transaction *trans;
	GList *entry;

	g_mutex_lock(&priv->lock);

	/* Seek correcponding request. */
	for (entry = priv->transactions; entry != NULL; entry = entry->next) {
		trans = (struct fcp_transaction *)entry->data;

		if (trans->req_frame[1] == resp_frame[1] &&
		    trans->req_frame[2] == resp_frame[2])
			break;
	}

	/* No requests corresponding to this response. */
	if (entry == NULL)
		goto end;

	memcpy(trans->resp_frame, resp_frame, resp_len);
	g_cond_signal(&trans->cond);
end:
	g_mutex_unlock(&priv->lock);

	/* Transfer no data in the response frame. */
	return NULL;
}

/**
 * hinawa_fw_fcp_listen:
 * @self: A #HinawaFwFcp
 * @unit: A #HinawaFwUnit
 * @exception: A #GError
 *
 * Start to listen to FCP responses.
 */
void hinawa_fw_fcp_listen(HinawaFwFcp *self, HinawaFwUnit *unit,
			  GError **exception)
{
	HinawaFwFcpPrivate *priv;

	g_return_if_fail(HINAWA_IS_FW_FCP(self));
	priv = hinawa_fw_fcp_get_instance_private(self);

	priv->resp = g_object_new(HINAWA_TYPE_FW_RESP, NULL);
	priv->unit = g_object_ref(unit);

	hinawa_fw_resp_register(priv->resp, priv->unit,
				FCP_RESPOND_ADDR, FCP_MAXIMUM_FRAME_BYTES,
				exception);
	if (*exception != NULL) {
		g_clear_object(&priv->resp);
		priv->resp = NULL;
		g_object_unref(priv->unit);
		priv->unit = NULL;
		return;
	}

	g_signal_connect(priv->resp, "requested",
			 G_CALLBACK(handle_response), self);

	g_mutex_init(&priv->lock);
	priv->transactions = NULL;
}

/**
 * hinawa_fw_fcp_unlisten:
 * @self: A #HinawaFwFcp
 *
 * Stop to listen to FCP responses.
 */
void hinawa_fw_fcp_unlisten(HinawaFwFcp *self)
{
	HinawaFwFcpPrivate *priv;

	g_return_if_fail(HINAWA_IS_FW_FCP(self));
	priv = hinawa_fw_fcp_get_instance_private(self);

	if (priv->resp == NULL)
		return;

	hinawa_fw_resp_unregister(priv->resp);
	priv->resp = NULL;
	g_object_unref(priv->unit);
	priv->unit = NULL;
}
