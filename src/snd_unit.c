#include <unistd.h>
#include <alsa/asoundlib.h>
#include <sound/firewire.h>

#include "hinawa_context.h"
#include "snd_unit.h"
#include "internal.h"

/**
 * SECTION:snd_unit
 * @Title: HinawaSndUnit
 * @Short_description: An event listener for ALSA FireWire sound devices 
 *
 * This class is an application of ALSA FireWire stack. Any functionality which
 * ALSA drivers in the stack can be available.
 */

typedef struct {
	GSource src;
	HinawaSndUnit *unit;
	gpointer tag;
} SndUnitSource;

struct _HinawaSndUnitPrivate {
	snd_hwdep_t *hwdep;
	gchar name[32];
	gint type;
	gint card;
	gchar device[16];
	guint64 guid;
	gboolean streaming;

	HinawaSndUnitHandle *handle;
	void *private_data;

	void *buf;
	unsigned int len;
	SndUnitSource *src;
};
G_DEFINE_TYPE_WITH_PRIVATE(HinawaSndUnit, hinawa_snd_unit, G_TYPE_OBJECT)
#define SND_UNIT_GET_PRIVATE(obj)					\
	(G_TYPE_INSTANCE_GET_PRIVATE((obj), 				\
				HINAWA_TYPE_SND_UNIT, HinawaSndUnitPrivate))

enum snd_unit_prop_type {
	SND_UNIT_PROP_TYPE_NAME = 1,
	SND_UNIT_PROP_TYPE_FW_TYPE,
	SND_UNIT_PROP_TYPE_CARD,
	SND_UNIT_PROP_TYPE_DEVICE,
	SND_UNIT_PROP_TYPE_GUID,
	SND_UNIT_PROP_TYPE_STREAMING,
	SND_UNIT_PROP_TYPE_COUNT,
};
static GParamSpec *snd_unit_props[SND_UNIT_PROP_TYPE_COUNT] = { NULL, };

/* This object has one signal. */
enum snd_unit_sig_type {
	SND_UNIT_SIG_TYPE_LOCK_STATUS = 0,
	SND_UNIT_SIG_TYPE_COUNT,
};
static guint snd_unit_sigs[SND_UNIT_SIG_TYPE_COUNT] = { 0 };

static void snd_unit_get_property(GObject *obj, guint id,
				  GValue *val, GParamSpec *spec)
{
	HinawaSndUnit *self = HINAWA_SND_UNIT(obj);

	switch (id) {
	case SND_UNIT_PROP_TYPE_NAME:
		g_value_set_string(val, (const gchar *)self->priv->name);
		break;
	case SND_UNIT_PROP_TYPE_FW_TYPE:
		g_value_set_int(val, self->priv->type);
		break;
	case SND_UNIT_PROP_TYPE_CARD:
		g_value_set_int(val, self->priv->card);
		break;
	case SND_UNIT_PROP_TYPE_DEVICE:
		g_value_set_string(val, (const gchar *)self->priv->device);
		break;
	case SND_UNIT_PROP_TYPE_GUID:
		g_value_set_uint64(val, self->priv->guid);
		break;
	case SND_UNIT_PROP_TYPE_STREAMING:
		g_value_set_boolean(val, self->priv->streaming);
		break;
	default:
		G_OBJECT_WARN_INVALID_PROPERTY_ID(obj, id, spec);
		break;
	}
}

static void snd_unit_set_property(GObject *obj, guint id,
				  const GValue *val, GParamSpec *spec)
{
	HinawaSndUnit *self = HINAWA_SND_UNIT(obj);

	switch (id) {
	case SND_UNIT_PROP_TYPE_NAME:
		if (g_value_get_string(val) != NULL)
			strncpy(self->priv->name, g_value_get_string(val),
			        sizeof(self->priv->name));
		break;
	case SND_UNIT_PROP_TYPE_FW_TYPE:
		self->priv->type = g_value_get_int(val);
		break;
	case SND_UNIT_PROP_TYPE_CARD:
		self->priv->card = g_value_get_int(val);
		break;
	case SND_UNIT_PROP_TYPE_DEVICE:
		if (g_value_get_string(val) != NULL)
			strncpy(self->priv->device, g_value_get_string(val),
			        sizeof(self->priv->device));
		break;
	case SND_UNIT_PROP_TYPE_GUID:
		self->priv->guid = g_value_get_uint64(val);
		break;
	case SND_UNIT_PROP_TYPE_STREAMING:
		self->priv->streaming = g_value_get_boolean(val);
		break;
	default:
		G_OBJECT_WARN_INVALID_PROPERTY_ID(obj, id, spec);
		break;
	}
}

static void snd_unit_dispose(GObject *obj)
{
	HinawaSndUnit *self = HINAWA_SND_UNIT(obj);
	HinawaSndUnitPrivate *priv = SND_UNIT_GET_PRIVATE(self);

	if (priv->src != NULL)
		hinawa_snd_unit_unlisten(self);

	snd_hwdep_close(priv->hwdep);

	G_OBJECT_CLASS(hinawa_snd_unit_parent_class)->dispose(obj);
}

static void snd_unit_finalize(GObject *gobject)
{
	G_OBJECT_CLASS(hinawa_snd_unit_parent_class)->finalize(gobject);
}

static void hinawa_snd_unit_class_init(HinawaSndUnitClass *klass)
{
	GObjectClass *gobject_class = G_OBJECT_CLASS(klass);

	gobject_class->get_property = snd_unit_get_property;
	gobject_class->set_property = snd_unit_set_property;
	gobject_class->dispose = snd_unit_dispose;
	gobject_class->finalize = snd_unit_finalize;

	snd_unit_props[SND_UNIT_PROP_TYPE_NAME] =
		g_param_spec_string("name", "name",
				    "A name of this sound device.",
				    NULL,
				    G_PARAM_READWRITE | G_PARAM_CONSTRUCT_ONLY);
	snd_unit_props[SND_UNIT_PROP_TYPE_FW_TYPE] =
		g_param_spec_int("type", "type",
				 "The value of SNDRV_FIREWIRE_TYPE_XXX",
				 0, INT_MAX,
				 0,
				 G_PARAM_READABLE);
	snd_unit_props[SND_UNIT_PROP_TYPE_CARD] =
		g_param_spec_int("card", "card",
				 "A numerical ID for ALSA sound card",
				 0, INT_MAX,
				 0,
				 G_PARAM_READABLE);
	snd_unit_props[SND_UNIT_PROP_TYPE_DEVICE] =
		g_param_spec_string("device", "device",
				    "A name of special file for this firewire "
				    "unit.",
				    NULL,
				    G_PARAM_READABLE);
	snd_unit_props[SND_UNIT_PROP_TYPE_STREAMING] =
		g_param_spec_boolean("streaming", "streaming",
				     "Whether this device is streaming or not",
				     FALSE,
				     G_PARAM_READABLE);
	snd_unit_props[SND_UNIT_PROP_TYPE_GUID] =
		g_param_spec_uint64("guid", "guid",
				    "Global unique ID for this firewire unit.",
				    0, ULONG_MAX, 0,
				    G_PARAM_READABLE);

	g_object_class_install_properties(gobject_class,
					  SND_UNIT_PROP_TYPE_COUNT,
					  snd_unit_props);

	/**
	 * HinawaSndUnit::lock-status:
	 * @self: A #HinawaSndUnit
	 * @state: %TRUE when locked, %FALSE when unlocked.
	 *
	 * When ALSA kernel-streaming status is changed, this ::lock-status
	 * signal is generated.
	 */
	snd_unit_sigs[SND_UNIT_SIG_TYPE_LOCK_STATUS] =
		g_signal_new("lock-status",
			     G_OBJECT_CLASS_TYPE(klass),
			     G_SIGNAL_RUN_LAST,
			     0,
			     NULL, NULL,
			     g_cclosure_marshal_VOID__BOOLEAN,
			     G_TYPE_NONE, 1, G_TYPE_BOOLEAN);
}

static void hinawa_snd_unit_init(HinawaSndUnit *self)
{
	self->priv = hinawa_snd_unit_get_instance_private(self);
}

/* For internal use. */
void hinawa_snd_unit_open(HinawaSndUnit *self, gchar *path, GError **exception)
{
	HinawaSndUnitPrivate *priv;
	snd_hwdep_t *hwdep;
	snd_hwdep_info_t *hwdep_info;
	struct snd_firewire_get_info fw_info;
	char *name;
	int err;

	g_return_if_fail(HINAWA_IS_SND_UNIT(self));
	priv = SND_UNIT_GET_PRIVATE(self);

	err = snd_hwdep_open(&hwdep, path, SND_HWDEP_OPEN_DUPLEX);
	if (err < 0)
		goto exception;

	/* Get HwDep device information. */
	err = snd_hwdep_info_malloc(&hwdep_info);
	if (err < 0) {
		snd_hwdep_close(hwdep);
		goto exception;
	}

	err = snd_hwdep_info(hwdep, hwdep_info);
	if (err == 0)
		err = snd_card_get_name(snd_hwdep_info_get_card(hwdep_info),
					&name);
	snd_hwdep_info_free(hwdep_info);
	if (err < 0) {
		snd_hwdep_close(hwdep);
		goto exception;
	}
	strncpy(priv->name, name, sizeof(priv->name));
	free(name);

	/* Get FireWire sound device information. */
	err = snd_hwdep_ioctl(hwdep, SNDRV_FIREWIRE_IOCTL_GET_INFO, &fw_info);
	if (err < 0) {
		snd_hwdep_close(hwdep);
		goto exception;
	}

	priv->hwdep = hwdep;
	priv->type = fw_info.type;
	priv->card = fw_info.card;
	priv->guid = GUINT64_FROM_BE(*((guint64 *)fw_info.guid));
	strcpy(priv->device, fw_info.device_name);

	return;
exception:
	g_set_error(exception, g_quark_from_static_string(__func__),
		    -err, "%s", snd_strerror(err));
}

/**
 * hinawa_snd_unit_new:
 * @path: A path to ALSA hwdep device (i.e. hw:0)
 * @exception: A #GError
 *
 * Returns: An instance of #HinawaSndUnit
 */
HinawaSndUnit *hinawa_snd_unit_new(gchar *path, GError **exception)
{
	HinawaSndUnit *self;

	self = g_object_new(HINAWA_TYPE_SND_UNIT, NULL);
	if (self == NULL) {
		g_set_error(exception, g_quark_from_static_string(__func__),
			    ENOMEM, "%s", strerror(ENOMEM));
		return NULL;
	}

	hinawa_snd_unit_open(self, path, exception);
	if (*exception != NULL) {
		g_clear_object(&self);
		return NULL;
	}

	return self;
}

/* For internal use only. */
void hinawa_snd_unit_add_handle(HinawaSndUnit *self,
				HinawaSndUnitHandle *handle, void *private_data)
{
	HinawaSndUnitPrivate *priv;

	g_return_if_fail(HINAWA_IS_SND_UNIT(self));
	priv = SND_UNIT_GET_PRIVATE(self);

	priv->handle = handle;
	priv->private_data = private_data;
}


/**
 * hinawa_snd_unit_lock:
 * @self: A #HinawaSndUnit
 * @exception: A #GError
 *
 * Disallow ALSA to start kernel-streaming.
 */
void hinawa_snd_unit_lock(HinawaSndUnit *self, GError **exception)
{
	int err;

	g_return_if_fail(HINAWA_IS_SND_UNIT(self));

	err = snd_hwdep_ioctl(self->priv->hwdep, SNDRV_FIREWIRE_IOCTL_LOCK,
			      NULL);
	if (err < 0)
		g_set_error(exception, g_quark_from_static_string(__func__),
			    -err, "%s", snd_strerror(err));
}

/**
 * hinawa_snd_unit_unlock:
 * @self: A #HinawaSndUnit
 * @exception: A #GError
 *
 * Allow ALSA to start kernel-streaming.
 */
void hinawa_snd_unit_unlock(HinawaSndUnit *self, GError **exception)
{
	int err;

	g_return_if_fail(HINAWA_IS_SND_UNIT(self));

	err = snd_hwdep_ioctl(self->priv->hwdep, SNDRV_FIREWIRE_IOCTL_UNLOCK,
			      NULL);
	if (err < 0)
		g_set_error(exception, g_quark_from_static_string(__func__),
			    -err, "%s", snd_strerror(err));
}

/* For internal use. */
void hinawa_snd_unit_write(HinawaSndUnit *self,
			   const void *buf, unsigned int length,
			   GError **exception)
{
	int err;

	g_return_if_fail(HINAWA_IS_SND_UNIT(self));

	err = snd_hwdep_write(self->priv->hwdep, buf, length);
	if (err < 0)
		g_set_error(exception, g_quark_from_static_string(__func__),
			    -err, "%s", snd_strerror(err));
}

static void handle_lock_event(HinawaSndUnit *self,
			      void *buf, unsigned int length)
{
	struct snd_firewire_event_lock_status *event =
			(struct snd_firewire_event_lock_status *)buf;

	g_return_if_fail(HINAWA_IS_SND_UNIT(self));

	g_signal_emit(self, snd_unit_sigs[SND_UNIT_SIG_TYPE_LOCK_STATUS], 0,
		      event->status);
}

static gboolean prepare_src(GSource *src, gint *timeout)
{
	/* Set 2msec for poll(2) timeout. */
	*timeout = 2;

	/* This source is not ready, let's poll(2) */
	return FALSE;
}

static gboolean check_src(GSource *gsrc)
{
	SndUnitSource *src = (SndUnitSource *)gsrc;
	HinawaSndUnit *unit = src->unit;
	HinawaSndUnitPrivate *priv = SND_UNIT_GET_PRIVATE(unit);
	GIOCondition condition;

	struct snd_firewire_event_common *common;
	int len;

	if (unit == NULL)
		goto end;

	/* Let's process this source if any inputs are available. */
	condition = g_source_query_unix_fd((GSource *)src, src->tag);
	if (!(condition & G_IO_IN))
		goto end;

	len = snd_hwdep_read(unit->priv->hwdep, unit->priv->buf,
			     unit->priv->len);
	if (len < 0)
		goto end;

	common = (struct snd_firewire_event_common *)unit->priv->buf;

	if (common->type == SNDRV_FIREWIRE_EVENT_LOCK_STATUS)
		handle_lock_event(unit, priv->buf, len);
	else if (priv->handle != NULL) {
		priv->handle(priv->private_data, priv->buf, len);
	}
end:
	/* Don't go to dispatch, then continue to process this source. */
	return FALSE;
}

static gboolean dispatch_src(GSource *src, GSourceFunc callback,
			     gpointer user_data)
{
	/* Just be sure to continue to process this source. */
	return TRUE;
}

static void finalize_src(GSource *src)
{
	/* Do nothing paticular. */
	return;
}

/**
 * hinawa_snd_unit_listen:
 * @self: A #HinawaSndUnit
 * @exception: A #GError
 *
 * Start listening to events.
 */
void hinawa_snd_unit_listen(HinawaSndUnit *self, GError **exception)
{
	static GSourceFuncs funcs = {
		.prepare	= prepare_src,
		.check		= check_src,
		.dispatch	= dispatch_src,
		.finalize	= finalize_src,
	};
	HinawaSndUnitPrivate *priv;
	void *buf;
	struct pollfd pfds;
	GSource *src;
	int err;

	g_return_if_fail(HINAWA_IS_SND_UNIT(self));
	priv = SND_UNIT_GET_PRIVATE(self);

	/*
	 * MEMO: allocate one page because we cannot assume the size of
	 * transaction frame.
	 */
	buf = g_malloc(getpagesize());
	if (buf == NULL) {
		g_set_error(exception, g_quark_from_static_string(__func__),
			    ENOMEM, "%s", strerror(ENOMEM));
		return;
	}

	if (snd_hwdep_poll_descriptors(priv->hwdep, &pfds, 1) != 1) {
		g_set_error(exception, g_quark_from_static_string(__func__),
			    EINVAL, "%s", strerror(EINVAL));
		return;
	}

	src = g_source_new(&funcs, sizeof(SndUnitSource));
	if (src == NULL) {
		g_set_error(exception, g_quark_from_static_string(__func__),
			    ENOMEM, "%s", strerror(ENOMEM));
		g_free(buf);
		return;
	}

	g_source_set_name(src, "HinawaSndUnit");
	g_source_set_priority(src, G_PRIORITY_HIGH_IDLE);
	g_source_set_can_recurse(src, TRUE);

	((SndUnitSource *)src)->unit = self;
	priv->src = (SndUnitSource *)src;
	priv->buf = buf;
	priv->len = getpagesize();

	((SndUnitSource *)src)->tag =
		hinawa_context_add_src(src, pfds.fd, G_IO_IN, exception);
	if (*exception != NULL) {
		g_free(buf);
		g_source_destroy(src);
		priv->buf = NULL;
		priv->len = 0;
		priv->src = NULL;
		return;
	}

	/* Check locked or not. */
	err = snd_hwdep_ioctl(priv->hwdep, SNDRV_FIREWIRE_IOCTL_LOCK,
			      NULL);
	priv->streaming = (err == -EBUSY);
	if (err == -EBUSY)
		return;
	snd_hwdep_ioctl(priv->hwdep, SNDRV_FIREWIRE_IOCTL_UNLOCK, NULL);
}

/**
 * hinawa_snd_unit_unlisten:
 * @self: A #HinawaSndUnit
 *
 * Stop listening to events.
 */
void hinawa_snd_unit_unlisten(HinawaSndUnit *self)
{
	HinawaSndUnitPrivate *priv;

	g_return_if_fail(HINAWA_IS_SND_UNIT(self));
	priv = SND_UNIT_GET_PRIVATE(self);

	if (priv->streaming)
		snd_hwdep_ioctl(priv->hwdep, SNDRV_FIREWIRE_IOCTL_UNLOCK, NULL);

	g_source_destroy((GSource *)priv->src);
	g_free(priv->src);
	priv->src = NULL;

	g_free(priv->buf);
	priv->buf = NULL;
	priv->len = 0;
}
