#ifndef __ALSA_TOOLS_HINAWA_FW_FCP_H__
#define __ALSA_TOOLS_HINAWA_FW_FCP_H__

#include <glib.h>
#include <glib-object.h>
#include "internal.h"
#include "fw_unit.h"

G_BEGIN_DECLS

#define HINAWA_TYPE_FW_FCP	(hinawa_fw_fcp_get_type())

#define HINAWA_FW_FCP(obj)					\
	(G_TYPE_CHECK_INSTANCE_CAST((obj),			\
				    HINAWA_TYPE_FW_FCP,		\
				    HinawaFwFcp))
#define HINAWA_IS_FW_FCP(obj)					\
	(G_TYPE_CHECK_INSTANCE_TYPE((obj),			\
				    HINAWA_TYPE_FW_FCP))

#define HINAWA_FW_FCP_CLASS(klass)				\
	(G_TYPE_CHECK_CLASS_CAST((klass),			\
				 HINAWA_TYPE_FW_FCP,		\
				 HinawaFwFcpClass))
#define HINAWA_IS_FW_FCP_CLASS(klass)				\
	(G_TYPE_CHECK_CLASS_TYPE((klass),			\
				 HINAWA_TYPE_FW_FCP))
#define HINAWA_FW_FCP_GET_CLASS(obj)				\
	(G_TYPE_INSTANCE_GET_CLASS((obj),			\
				   HINAWA_TYPE_FW_FCP,		\
				   HinawaFwFcpClass))

typedef struct _HinawaFwFcp		HinawaFwFcp;
typedef struct _HinawaFwFcpClass	HinawaFwFcpClass;
typedef struct _HinawaFwFcpPrivate	HinawaFwFcpPrivate;

struct _HinawaFwFcp {
	GObject parent_instance;

	HinawaFwFcpPrivate *priv;
};

struct _HinawaFwFcpClass {
	GObjectClass parent_class;
};

GType hinawa_fw_fcp_get_type(void) G_GNUC_CONST;

void hinawa_fw_fcp_listen(HinawaFwFcp *self, HinawaFwUnit *unit,
			  GError **exception);
void hinawa_fw_fcp_transact(HinawaFwFcp *self,
			    guint8 req_frame[], guint req_len,
			    guint8 resp_frame[], guint resp_len,
			    GError **exception);
void hinawa_fw_fcp_unlisten(HinawaFwFcp *self);

#endif
