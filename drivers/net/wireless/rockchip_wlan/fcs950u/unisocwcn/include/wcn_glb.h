#ifndef __WCN_GLB_H__
#define __WCN_GLB_H__

#include "marlin_platform.h"

#include "bufring.h"
#include "loopcheck.h"
#include "mdbg_type.h"
#include "rdc_debug.h"
#include "reset.h"
#include "sysfs.h"
#include "wcn_dbg.h"
#include "wcn_parn_parser.h"
#include "wcn_txrx.h"
#include "wcn_log.h"

#ifdef CONFIG_SC2342_INTEG
#include "sc2342_integrate_glb.h"
#endif

#ifdef CONFIG_SC2355
#include "sc2355_glb.h"
#include "wcn_dump.h"
#endif

#ifdef CONFIG_UMW2652
#include "umw2652_glb.h"
#include "wcn_dump.h"
#endif

#ifdef CONFIG_UMW2653
#include "umw2653_glb.h"
#include "wcn_dump.h"
#endif

#endif
