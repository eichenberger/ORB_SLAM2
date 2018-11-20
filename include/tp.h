#undef TRACEPOINT_PROVIDER
#define TRACEPOINT_PROVIDER my_provider

#undef TRACEPOINT_INCLUDE
#define TRACEPOINT_INCLUDE "./tp.h"

#if !defined(_TP_H) || defined(TRACEPOINT_HEADER_MULTI_READ)
#define _TP_H

#include <lttng/tracepoint.h>
#include <stdio.h>

#include "ORBextractorOCV.h"

TRACEPOINT_EVENT(
   my_provider,
   detectAndCompute,
   TP_ARGS(),
   TP_FIELDS()
)

TRACEPOINT_EVENT(
   my_provider,
   computeImagePyramid,
   TP_ARGS(),
   TP_FIELDS()
)

TRACEPOINT_EVENT(
   my_provider,
   endOperator,
   TP_ARGS(),
   TP_FIELDS()
)
#endif /* _TP_H */

#include <lttng/tracepoint-event.h>
