
#ifndef iRRAM_VERSION_H
#define iRRAM_VERSION_H

/* fix for bug related to '::max_align_t' in <gmp.h> exposed since >=gcc-4.9 */
#include <stddef.h>

#define iRRAM_HAVE_TLS		1
#define iRRAM_TLS		thread_local
#define iRRAM_TLS_STD		1

#define iRRAM_HAVE_GMP_C	1

#if !defined(__cplusplus) && iRRAM_TLS_STD && !defined(thread_local)
# define thread_local _Thread_local
#endif

#define iRRAM_VERSION_ct	"2014_01"
#define iRRAM_BACKENDS		"MPFR"
#define iRRAM_BACKEND_MPFR	1

#ifdef __cplusplus
extern "C" {
#endif

extern const char *iRRAM_VERSION_rt;

#ifdef __cplusplus
}
#endif

#endif
