
#ifndef iRRAM_COMMON_H
#define iRRAM_COMMON_H

#if defined(__GNUC__) || defined(__clang__) || defined(__builtin_expect)
/* Don't need to check version of compilers, they all support __builtin_expect()
 * since the time they know of C++11 */
# define iRRAM_expect(e,n)	__builtin_expect((e),(n))
#else
# define iRRAM_expect(e,n)	(e)
#endif
#define iRRAM_likely(x)		iRRAM_expect(!!(x), 1)
#define iRRAM_unlikely(x)	iRRAM_expect(!!(x), 0)

#include <iRRAM/version.h>

#ifdef __cplusplus
extern "C" {
#endif

#define iRRAM_DEFAULT_PREC_SKIP   5
#define iRRAM_DEFAULT_PREC_START  1
#define iRRAM_DEFAULT_DEBUG       0
#define iRRAM_prec_steps          512

struct iRRAM_init_options {
	int    starting_prec;
	int    prec_inc;
	double prec_factor;
	int    debug;
	int    prec_skip;
	int    prec_start;
};

#define iRRAM_INIT_OPTIONS_INIT { \
	/* .starting_prec = */ -50,                       \
	/* .prec_inc      = */ -20,                       \
	/* .prec_factor   = */  1.25,                     \
	/* .debug         = */  iRRAM_DEFAULT_DEBUG,      \
	/* .prec_skip     = */  iRRAM_DEFAULT_PREC_SKIP,  \
	/* .prec_start    = */  iRRAM_DEFAULT_PREC_START, \
}

void iRRAM_initialize(int argc, char **argv);

/*! \brief like iRRAM_initialize(), but modifies its arguments as to remove
 *         parsed options pertaining iRRAM. */
void iRRAM_initialize2(int *argc, char **argv);

void iRRAM_initialize3(const struct iRRAM_init_options *opts);

int iRRAM_parse_args(struct iRRAM_init_options *opts, int *argc, char **argv);

void iRRAM_finalize(void);

extern const char *const *const iRRAM_error_msg;

#define ERRORDEFINE(x, y) x,
enum iRRAM_exception_list {
#include <iRRAM/errno.h>
};
#undef ERRORDEFINE

int iRRAM_exec(void (*)(void *), void *);

#ifdef __cplusplus
}
#endif

#endif
