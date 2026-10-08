/*
 * Copyright (C) 2000, Imperial College
 *
 * This file is part of the Imperial College Exact Real Arithmetic Library.
 * See the copyright notice included in the distribution for conditions
 * of use.
 */

#ifdef DAVINCI
#define DAVINCI 1
#else
#define DAVINCI 0
#endif

#include <stdint.h>

#ifndef REALDIR
#define REALDIR "."
#endif

/*
 * There are three choices for tracing.
 * TRACE=0, off
 * TRACE=1, on
 * TRACE=traceOn, software controlled.
 *
 * The default, if not set when compiled (-DTRACE=<val>), is off.
 */
#ifndef TRACE
#define TRACE 0
#endif

void debugTrace(int);   /* this function just sets traceOn to the argument */
extern int traceOn;

/*
 * The following are used in the routine
 * Error(fatal_flag, error_type, proc, fmt, arg... )
 *
 * When FATAL is used (rather than !FATAL), Error exists after
 * printing a message.
 */
#define FATAL   1

/*
 * Error types, E_SYS for system errors (eg opening files) and E_INT
 * for internal errors not involving system calls.
 */
#define E_SYS   1
#define E_INT   2
void Error(int fatal_flag, int error_type, char *proc, char *fmt, ...);
 
/*
 * Macros and utilities not meant for the user.
 */
#ifndef MAX
#define MAX(a,b) ((a) >= (b) ? (a) : (b))
#endif

#ifndef MIN
#define MIN(a,b) ((a) <= (b) ? (a) : (b))
#endif

#ifdef USED_FOR_GMP_2
/*
 * This is a little hack to avoid assignment and storage allocation
 * within gmp. This just swaps the fields describing an mpz_t and
 * that way we can multiply two matrices and put the result in the
 * first matrix without allocating temporary storage every time.
 */
#define MPZ_SWAP(a,b) \
    ({mpz_t localz; \
      localz[0] = (a)[0]; \
      (a)[0] = (b)[0]; \
      (b)[0] = localz[0];})
#endif

#define MPZ_SWAP(a,b)	mpz_swap(a,b)

/*
 * This macro evaluates its argument more that once. It is applied to the output
 * of some GMP comparison functions. Some comparisons return {-1,0,1} while
 * others only specify a negative, zero or positive value. The former is
 * slightly better since we can use a case statement. For the latter
 * functions we wrap them in SIGN so we can uniformly use case statements.
 */
#define MPZ_SIGN(x) ((x > 0) ? 1 : ((x < 0) ? -1 : 0))
#define MAXINT  0x7fffffff

/*
 * These are available for temporary storage. But one must
 * be careful. They are used in the matrix multiplication operations
 * amongst other places.
 */
extern mpz_t tmpa_z, tmpb_z, tmpc_z, tmpd_z, tmpe_z, tmpf_z;

/*
 * The constant zero as a big integer
 */
extern mpz_t zero_z;

extern Matrix bigTmpMat;
extern Tensor bigTmpTen;

extern int debug;

void canonVector(Vector);
int normalizeVector(Vector);
int normalizeMatrix(Matrix);
int normalizeTensor(Tensor);

void absorb_DigsX_Into_DigsX(DigsX *);
void absorb_DigsX_Into_MatX(MatX *);
void absorb_DigsX_Into_TenXY_X(TenXY *);
void absorb_DigsX_Into_TenXY_Y(TenXY *);
void reduceDigsXList(DigsX *);

/*
 *
 * The entire computation is driven by a stack of frames which define
 * the work to be done.  The stack grows upward with sp pointing to the
 * top of the stack.
 */
#ifndef STACK_SIZE
#define STACK_SIZE 8000
#endif

typedef uintptr_t StackWord;

extern StackWord *stack;
extern StackWord *stackBound;
extern StackWord *sp;

#define NEED_STACK(n) \
	({if ((sp + n) >= stackBound) \
		Error(FATAL, E_INT, "push", "stack overflow");})

#define PUSH(x)		(*++sp = (StackWord) (x))
#define POP			(*sp--)

#if DAVINCI == 1
void highlightTOS();

#define PUSH_4(func, dst, a, b) \
	push_4((StackWord) (func), (StackWord) (dst), (StackWord) (a), (StackWord) (b))

#define PUSH_3(func, dst, a) \
	push_3((StackWord) (func), (StackWord) (dst), (StackWord) (a))

#define PUSH_2(func, dst) \
	push_2((StackWord) (func), (StackWord) (dst))

/*
 * This uses the GNU inline extension.  Could be done with macros but
 * inlines are nicer.
 */
static inline void
push_4(StackWord func, StackWord dst, StackWord a, StackWord b)
{
	NEED_STACK(4);
	PUSH(b);
	PUSH(a);
	PUSH(dst);
	PUSH(func);
	highlightTOS();
}

static inline void
push_3(StackWord func, StackWord dst, StackWord a)
{
	NEED_STACK(3);
	PUSH(a);
	PUSH(dst);
	PUSH(func);
	highlightTOS();
}

static inline void
push_2(StackWord func, StackWord dst)
{
	NEED_STACK(2);
	PUSH(dst);
	PUSH(func);
	highlightTOS();
}

#else
#define PUSH_4(func, dst, a, b) \
	({NEED_STACK(4); \
	PUSH(b); \
	PUSH(a); \
	PUSH(dst); \
	PUSH(func);})
#define PUSH_3(func, dst, a) \
	({NEED_STACK(3); \
	PUSH(a); \
	PUSH(dst); \
	PUSH(func);})
#define PUSH_2(func, dst) \
	({NEED_STACK(2); \
	PUSH(dst); \
	PUSH(func);})
#endif

/*
 * this is used to force blocking when reading responses from
 * daVinci.
 */
#define BLOCK 1

/*
 * The following constant is the default number of digits forced from
 * an LFT as demanded by predicates and when the epsilon-delta analysis
 * doesn't tell us useful information.
 */
#ifndef DEFAULT_FORCE_COUNT
#define DEFAULT_FORCE_COUNT 1
#endif

Vec *allocVec();
DigsX *allocDigsX();
MatX *allocMatX();
TenXY *allocTenXY();

SignX *allocSignX(Real, int);
Cls *allocCls(void (*)(), void *);

/*
 * This is the type of a function for emitting a digit from a vector, matrix
 * or tensor. So "edf" means "emit digit function". The argument type is given
 * as a (void *) rather than LFT because there are occasions when it is applied
 * to things other than LFTs, for example in the square root of a rational.
 */
typedef bool (*edf)(void *, Digit *);
int emitDigits(DigsX *, edf, void *, int);

void newDigsX(DigsX *);

void multVectorPairTimesVector(Vector, Vector, Vector);
void multVectorPairTimesMatrix(Vector, Vector, Matrix);
void multVectorPairTimesSmallMatrix(Vector, Vector, SmallMatrix);
void makeSmallMatrixFromDigits(SmallMatrix, DigsX *);
void makeMatrixFromDigits(Matrix, DigsX *);

/*
 * Now we define the ``prototypes'' for all the functions available
 * to applications.
 */
Digit intToDigit(int);
char *digitToString(Digit);
char *signToString(Sign);
char *comparisonToString(Comparison);

/*
 * For debugging purposes we provide a facility to map a force method to
 * a descriptor which gives a printable string for the method, 
 * the number of arguments expected by the method and, in the case when the
 * consumer is an element in the heap with 2 arguments, a constant indicating
 * whether the information is coming from the x or y argument.
 */
#define ARG_X 0
#define ARG_Y 1
#define ARG_NEITHER 2

typedef struct {
	void (*func)();
	char *funcName;
	int nArgs;
	int argXOrY;
} ForceFuncDesc;

ForceFuncDesc *getDescForForceFunc(void (*)());

void registerForceFunc(void (*)(), char *, int);

extern int nodeId;

extern int defaultForceCount;
extern int forceDecUpperBound;
extern int stackSize;
