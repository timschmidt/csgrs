/*
 * Copyright (C) 2000, Imperial College
 *
 * This file is part of the Imperial College Exact Real Arithmetic Library.
 * See the copyright notice included in the distribution for conditions
 * of use.
 */

/*
 * This is the include for all applications using reals. It defines all
 * the types the user sees, principally Real and Bool, and the prototypes
 * functions which operate on these types.
 *
 * It also defines the various types of objects in the heap. Each is a
 * structure, the first word of which is a tag, containing amongst other
 * things a code to identify the type of object.
 */

#include <gmp.h>

typedef union _Real * Real;
typedef union _Bool * Bool;

/*
 * Each object in the heap is assigned one of the following codes.
 */
typedef enum {
	ALT,		/* selects amongst boolean guarded expressions */
	VECTOR,		/* a vector (ie rational) */
	MATX,		/* a matrix (the X reflects that there is one argument) */
	TENXY,		/* a tensor (with two arguments) */
	DIGSX,		/* a string or sequence of digits */
	SIGNX,		/* this holds one of the sign values */
	CLOSURE,	/* usually used to lazily produce futher lfts in expressions */
	BOOLX,		/* propositional operator with one argument, eg negation */
	BOOLXY,		/* operator with two arguments, eg conjunction */
	PREDX		/* predicate over reals with one argument, eg x > 0 */
} ObjType;

typedef enum {FALSE = 0, TRUE = 1} bool;
typedef enum {LT = -1, EQ = 0, GT = 1} Comparison;

/*
 * The first word of each structure in the heap is a tag word defined
 * below. 
 */
typedef struct {
	unsigned type : 4;		/* an ObjType as defined above */
	unsigned dumped : 1;	/* for traversing recursively when printing */
	unsigned value : 3;		/* sign or boolean value */
	unsigned isSigned : 1;	/* true when a real must be signed */
	unsigned nodeId : 21;	/* each node has a unique id for debugging */
} Tag;

/*
 * The next segment of the file defines the structures and constants
 * dealing with reals.
 */

/*
 * Next we define the possible sign constant which includes a constant 
 * indicating that the sign is unknown.
 * The order of the entries in the enum which follows is significant. It
 * reflects our preference for SZERO. However, note that changing the order of
 * the constants below is not sufficient to change the preference. One
 * must also modify emitSign.c.
 */
typedef enum {SZERO, SINF, SPOS, SNEG, SIGN_UNKN} Sign;
typedef enum {DPOS = 1, DNEG = -1, DZERO = 0} Digit;

/*
 * We define the LFTS: vector, matrix and tensor. There are two
 * varieties of each: one in which the entries are machine sized integers
 * and the other in which the entries are GMP large integers.
 *
 * Vectors, matrices and tensors are arrays rather than structs. That
 * way they have size and yet are passed as pointers (lvalues). This
 * avoids some referencing and dereferencing. It turns out to be more
 * convenient to define tensors as arrays of vectors rather than pairs
 * of matrices since often we need to pull out columns without respecting
 * matrix boundaries.
 */
typedef mpz_t Vector[2];
typedef Vector Matrix[2];
typedef Vector Tensor[4];

typedef int SmallVector[2];
typedef SmallVector SmallMatrix[2];
typedef SmallVector SmallTensor[4];

typedef union {
	Vector Vec;
	Matrix Mat;
	Tensor Ten;
} LFT;

/*
 * This defines the fields which are common to all real continuations.
 */
typedef struct {
	Tag tag;
} Generic;

typedef struct {
	Bool guard;
	Real x;
} GuardedExpr;

typedef struct {
	Tag tag;
	void (*force)();	/* is this needed ? */
	Real redirect;		/* this is the ultimate value of the conditional */
	int nextGE;			/* index of next guarded expression to force */
	int numGE;			/* number of guarded expressions in the list */
	GuardedExpr *GE;	/* the list of guarded expressions */
} Alt;

/*
 * The x field below is the alternative representation of the real the 
 * Closure denotes. That is, when the closure is forced, it struct points
 * to the object it unfolds to. That way, any subsequent request to the
 * closure, won't force a second distinct unfolding.
 */
typedef struct {
	Tag tag;
	void (*force)();
	void *userData;
	Real redirect;	
} Cls;

/*
Heap elements which hold lfts have a strm field. This hold a real stream 
(ie a list of DigsX structures possibly prefixed by a SignX). This real 
denotes the same value as the expression rooted by the lft. In some cases
having both representations is necessary. For top level vectors and matrices,
having the stream representation allows us to extract information from
the lft when demanded for printing. At the same time, the vectors and
matrices are still available should they become the argument of another
lft whereupon we want to reduce. The strm field is used by tensors since
reduction against a tensor argument is not possible and we must emit
imformation. However, it is sometimes not possible to create the stream
in advance. If the tensor has an argument guarded by an alt, then the alt
might eventually reduce to a vector and the tensor reduce to a matrix.
*/
 
typedef struct {
	Tag tag;
	void (*force)();
	Real strm;
	Vector vec;
} Vec;

typedef struct {
	Tag tag;
	void (*force)();
	Real strm;
	Real x;
	int totalEmitted;
	Matrix mat;
} MatX;

typedef struct {
	Tag tag;
	void (*forceX)();
	void (*forceY)();
	Real strm;
	Real x;
	Real y;
	signed short totalEmitted;
	signed short tensorFairness; /* counts left vs right absorptions */
	int xDigitsNeeded;
	Tensor ten;
} TenXY;

typedef struct {
	Tag tag;
	void (*force)();
	Real x;
} SignX;

/*
 * Sequences of digits are represented by ``characteristic pairs''. 
 * This is a pair of integers (n,c) in which c is the coding of
 * n digits. When n <= DIGITS_PER_WORD, then c can be represented in a
 * machine word. Otherwise we need a big integer.
 *
 * Below we use the value 30. In fact it is possible to use 31. The problem
 * is that we need DIGITS_PER_WORD + 1 bits when we form a matrix.
 * Setting it to 30 ensures that it always yields a matrix which fits
 * into a signed machine word.
 */
#define DIGITS_PER_WORD 29

typedef struct _DigsX {
	Tag tag;
	void (*force)();
	Real x;
	unsigned count;
	union {
		int small;
		mpz_t big;
	} word;
} DigsX;

union _Real {
	Generic gen;
	Alt alt;
	Vec vec;
	SignX signX;
	MatX matX;
	DigsX digsX;
	TenXY tenXY;
	Cls cls;
};

/*
 * The next segment of the file defines the structures and constants
 * dealing with the lazy boolean type.
 *
 * I'm not very happy with the names of things here. There is too much
 * overloading of the term bool (in both upper and lower case).
 *
 * bool : this is the usual booleans (two-valued) and used internally
 *        instead of int (and 0/1) simply as a matter of style.
 * BoolVal : this is three valued boolean type
 * Bool : this is the lazy boolean type which takes on values from BoolVal.
 *        The type is a union of the various heap allocated structures
 *        relevant to lazy booleans.
 */
typedef enum {LAZY_TRUE, LAZY_FALSE, LAZY_UNKNOWN} BoolVal;

/*
 * This is the constant to flag the default case in the realAlt function.
 * The second of these is for backwards compatibility.
 */
#define DEFAULT_GUARD ((unsigned) 0xffffffffL)
#define B_DEFAULT (DEFAULT_GUARD)

typedef struct {
	Tag tag;
	void (*force)();
	Real x;
} PredX;

typedef struct {
	Tag tag;
	void (*force)();
	Bool x;
} BoolX;

typedef struct {
	Tag tag;
	void (*force)();
	Bool x;
	Bool y;
} BoolXY;

union _Bool {
	struct {
		Tag tag;
		void (*force)();
	} gen;
	BoolX boolX;
	BoolXY boolXY;
	PredX predX;
};

void initReals();

Real consCN(Real, char *, int, int, int);

void dumpReal(Real);
void dumpBool(Bool);
void dumpCell(void *);

/*
 * An application can set the following string to the name of the 
 * application, typically argv[0].
 */
extern char *MyName;
 
void force_R_Dec(Real, int);
void force_R_Digs(Real, int);

void print_R_Dec(Real, int);
void print_R_Digs(Real, int);

void print_R(Real);

double realToDouble(Real);

/*
 * Basic LFT creation. This list includes the preferred names (real_*, lft_*)
 * and the older names which may disappear in a future release.
 */
Real real_QInt(int, int);
Real real_QZ(mpz_t, mpz_t);

Real lft_R_Int(Real, int, int, int, int);
Real lft_R_Z(Real, mpz_t, mpz_t, mpz_t, mpz_t);

Real lft_R_R_Int(Real, Real, int, int, int, int, int, int, int, int);
Real lft_R_R_Z(Real, Real, mpz_t, mpz_t, mpz_t, mpz_t,
                           mpz_t, mpz_t, mpz_t, mpz_t);

Real vector_Int(int, int);
Real vector_Z(mpz_t, mpz_t);

Real matrix_Int(Real, int, int, int, int);
Real matrix_Z(Real, mpz_t, mpz_t, mpz_t, mpz_t);

Real tensor_Int(Real, Real, int, int, int, int, int, int, int, int);
Real tensor_Z(Real, Real, mpz_t, mpz_t, mpz_t, mpz_t,
                          mpz_t, mpz_t, mpz_t, mpz_t);

Real makeStream(Real);

/*
 * Basic arithmetic functions
 */
Real add_R_R(Real, Real);
Real add_R_Int(Real, int);
Real add_R_QInt(Real, int, int);
Real add_R_QZ(Real, mpz_t, mpz_t);

Real sub_R_R(Real, Real);
Real sub_R_Int(Real, int);
Real sub_R_QInt(Real, int, int);
Real sub_Int_R(int, Real);
Real sub_QInt_R(int, int, Real);

Real mul_R_R(Real, Real);
Real mul_R_Int(Real, int);
Real mul_R_QInt(Real, int, int);
Real mul_R_QZ(Real, mpz_t, mpz_t);

Real div_R_R(Real, Real);
Real div_R_Int(Real, int);
Real div_R_QInt(Real, int, int);
Real div_Int_R(int, Real);
Real div_QInt_R(int, int, Real);

Real pow_R_R(Real, Real);

Real abs_R(Real);

Real neg_R(Real);

/*
 * Analytic functions
 */
Real tan_R(Real);
Real tan_QZ(mpz_t, mpz_t);
Real tan_QInt(int, int);

Real atan_R(Real);
Real atan_QZ(mpz_t, mpz_t);
Real atan_QInt(int, int);

Real tanh_R(Real);
Real tanh_QZ(mpz_t, mpz_t);
Real tanh_QInt(int, int);

Real atanh_R(Real);
Real atanh_QZ(mpz_t, mpz_t);
Real atanh_QInt(int, int);

Real sin_R(Real);
Real sin_QZ(mpz_t, mpz_t);
Real sin_QInt(int, int);

Real asin_R(Real);
Real asin_QZ(mpz_t, mpz_t);
Real asin_QInt(int, int);

Real sinh_R(Real);
Real sinh_QZ(mpz_t, mpz_t);
Real sinh_QInt(int, int);

Real asinh_R(Real);
Real asinh_QZ(mpz_t, mpz_t);
Real asinh_QInt(int, int);

Real cos_R(Real);
Real cos_QZ(mpz_t, mpz_t);
Real cos_QInt(int, int);

Real acos_R(Real);
Real acos_QZ(mpz_t, mpz_t);
Real acos_QInt(int, int);

Real cosh_R(Real);
Real cosh_QZ(mpz_t, mpz_t);
Real cosh_QInt(int, int);

Real acosh_R(Real);
Real acosh_QZ(mpz_t, mpz_t);
Real acosh_QInt(int, int);

Real sec_R(Real);
Real sec_QInt(int, int);
Real sec_QZ(mpz_t, mpz_t);

Real asec_R(Real);
Real asec_QInt(int, int);
Real asec_QZ(mpz_t, mpz_t);

Real sech_R(Real);
Real sech_QInt(int, int);
Real sech_QZ(mpz_t, mpz_t);

Real asech_R(Real);
Real asech_QInt(int, int);
Real asech_QZ(mpz_t, mpz_t);

Real cosec_R(Real);
Real cosec_QInt(int, int);
Real cosec_QZ(mpz_t, mpz_t);

Real acosec_R(Real);
Real acosec_QInt(int, int);
Real acosec_QZ(mpz_t, mpz_t);

Real cosech_R(Real);
Real cosech_QInt(int, int);
Real cosech_QZ(mpz_t, mpz_t);

Real acosech_R(Real);
Real acosech_QInt(int, int);
Real acosech_QZ(mpz_t, mpz_t);

Real cotan_R(Real);
Real cotan_QInt(int, int);
Real cotan_QZ(mpz_t, mpz_t);

Real acotan_R(Real);
Real acotan_QInt(int, int);
Real acotan_QZ(mpz_t, mpz_t);

Real cotanh_R(Real);
Real cotanh_QInt(int, int);
Real cotanh_QZ(mpz_t, mpz_t);

Real acotanh_R(Real);
Real acotanh_QInt(int, int);
Real acotanh_QZ(mpz_t, mpz_t);

Real Pi;
Real E;

Real sqrt_R(Real);
Real sqrt_QZ(mpz_t, mpz_t);
Real sqrt_QInt(int, int);

Real exp_R(Real);
Real exp_QZ(mpz_t, mpz_t);
Real exp_QInt(int, int);

Real log_R(Real);
Real log_QZ(mpz_t, mpz_t);
Real log_QInt(int, int);

/*
 * Predicates, boolean operators and the conditional.
 */
Bool ltEq_R_0(Real);
Bool lt_R_0(Real);

Bool lt_R_R(Real, Real);
Bool ltEq_R_R(Real, Real);

Bool ltEq_R_QInt(Real, int, int);
Bool lt_R_QInt(Real, int, int);

Bool gtEq_R_0(Real);
Bool gt_R_0(Real);

Bool gtEq_R_QInt(Real, int, int);
Bool gt_R_QInt(Real, int, int);

Bool gt_R_R(Real, Real);
Bool gtEq_R_R(Real, Real);

Bool and_B_B(Bool, Bool);
Bool or_B_B(Bool, Bool);
Bool not_B(Bool);

/* BoolVal boolValue(Bool b) */

#define boolValue(b) ((b)->gen.tag.value)

Real realIf(int, ...);
Real realError(char *);

typedef void * Delay_Arg;
typedef Real (*Delay_Fun)(Delay_Arg);

char *digitToString(Digit);
char *signToString(Sign);

void retrieveInfo(Real, Sign *, int *, mpz_t);
Digit takeDigit(int *, mpz_t);


