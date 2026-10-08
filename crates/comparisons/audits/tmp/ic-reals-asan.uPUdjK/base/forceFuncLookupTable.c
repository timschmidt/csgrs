/*
 * Copyright (C) 2000, Imperial College
 *
 * This file is part of the Imperial College Exact Real Arithmetic Library.
 * See the copyright notice included in the distribution for conditions
 * of use.
 */

#include <search.h>
#include <stdio.h>
#include "real.h"
#include "real-impl.h"

/*
 * For debugging purposes we keep a table which describes each force method.
 * The table maps a pointer to a function to a descriptor which gives a
 * printable string for the function name and the number of arguments
 * expected by the function.
 */

void delayCont();
void force_Alt_Eval();
void force_From_Bool_X_Only_Cont();
void force_From_Bool_X_Only_Entry();
void force_From_Bool_Y_Only_Cont();
void force_From_Bool_Y_Only_Entry();
void forceGtEqZero_To_PredX_From_Alt_Cont();
void forceGtEqZero_To_PredX_From_Alt_Entry();
void forceGtEqZero_To_PredX_From_Cls_Cont();
void forceGtEqZero_To_PredX_From_Cls_Entry();
void forceGtEqZero_To_PredX_From_DigsX_SINF_Cont();
void forceGtEqZero_To_PredX_From_DigsX_SINF_Entry();
void forceGtEqZero_To_PredX_From_DigsX_SNEG_Cont();
void forceGtEqZero_To_PredX_From_DigsX_SNEG_Entry();
void forceGtEqZero_To_PredX_From_DigsX_SZERO_Cont();
void forceGtEqZero_To_PredX_From_DigsX_SZERO_Entry();
void forceGtEqZero_To_PredX_From_MatX_Cont();
void forceGtEqZero_To_PredX_From_MatX_Entry();
void forceGtEqZero_To_PredX_From_MatX_Signed_Cont();
void forceGtEqZero_To_PredX_From_MatX_Signed_Entry();
void forceGtEqZero_To_PredX_From_SignX_Cont();
void forceGtEqZero_To_PredX_From_SignX_Entry();
void forceGtEqZero_To_PredX_From_TenXY();
void forceGtZero_To_PredX_From_Alt_Cont();
void forceGtZero_To_PredX_From_Alt_Entry();
void forceGtZero_To_PredX_From_Cls_Cont();
void forceGtZero_To_PredX_From_Cls_Entry();
void forceGtZero_To_PredX_From_DigsX_SPOS_Cont();
void forceGtZero_To_PredX_From_DigsX_SPOS_Entry();
void forceGtZero_To_PredX_From_DigsX_SZERO_Cont();
void forceGtZero_To_PredX_From_DigsX_SZERO_Entry();
void forceGtZero_To_PredX_From_MatX_Cont();
void forceGtZero_To_PredX_From_MatX_Entry();
void forceGtZero_To_PredX_From_MatX_Signed_Cont();
void forceGtZero_To_PredX_From_MatX_Signed_Entry();
void forceGtZero_To_PredX_From_SignX_Cont();
void forceGtZero_To_PredX_From_SignX_Entry();
void forceGtZero_To_PredX_From_TenXY();
void force_To_Alt_Cont();
void force_To_Alt_Entry();
void force_To_Alt_From_The_Abyss();
void force_To_Alt_Reduce();
void force_To_And_From_Bool_X_Cont();
void force_To_And_From_Bool_X_Entry();
void force_To_And_From_Bool_Y_Cont();
void force_To_And_From_Bool_Y_Entry();
void force_To_Bool_From_The_Abyss();
void force_To_DigsX_From_Alt_Cont();
void force_To_DigsX_From_Alt_Entry();
void force_To_DigsX_From_Cls_Cont();
void force_To_DigsX_From_Cls_Entry();
void force_To_DigsX_From_DigsX_Cont();
void force_To_DigsX_From_DigsX_Entry();
void force_To_DigsX_From_MatX_Cont();
void force_To_DigsX_From_MatX_Entry();
void force_To_DigsX_From_TenXY_Cont();
void force_To_DigsX_From_TenXY_Cont_X();
void force_To_DigsX_From_TenXY_Entry();
void force_To_DigsX_From_Vec();
void force_To_MatX_From_Alt_Cont();
void force_To_MatX_From_Alt_Entry();
void force_To_MatX_From_Alt_Signed_Cont();
void force_To_MatX_From_Alt_Signed_Entry();
void force_To_MatX_From_Cls_Cont();
void force_To_MatX_From_Cls_Entry();
void force_To_MatX_From_Cls_Signed_Cont();
void force_To_MatX_From_Cls_Signed_Entry();
void force_To_MatX_From_DigsX_Cont();
void force_To_MatX_From_DigsX_Entry();
void force_To_MatX_From_DigsX_Signed();
void force_To_MatX_From_MatX();
void force_To_MatX_From_MatX_Signed_Cont();
void force_To_MatX_From_MatX_Signed_Entry();
void force_To_MatX_From_SignX_Cont();
void force_To_MatX_From_SignX_Entry();
void force_To_MatX_From_TenXY();
void force_To_MatX_From_TenXY_Signed_Cont();
void force_To_MatX_From_TenXY_Signed_Cont_X();
void force_To_MatX_From_TenXY_Signed_Entry();
void force_To_MatX_From_Vec();
void force_To_MatX_From_Vec_Signed();
void force_To_Not_From_Bool_Cont();
void force_To_Not_From_Bool_Entry();
void force_To_Or_From_Bool_X_Cont();
void force_To_Or_From_Bool_X_Entry();
void force_To_Or_From_Bool_Y_Cont();
void force_To_Or_From_Bool_Y_Entry();
void force_To_PredX_From_DigsX_2n_minus_1_False_Cont();
void force_To_PredX_From_DigsX_2n_minus_1_False_Entry();
void force_To_PredX_From_DigsX_2n_minus_1_True_Cont();
void force_To_PredX_From_DigsX_2n_minus_1_True_Entry();
void force_To_PredX_From_DigsX_minus_2n_minus_1_False_Cont();
void force_To_PredX_From_DigsX_minus_2n_minus_1_False_Entry();
void force_To_PredX_From_DigsX_minus_2n_minus_1_True_Cont();
void force_To_PredX_From_DigsX_minus_2n_minus_1_True_Entry();
void force_To_PredX_From_The_Abyss();
void force_To_SignX_From_Alt_Cont();
void force_To_SignX_From_Alt_Entry();
void force_To_SignX_From_Cls_Cont();
void force_To_SignX_From_Cls_Entry();
void force_To_SignX_From_DigsX();
void force_To_SignX_From_MatX_Cont();
void force_To_SignX_From_MatX_Entry();
void force_To_SignX_From_TenXY_Cont();
void force_To_SignX_From_TenXY_Cont_X();
void force_To_SignX_From_TenXY_Entry();
void force_To_SignX_From_Vec();
void force_To_TenXY_X_From_Alt_Cont();
void force_To_TenXY_X_From_Alt_Entry();
void force_To_TenXY_X_From_Alt_Signed_Cont();
void force_To_TenXY_X_From_Alt_Signed_Entry();
void force_To_TenXY_X_From_Cls_Cont();
void force_To_TenXY_X_From_Cls_Entry();
void force_To_TenXY_X_From_Cls_Signed_Cont();
void force_To_TenXY_X_From_Cls_Signed_Entry();
void force_To_TenXY_X_From_DigsX_Cont();
void force_To_TenXY_X_From_DigsX_Entry();
void force_To_TenXY_X_From_DigsX_Signed();
void force_To_TenXY_X_From_MatX();
void force_To_TenXY_X_From_MatX_Signed_Cont();
void force_To_TenXY_X_From_MatX_Signed_Entry();
void force_To_TenXY_X_From_SignX_Cont();
void force_To_TenXY_X_From_SignX_Entry();
void force_To_TenXY_X_From_TenXY();
void force_To_TenXY_X_From_TenXY_Signed_Cont();
void force_To_TenXY_X_From_TenXY_Signed_Cont_X();
void force_To_TenXY_X_From_TenXY_Signed_Entry();
void force_To_TenXY_X_From_Vec();
void force_To_TenXY_X_From_Vec_Signed();
void force_To_TenXY_X_Until_Refining();
void force_To_TenXY_Y_From_Alt_Cont();
void force_To_TenXY_Y_From_Alt_Entry();
void force_To_TenXY_Y_From_Alt_Signed_Cont();
void force_To_TenXY_Y_From_Alt_Signed_Entry();
void force_To_TenXY_Y_From_Cls_Cont();
void force_To_TenXY_Y_From_Cls_Entry();
void force_To_TenXY_Y_From_Cls_Signed_Cont();
void force_To_TenXY_Y_From_Cls_Signed_Entry();
void force_To_TenXY_Y_From_DigsX_Cont();
void force_To_TenXY_Y_From_DigsX_Entry();
void force_To_TenXY_Y_From_DigsX_Signed();
void force_To_TenXY_Y_From_MatX();
void force_To_TenXY_Y_From_MatX_Signed_Cont();
void force_To_TenXY_Y_From_MatX_Signed_Entry();
void force_To_TenXY_Y_From_SignX_Cont();
void force_To_TenXY_Y_From_SignX_Entry();
void force_To_TenXY_Y_From_TenXY();
void force_To_TenXY_Y_From_TenXY_Signed_Cont();
void force_To_TenXY_Y_From_TenXY_Signed_Cont_X();
void force_To_TenXY_Y_From_TenXY_Signed_Entry();
void force_To_TenXY_Y_From_Vec();
void force_To_TenXY_Y_From_Vec_Signed();

ForceFuncDesc descTable[] = {
	{delayCont, "delayCont", 2, ARG_NEITHER},
	{force_Alt_Eval, "force_Alt_Eval", 2, ARG_NEITHER},
	{force_From_Bool_X_Only_Cont, "force_From_Bool_X_Only_Cont", 2, ARG_NEITHER},
	{force_From_Bool_X_Only_Entry, "force_From_Bool_X_Only_Entry", 2, ARG_NEITHER},
	{force_From_Bool_Y_Only_Cont, "force_From_Bool_Y_Only_Cont", 2, ARG_NEITHER},
	{force_From_Bool_Y_Only_Entry, "force_From_Bool_Y_Only_Entry", 2, ARG_NEITHER},
	{forceGtEqZero_To_PredX_From_Alt_Cont, "forceGtEqZero_To_PredX_From_Alt_Cont", 2, ARG_NEITHER},
	{forceGtEqZero_To_PredX_From_Alt_Entry, "forceGtEqZero_To_PredX_From_Alt_Entry", 2, ARG_NEITHER},
	{forceGtEqZero_To_PredX_From_Cls_Cont, "forceGtEqZero_To_PredX_From_Cls_Cont", 2, ARG_NEITHER},
	{forceGtEqZero_To_PredX_From_Cls_Entry, "forceGtEqZero_To_PredX_From_Cls_Entry", 2, ARG_NEITHER},
	{forceGtEqZero_To_PredX_From_DigsX_SINF_Cont, "forceGtEqZero_To_PredX_From_DigsX_SINF_Cont", 2, ARG_NEITHER},
	{forceGtEqZero_To_PredX_From_DigsX_SINF_Entry, "forceGtEqZero_To_PredX_From_DigsX_SINF_Entry", 2, ARG_NEITHER},
	{forceGtEqZero_To_PredX_From_DigsX_SNEG_Cont, "forceGtEqZero_To_PredX_From_DigsX_SNEG_Cont", 2, ARG_NEITHER},
	{forceGtEqZero_To_PredX_From_DigsX_SNEG_Entry, "forceGtEqZero_To_PredX_From_DigsX_SNEG_Entry", 2, ARG_NEITHER},
	{forceGtEqZero_To_PredX_From_DigsX_SZERO_Cont, "forceGtEqZero_To_PredX_From_DigsX_SZERO_Cont", 2, ARG_NEITHER},
	{forceGtEqZero_To_PredX_From_DigsX_SZERO_Entry, "forceGtEqZero_To_PredX_From_DigsX_SZERO_Entry", 2, ARG_NEITHER},
	{forceGtEqZero_To_PredX_From_MatX_Cont, "forceGtEqZero_To_PredX_From_MatX_Cont", 2, ARG_NEITHER},
	{forceGtEqZero_To_PredX_From_MatX_Entry, "forceGtEqZero_To_PredX_From_MatX_Entry", 2, ARG_NEITHER},
	{forceGtEqZero_To_PredX_From_MatX_Signed_Cont, "forceGtEqZero_To_PredX_From_MatX_Signed_Cont", 2, ARG_NEITHER},
	{forceGtEqZero_To_PredX_From_MatX_Signed_Entry, "forceGtEqZero_To_PredX_From_MatX_Signed_Entry", 2, ARG_NEITHER},
	{forceGtEqZero_To_PredX_From_SignX_Cont, "forceGtEqZero_To_PredX_From_SignX_Cont", 2, ARG_NEITHER},
	{forceGtEqZero_To_PredX_From_SignX_Entry, "forceGtEqZero_To_PredX_From_SignX_Entry", 2, ARG_NEITHER},
	{forceGtEqZero_To_PredX_From_TenXY, "forceGtEqZero_To_PredX_From_TenXY", 2, ARG_NEITHER},
	{forceGtZero_To_PredX_From_Alt_Cont, "forceGtZero_To_PredX_From_Alt_Cont", 2, ARG_NEITHER},
	{forceGtZero_To_PredX_From_Alt_Entry, "forceGtZero_To_PredX_From_Alt_Entry", 2, ARG_NEITHER},
	{forceGtZero_To_PredX_From_Cls_Cont, "forceGtZero_To_PredX_From_Cls_Cont", 2, ARG_NEITHER},
	{forceGtZero_To_PredX_From_Cls_Entry, "forceGtZero_To_PredX_From_Cls_Entry", 2, ARG_NEITHER},
	{forceGtZero_To_PredX_From_DigsX_SPOS_Cont, "forceGtZero_To_PredX_From_DigsX_SPOS_Cont", 2, ARG_NEITHER},
	{forceGtZero_To_PredX_From_DigsX_SPOS_Entry, "forceGtZero_To_PredX_From_DigsX_SPOS_Entry", 2, ARG_NEITHER},
	{forceGtZero_To_PredX_From_DigsX_SZERO_Cont, "forceGtZero_To_PredX_From_DigsX_SZERO_Cont", 2, ARG_NEITHER},
	{forceGtZero_To_PredX_From_DigsX_SZERO_Entry, "forceGtZero_To_PredX_From_DigsX_SZERO_Entry", 2, ARG_NEITHER},
	{forceGtZero_To_PredX_From_MatX_Cont, "forceGtZero_To_PredX_From_MatX_Cont", 2, ARG_NEITHER},
	{forceGtZero_To_PredX_From_MatX_Entry, "forceGtZero_To_PredX_From_MatX_Entry", 2, ARG_NEITHER},
	{forceGtZero_To_PredX_From_MatX_Signed_Cont, "forceGtZero_To_PredX_From_MatX_Signed_Cont", 2, ARG_NEITHER},
	{forceGtZero_To_PredX_From_MatX_Signed_Entry, "forceGtZero_To_PredX_From_MatX_Signed_Entry", 2, ARG_NEITHER},
	{forceGtZero_To_PredX_From_SignX_Cont, "forceGtZero_To_PredX_From_SignX_Cont", 2, ARG_NEITHER},
	{forceGtZero_To_PredX_From_SignX_Entry, "forceGtZero_To_PredX_From_SignX_Entry", 2, ARG_NEITHER},
	{forceGtZero_To_PredX_From_TenXY, "forceGtZero_To_PredX_From_TenXY", 2, ARG_NEITHER},
	{force_To_Alt_Cont, "force_To_Alt_Cont", 2, ARG_NEITHER},
	{force_To_Alt_Entry, "force_To_Alt_Entry", 2, ARG_NEITHER},
	{force_To_Alt_From_The_Abyss, "force_To_Alt_From_The_Abyss", 2, ARG_NEITHER},
	{force_To_Alt_Reduce, "force_To_Alt_Reduce", 2, ARG_NEITHER},
	{force_To_And_From_Bool_X_Cont, "force_To_And_From_Bool_X_Cont", 2, ARG_X},
	{force_To_And_From_Bool_X_Entry, "force_To_And_From_Bool_X_Entry", 2, ARG_X},
	{force_To_And_From_Bool_Y_Cont, "force_To_And_From_Bool_Y_Cont", 2, ARG_Y},
	{force_To_And_From_Bool_Y_Entry, "force_To_And_From_Bool_Y_Entry", 2, ARG_Y},
	{force_To_Bool_From_The_Abyss, "force_To_Bool_From_The_Abyss", 2, ARG_NEITHER},
	{force_To_DigsX_From_Alt_Cont, "force_To_DigsX_From_Alt_Cont", 3, ARG_NEITHER},
	{force_To_DigsX_From_Alt_Entry, "force_To_DigsX_From_Alt_Entry", 3, ARG_NEITHER},
	{force_To_DigsX_From_Cls_Cont, "force_To_DigsX_From_Cls_Cont", 2, ARG_NEITHER},
	{force_To_DigsX_From_Cls_Entry, "force_To_DigsX_From_Cls_Entry", 2, ARG_NEITHER},
	{force_To_DigsX_From_DigsX_Cont, "force_To_DigsX_From_DigsX_Cont", 3, ARG_NEITHER},
	{force_To_DigsX_From_DigsX_Entry, "force_To_DigsX_From_DigsX_Entry", 3, ARG_NEITHER},
	{force_To_DigsX_From_MatX_Cont, "force_To_DigsX_From_MatX_Cont", 3, ARG_NEITHER},
	{force_To_DigsX_From_MatX_Entry, "force_To_DigsX_From_MatX_Entry", 3, ARG_NEITHER},
	{force_To_DigsX_From_TenXY_Cont, "force_To_DigsX_From_TenXY_Cont", 3, ARG_NEITHER},
	{force_To_DigsX_From_TenXY_Cont_X, "force_To_DigsX_From_TenXY_Cont_X", 3, ARG_NEITHER},
	{force_To_DigsX_From_TenXY_Entry, "force_To_DigsX_From_TenXY_Entry", 3, ARG_NEITHER},
	{force_To_DigsX_From_Vec, "force_To_DigsX_From_Vec", 3, ARG_NEITHER},
	{force_To_MatX_From_Alt_Cont, "force_To_MatX_From_Alt_Cont", 3, ARG_NEITHER},
	{force_To_MatX_From_Alt_Entry, "force_To_MatX_From_Alt_Entry", 3, ARG_NEITHER},
	{force_To_MatX_From_Alt_Signed_Cont, "force_To_MatX_From_Alt_Signed_Cont", 2, ARG_NEITHER},
	{force_To_MatX_From_Alt_Signed_Entry, "force_To_MatX_From_Alt_Signed_Entry", 2, ARG_NEITHER},
	{force_To_MatX_From_Cls_Cont, "force_To_MatX_From_Cls_Cont", 2, ARG_NEITHER},
	{force_To_MatX_From_Cls_Entry, "force_To_MatX_From_Cls_Entry", 2, ARG_NEITHER},
	{force_To_MatX_From_Cls_Signed_Cont, "force_To_MatX_From_Cls_Signed_Cont", 2, ARG_NEITHER},
	{force_To_MatX_From_Cls_Signed_Entry, "force_To_MatX_From_Cls_Signed_Entry", 2, ARG_NEITHER},
	{force_To_MatX_From_DigsX_Cont, "force_To_MatX_From_DigsX_Cont", 3, ARG_NEITHER},
	{force_To_MatX_From_DigsX_Entry, "force_To_MatX_From_DigsX_Entry", 3, ARG_NEITHER},
	{force_To_MatX_From_DigsX_Signed, "force_To_MatX_From_DigsX_Signed", 2, ARG_NEITHER},
	{force_To_MatX_From_MatX, "force_To_MatX_From_MatX", 3, ARG_NEITHER},
	{force_To_MatX_From_MatX_Signed_Cont, "force_To_MatX_From_MatX_Signed_Cont", 2, ARG_NEITHER},
	{force_To_MatX_From_MatX_Signed_Entry, "force_To_MatX_From_MatX_Signed_Entry", 2, ARG_NEITHER},
	{force_To_MatX_From_SignX_Cont, "force_To_MatX_From_SignX_Cont", 2, ARG_NEITHER},
	{force_To_MatX_From_SignX_Entry, "force_To_MatX_From_SignX_Entry", 2, ARG_NEITHER},
	{force_To_MatX_From_TenXY, "force_To_MatX_From_TenXY", 3, ARG_NEITHER},
	{force_To_MatX_From_TenXY_Signed_Cont, "force_To_MatX_From_TenXY_Signed_Cont", 2, ARG_NEITHER},
	{force_To_MatX_From_TenXY_Signed_Cont_X, "force_To_MatX_From_TenXY_Signed_Cont_X", 2, ARG_NEITHER},
	{force_To_MatX_From_TenXY_Signed_Entry, "force_To_MatX_From_TenXY_Signed_Entry", 2, ARG_NEITHER},
	{force_To_MatX_From_Vec, "force_To_MatX_From_Vec", 3, ARG_NEITHER},
	{force_To_MatX_From_Vec_Signed, "force_To_MatX_From_Vec_Signed", 2, ARG_NEITHER},
	{force_To_Not_From_Bool_Cont, "force_To_Not_From_Bool_Cont", 2, ARG_NEITHER},
	{force_To_Not_From_Bool_Entry, "force_To_Not_From_Bool_Entry", 2, ARG_NEITHER},
	{force_To_Or_From_Bool_X_Cont, "force_To_Or_From_Bool_X_Cont", 2, ARG_X},
	{force_To_Or_From_Bool_X_Entry, "force_To_Or_From_Bool_X_Entry", 2, ARG_X},
	{force_To_Or_From_Bool_Y_Cont, "force_To_Or_From_Bool_Y_Cont", 2, ARG_Y},
	{force_To_Or_From_Bool_Y_Entry, "force_To_Or_From_Bool_Y_Entry", 2, ARG_Y},
	{force_To_PredX_From_DigsX_2n_minus_1_False_Cont, "force_To_PredX_From_DigsX_2n_minus_1_False_Cont", 2, ARG_NEITHER},
	{force_To_PredX_From_DigsX_2n_minus_1_False_Entry, "force_To_PredX_From_DigsX_2n_minus_1_False_Entry", 2, ARG_NEITHER},
	{force_To_PredX_From_DigsX_2n_minus_1_True_Cont, "force_To_PredX_From_DigsX_2n_minus_1_True_Cont", 2, ARG_NEITHER},
	{force_To_PredX_From_DigsX_2n_minus_1_True_Entry, "force_To_PredX_From_DigsX_2n_minus_1_True_Entry", 2, ARG_NEITHER},
	{force_To_PredX_From_DigsX_minus_2n_minus_1_False_Cont, "force_To_PredX_From_DigsX_minus_2n_minus_1_False_Cont", 2, ARG_NEITHER},
	{force_To_PredX_From_DigsX_minus_2n_minus_1_False_Entry, "force_To_PredX_From_DigsX_minus_2n_minus_1_False_Entry", 2, ARG_NEITHER},
	{force_To_PredX_From_DigsX_minus_2n_minus_1_True_Cont, "force_To_PredX_From_DigsX_minus_2n_minus_1_True_Cont", 2, ARG_NEITHER},
	{force_To_PredX_From_DigsX_minus_2n_minus_1_True_Entry, "force_To_PredX_From_DigsX_minus_2n_minus_1_True_Entry", 2, ARG_NEITHER},
	{force_To_PredX_From_The_Abyss, "force_To_PredX_From_The_Abyss", 2, ARG_NEITHER},
	{force_To_SignX_From_Alt_Cont, "force_To_SignX_From_Alt_Cont", 2, ARG_NEITHER},
	{force_To_SignX_From_Alt_Entry, "force_To_SignX_From_Alt_Entry", 2, ARG_NEITHER},
	{force_To_SignX_From_Cls_Cont, "force_To_SignX_From_Cls_Cont", 2, ARG_NEITHER},
	{force_To_SignX_From_Cls_Entry, "force_To_SignX_From_Cls_Entry", 2, ARG_NEITHER},
	{force_To_SignX_From_DigsX, "force_To_SignX_From_DigsX", 2, ARG_NEITHER},
	{force_To_SignX_From_MatX_Cont, "force_To_SignX_From_MatX_Cont", 2, ARG_NEITHER},
	{force_To_SignX_From_MatX_Entry, "force_To_SignX_From_MatX_Entry", 2, ARG_NEITHER},
	{force_To_SignX_From_TenXY_Cont, "force_To_SignX_From_TenXY_Cont", 2, ARG_NEITHER},
	{force_To_SignX_From_TenXY_Cont_X, "force_To_SignX_From_TenXY_Cont_X", 2, ARG_NEITHER},
	{force_To_SignX_From_TenXY_Entry, "force_To_SignX_From_TenXY_Entry", 2, ARG_NEITHER},
	{force_To_SignX_From_Vec, "force_To_SignX_From_Vec", 2, ARG_NEITHER},
	{force_To_TenXY_X_From_Alt_Cont, "force_To_TenXY_X_From_Alt_Cont", 3, ARG_X},
	{force_To_TenXY_X_From_Alt_Entry, "force_To_TenXY_X_From_Alt_Entry", 3, ARG_X},
	{force_To_TenXY_X_From_Alt_Signed_Cont, "force_To_TenXY_X_From_Alt_Signed_Cont", 2, ARG_X},
	{force_To_TenXY_X_From_Alt_Signed_Entry, "force_To_TenXY_X_From_Alt_Signed_Entry", 2, ARG_X},
	{force_To_TenXY_X_From_Cls_Cont, "force_To_TenXY_X_From_Cls_Cont", 3, ARG_X},
	{force_To_TenXY_X_From_Cls_Entry, "force_To_TenXY_X_From_Cls_Entry", 3, ARG_X},
	{force_To_TenXY_X_From_Cls_Signed_Cont, "force_To_TenXY_X_From_Cls_Signed_Cont", 2, ARG_X},
	{force_To_TenXY_X_From_Cls_Signed_Entry, "force_To_TenXY_X_From_Cls_Signed_Entry", 2, ARG_X},
	{force_To_TenXY_X_From_DigsX_Cont, "force_To_TenXY_X_From_DigsX_Cont", 3, ARG_X},
	{force_To_TenXY_X_From_DigsX_Entry, "force_To_TenXY_X_From_DigsX_Entry", 3, ARG_X},
	{force_To_TenXY_X_From_DigsX_Signed, "force_To_TenXY_X_From_DigsX_Signed", 2, ARG_X},
	{force_To_TenXY_X_From_MatX, "force_To_TenXY_X_From_MatX", 3, ARG_X},
	{force_To_TenXY_X_From_MatX_Signed_Cont, "force_To_TenXY_X_From_MatX_Signed_Cont", 2, ARG_X},
	{force_To_TenXY_X_From_MatX_Signed_Entry, "force_To_TenXY_X_From_MatX_Signed_Entry", 2, ARG_X},
	{force_To_TenXY_X_From_SignX_Cont, "force_To_TenXY_X_From_SignX_Cont", 2, ARG_X},
	{force_To_TenXY_X_From_SignX_Entry, "force_To_TenXY_X_From_SignX_Entry", 2, ARG_X},
	{force_To_TenXY_X_From_TenXY, "force_To_TenXY_X_From_TenXY", 3, ARG_X},
	{force_To_TenXY_X_From_TenXY_Signed_Cont, "force_To_TenXY_X_From_TenXY_Signed_Cont", 2, ARG_X},
	{force_To_TenXY_X_From_TenXY_Signed_Cont_X, "force_To_TenXY_X_From_TenXY_Signed_Cont_X", 2, ARG_X},
	{force_To_TenXY_X_From_TenXY_Signed_Entry, "force_To_TenXY_X_From_TenXY_Signed_Entry", 2, ARG_X},
	{force_To_TenXY_X_From_Vec, "force_To_TenXY_X_From_Vec", 3, ARG_X},
	{force_To_TenXY_X_From_Vec_Signed, "force_To_TenXY_X_From_Vec_Signed", 2, ARG_X},
	{force_To_TenXY_X_Until_Refining, "force_To_TenXY_X_Until_Refining", 2, ARG_X},
	{force_To_TenXY_Y_From_Alt_Cont, "force_To_TenXY_Y_From_Alt_Cont", 3, ARG_Y},
	{force_To_TenXY_Y_From_Alt_Entry, "force_To_TenXY_Y_From_Alt_Entry", 3, ARG_Y},
	{force_To_TenXY_Y_From_Alt_Signed_Cont, "force_To_TenXY_Y_From_Alt_Signed_Cont", 2, ARG_Y},
	{force_To_TenXY_Y_From_Alt_Signed_Entry, "force_To_TenXY_Y_From_Alt_Signed_Entry", 2, ARG_Y},
	{force_To_TenXY_Y_From_Cls_Cont, "force_To_TenXY_Y_From_Cls_Cont", 3, ARG_Y},
	{force_To_TenXY_Y_From_Cls_Entry, "force_To_TenXY_Y_From_Cls_Entry", 3, ARG_Y},
	{force_To_TenXY_Y_From_Cls_Signed_Cont, "force_To_TenXY_Y_From_Cls_Signed_Cont", 2, ARG_Y},
	{force_To_TenXY_Y_From_Cls_Signed_Entry, "force_To_TenXY_Y_From_Cls_Signed_Entry", 2, ARG_Y},
	{force_To_TenXY_Y_From_DigsX_Cont, "force_To_TenXY_Y_From_DigsX_Cont", 3, ARG_Y},
	{force_To_TenXY_Y_From_DigsX_Entry, "force_To_TenXY_Y_From_DigsX_Entry", 3, ARG_Y},
	{force_To_TenXY_Y_From_DigsX_Signed, "force_To_TenXY_Y_From_DigsX_Signed", 2, ARG_Y},
	{force_To_TenXY_Y_From_MatX, "force_To_TenXY_Y_From_MatX", 3, ARG_Y},
	{force_To_TenXY_Y_From_MatX_Signed_Cont, "force_To_TenXY_Y_From_MatX_Signed_Cont", 2, ARG_Y},
	{force_To_TenXY_Y_From_MatX_Signed_Entry, "force_To_TenXY_Y_From_MatX_Signed_Entry", 2, ARG_Y},
	{force_To_TenXY_Y_From_SignX_Cont, "force_To_TenXY_Y_From_SignX_Cont", 2, ARG_Y},
	{force_To_TenXY_Y_From_SignX_Entry, "force_To_TenXY_Y_From_SignX_Entry", 2, ARG_Y},
	{force_To_TenXY_Y_From_TenXY, "force_To_TenXY_Y_From_TenXY", 3, ARG_Y},
	{force_To_TenXY_Y_From_TenXY_Signed_Cont, "force_To_TenXY_Y_From_TenXY_Signed_Cont", 2, ARG_Y},
	{force_To_TenXY_Y_From_TenXY_Signed_Cont_X, "force_To_TenXY_Y_From_TenXY_Signed_Cont_X", 2, ARG_Y},
	{force_To_TenXY_Y_From_TenXY_Signed_Entry, "force_To_TenXY_Y_From_TenXY_Signed_Entry", 2, ARG_Y},
	{force_To_TenXY_Y_From_Vec, "force_To_TenXY_Y_From_Vec", 3, ARG_Y},
	{force_To_TenXY_Y_From_Vec_Signed, "force_To_TenXY_Y_From_Vec_Signed", 2, ARG_Y}
};

void *root;

/*
 * Compares the functions in two descriptors.
 */
static int
compare(const void *p, const void *q)
{
	if (((ForceFuncDesc *) p)->func < ((ForceFuncDesc *) q)->func)
		return -1;
	if (((ForceFuncDesc *) p)->func > ((ForceFuncDesc *) q)->func)
		return 1;
	return 0;
}

/*
 * Retrieves the descriptor for the specified function.
 */
ForceFuncDesc *
getDescForForceFunc(void (*func)())
{
	void *p;
	ForceFuncDesc fd = {NULL, "no-name", 0};

	fd.func = func;
	p = tfind((void *) &fd, (void **) &root, compare);
	if (p == NULL)
		return (ForceFuncDesc *) NULL;
	else
		return *((ForceFuncDesc **) p);
}

/*
 * The table above only lists those continuation/force functions pertaining
 * to bools, signs and digits for the basic types of objects in the heap.
 * These include lfts and booleans. It does not include any information
 * relating to user defined continutions/closures. These include the closures
 * for analytic functions which generate more matrices and tensors when 
 * forced. The function below allows one to add descriptive information
 * for user-defined continuations. 
 */
void
registerForceFunc(void (*func)(), char *funcName, int nArgs)
{
	ForceFuncDesc *p;

	if ((p = (ForceFuncDesc *) malloc(sizeof(ForceFuncDesc))) == NULL)
		Error(FATAL, E_INT, "registerForceMethod", "malloc failed");

	p->func = func;
	p->funcName = funcName;
	p->nArgs = nArgs;
	p->argXOrY = ARG_NEITHER;

	(void) tsearch((void *) p, &root, compare);
}

void
initForceMethodLookupTable()
{
	int i;
	void *p;

	root = NULL;
	for (i = 0; i < sizeof(descTable) / sizeof(ForceFuncDesc); i++) {
		p = tsearch((void *) (descTable + i), &root, compare);
		if (p == NULL)
			Error(FATAL, E_INT, "initForceMethodLookupTable",
						"tsearch returned NULL");
	}
}
