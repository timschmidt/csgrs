-- parser produced by Happy Version 1.5


module Parser (
-- Types
Exp, Exp1, Term, Factor,
Token,

-- Functions
parse

  ) where

import ParseTree
import Lexer


data HappyAbsSyn t1 t2 t3 t4 t5 t6 t7 t8 t9
	= HappyTerminal Token
	| HappyErrorToken Int
	| HappyAbsSyn1 t1
	| HappyAbsSyn2 t2
	| HappyAbsSyn3 t3
	| HappyAbsSyn4 t4
	| HappyAbsSyn5 t5
	| HappyAbsSyn6 t6
	| HappyAbsSyn7 t7
	| HappyAbsSyn8 t8
	| HappyAbsSyn9 t9

action_0 (10) = happyShift action_8
action_0 (11) = happyShift action_9
action_0 (12) = happyShift action_10
action_0 (13) = happyShift action_11
action_0 (14) = happyShift action_12
action_0 (15) = happyShift action_13
action_0 (16) = happyShift action_14
action_0 (17) = happyShift action_15
action_0 (18) = happyShift action_16
action_0 (19) = happyShift action_17
action_0 (20) = happyShift action_18
action_0 (21) = happyShift action_19
action_0 (22) = happyShift action_20
action_0 (23) = happyShift action_21
action_0 (26) = happyShift action_22
action_0 (29) = happyShift action_23
action_0 (1) = happyGoto action_1
action_0 (2) = happyGoto action_2
action_0 (3) = happyGoto action_3
action_0 (4) = happyGoto action_4
action_0 (5) = happyGoto action_5
action_0 (8) = happyGoto action_6
action_0 (9) = happyGoto action_7
action_0 _ = happyReduce_5

action_1 (34) = happyAccept
action_1 _ = happyFail

action_2 (25) = happyShift action_42
action_2 (26) = happyShift action_43
action_2 _ = happyReduce_3

action_3 (27) = happyShift action_40
action_3 (28) = happyShift action_41
action_3 _ = happyReduce_8

action_4 (31) = happyShift action_39
action_4 _ = happyReduce_12

action_5 _ = happyReduce_14

action_6 _ = happyReduce_16

action_7 _ = happyReduce_17

action_8 (24) = happyShift action_38
action_8 _ = happyFail

action_9 _ = happyReduce_4

action_10 _ = happyReduce_15

action_11 _ = happyReduce_30

action_12 (24) = happyShift action_37
action_12 _ = happyReduce_31

action_13 (29) = happyShift action_31
action_13 (6) = happyGoto action_36
action_13 _ = happyFail

action_14 (29) = happyShift action_31
action_14 (6) = happyGoto action_35
action_14 _ = happyFail

action_15 (29) = happyShift action_31
action_15 (6) = happyGoto action_34
action_15 _ = happyFail

action_16 (29) = happyShift action_31
action_16 (6) = happyGoto action_33
action_16 _ = happyFail

action_17 (29) = happyShift action_31
action_17 (6) = happyGoto action_32
action_17 _ = happyFail

action_18 (29) = happyShift action_31
action_18 (6) = happyGoto action_30
action_18 _ = happyFail

action_19 (29) = happyShift action_28
action_19 (7) = happyGoto action_29
action_19 _ = happyFail

action_20 (29) = happyShift action_28
action_20 (7) = happyGoto action_27
action_20 _ = happyFail

action_21 _ = happyReduce_26

action_22 (12) = happyShift action_10
action_22 (13) = happyShift action_11
action_22 (14) = happyShift action_25
action_22 (15) = happyShift action_13
action_22 (16) = happyShift action_14
action_22 (17) = happyShift action_15
action_22 (18) = happyShift action_16
action_22 (19) = happyShift action_17
action_22 (20) = happyShift action_18
action_22 (21) = happyShift action_19
action_22 (22) = happyShift action_20
action_22 (23) = happyShift action_21
action_22 (29) = happyShift action_23
action_22 (5) = happyGoto action_26
action_22 (8) = happyGoto action_6
action_22 (9) = happyGoto action_7
action_22 _ = happyFail

action_23 (12) = happyShift action_10
action_23 (13) = happyShift action_11
action_23 (14) = happyShift action_25
action_23 (15) = happyShift action_13
action_23 (16) = happyShift action_14
action_23 (17) = happyShift action_15
action_23 (18) = happyShift action_16
action_23 (19) = happyShift action_17
action_23 (20) = happyShift action_18
action_23 (21) = happyShift action_19
action_23 (22) = happyShift action_20
action_23 (23) = happyShift action_21
action_23 (26) = happyShift action_22
action_23 (29) = happyShift action_23
action_23 (2) = happyGoto action_24
action_23 (3) = happyGoto action_3
action_23 (4) = happyGoto action_4
action_23 (5) = happyGoto action_5
action_23 (8) = happyGoto action_6
action_23 (9) = happyGoto action_7
action_23 _ = happyFail

action_24 (25) = happyShift action_42
action_24 (26) = happyShift action_43
action_24 (30) = happyShift action_53
action_24 _ = happyFail

action_25 _ = happyReduce_31

action_26 _ = happyReduce_13

action_27 _ = happyReduce_24

action_28 (12) = happyShift action_10
action_28 (13) = happyShift action_11
action_28 (14) = happyShift action_25
action_28 (15) = happyShift action_13
action_28 (16) = happyShift action_14
action_28 (17) = happyShift action_15
action_28 (18) = happyShift action_16
action_28 (19) = happyShift action_17
action_28 (20) = happyShift action_18
action_28 (21) = happyShift action_19
action_28 (22) = happyShift action_20
action_28 (23) = happyShift action_21
action_28 (26) = happyShift action_22
action_28 (29) = happyShift action_23
action_28 (2) = happyGoto action_52
action_28 (3) = happyGoto action_3
action_28 (4) = happyGoto action_4
action_28 (5) = happyGoto action_5
action_28 (8) = happyGoto action_6
action_28 (9) = happyGoto action_7
action_28 _ = happyFail

action_29 _ = happyReduce_25

action_30 _ = happyReduce_23

action_31 (12) = happyShift action_10
action_31 (13) = happyShift action_11
action_31 (14) = happyShift action_25
action_31 (15) = happyShift action_13
action_31 (16) = happyShift action_14
action_31 (17) = happyShift action_15
action_31 (18) = happyShift action_16
action_31 (19) = happyShift action_17
action_31 (20) = happyShift action_18
action_31 (21) = happyShift action_19
action_31 (22) = happyShift action_20
action_31 (23) = happyShift action_21
action_31 (26) = happyShift action_22
action_31 (29) = happyShift action_23
action_31 (2) = happyGoto action_51
action_31 (3) = happyGoto action_3
action_31 (4) = happyGoto action_4
action_31 (5) = happyGoto action_5
action_31 (8) = happyGoto action_6
action_31 (9) = happyGoto action_7
action_31 _ = happyFail

action_32 _ = happyReduce_21

action_33 _ = happyReduce_22

action_34 _ = happyReduce_20

action_35 _ = happyReduce_19

action_36 _ = happyReduce_18

action_37 (12) = happyShift action_10
action_37 (13) = happyShift action_11
action_37 (14) = happyShift action_25
action_37 (15) = happyShift action_13
action_37 (16) = happyShift action_14
action_37 (17) = happyShift action_15
action_37 (18) = happyShift action_16
action_37 (19) = happyShift action_17
action_37 (20) = happyShift action_18
action_37 (21) = happyShift action_19
action_37 (22) = happyShift action_20
action_37 (23) = happyShift action_21
action_37 (26) = happyShift action_22
action_37 (29) = happyShift action_23
action_37 (2) = happyGoto action_50
action_37 (3) = happyGoto action_3
action_37 (4) = happyGoto action_4
action_37 (5) = happyGoto action_5
action_37 (8) = happyGoto action_6
action_37 (9) = happyGoto action_7
action_37 _ = happyFail

action_38 (13) = happyShift action_49
action_38 _ = happyFail

action_39 (12) = happyShift action_10
action_39 (13) = happyShift action_11
action_39 (14) = happyShift action_25
action_39 (15) = happyShift action_13
action_39 (16) = happyShift action_14
action_39 (17) = happyShift action_15
action_39 (18) = happyShift action_16
action_39 (19) = happyShift action_17
action_39 (20) = happyShift action_18
action_39 (21) = happyShift action_19
action_39 (22) = happyShift action_20
action_39 (23) = happyShift action_21
action_39 (26) = happyShift action_22
action_39 (29) = happyShift action_23
action_39 (4) = happyGoto action_48
action_39 (5) = happyGoto action_5
action_39 (8) = happyGoto action_6
action_39 (9) = happyGoto action_7
action_39 _ = happyFail

action_40 (12) = happyShift action_10
action_40 (13) = happyShift action_11
action_40 (14) = happyShift action_25
action_40 (15) = happyShift action_13
action_40 (16) = happyShift action_14
action_40 (17) = happyShift action_15
action_40 (18) = happyShift action_16
action_40 (19) = happyShift action_17
action_40 (20) = happyShift action_18
action_40 (21) = happyShift action_19
action_40 (22) = happyShift action_20
action_40 (23) = happyShift action_21
action_40 (26) = happyShift action_22
action_40 (29) = happyShift action_23
action_40 (4) = happyGoto action_47
action_40 (5) = happyGoto action_5
action_40 (8) = happyGoto action_6
action_40 (9) = happyGoto action_7
action_40 _ = happyFail

action_41 (12) = happyShift action_10
action_41 (13) = happyShift action_11
action_41 (14) = happyShift action_25
action_41 (15) = happyShift action_13
action_41 (16) = happyShift action_14
action_41 (17) = happyShift action_15
action_41 (18) = happyShift action_16
action_41 (19) = happyShift action_17
action_41 (20) = happyShift action_18
action_41 (21) = happyShift action_19
action_41 (22) = happyShift action_20
action_41 (23) = happyShift action_21
action_41 (26) = happyShift action_22
action_41 (29) = happyShift action_23
action_41 (4) = happyGoto action_46
action_41 (5) = happyGoto action_5
action_41 (8) = happyGoto action_6
action_41 (9) = happyGoto action_7
action_41 _ = happyFail

action_42 (12) = happyShift action_10
action_42 (13) = happyShift action_11
action_42 (14) = happyShift action_25
action_42 (15) = happyShift action_13
action_42 (16) = happyShift action_14
action_42 (17) = happyShift action_15
action_42 (18) = happyShift action_16
action_42 (19) = happyShift action_17
action_42 (20) = happyShift action_18
action_42 (21) = happyShift action_19
action_42 (22) = happyShift action_20
action_42 (23) = happyShift action_21
action_42 (26) = happyShift action_22
action_42 (29) = happyShift action_23
action_42 (3) = happyGoto action_45
action_42 (4) = happyGoto action_4
action_42 (5) = happyGoto action_5
action_42 (8) = happyGoto action_6
action_42 (9) = happyGoto action_7
action_42 _ = happyFail

action_43 (12) = happyShift action_10
action_43 (13) = happyShift action_11
action_43 (14) = happyShift action_25
action_43 (15) = happyShift action_13
action_43 (16) = happyShift action_14
action_43 (17) = happyShift action_15
action_43 (18) = happyShift action_16
action_43 (19) = happyShift action_17
action_43 (20) = happyShift action_18
action_43 (21) = happyShift action_19
action_43 (22) = happyShift action_20
action_43 (23) = happyShift action_21
action_43 (26) = happyShift action_22
action_43 (29) = happyShift action_23
action_43 (3) = happyGoto action_44
action_43 (4) = happyGoto action_4
action_43 (5) = happyGoto action_5
action_43 (8) = happyGoto action_6
action_43 (9) = happyGoto action_7
action_43 _ = happyFail

action_44 (27) = happyShift action_40
action_44 (28) = happyShift action_41
action_44 _ = happyReduce_7

action_45 (27) = happyShift action_40
action_45 (28) = happyShift action_41
action_45 _ = happyReduce_6

action_46 _ = happyReduce_10

action_47 _ = happyReduce_9

action_48 _ = happyReduce_11

action_49 _ = happyReduce_1

action_50 (25) = happyShift action_42
action_50 (26) = happyShift action_43
action_50 _ = happyReduce_2

action_51 (25) = happyShift action_42
action_51 (26) = happyShift action_43
action_51 (30) = happyShift action_55
action_51 _ = happyFail

action_52 (25) = happyShift action_42
action_52 (26) = happyShift action_43
action_52 (32) = happyShift action_54
action_52 _ = happyFail

action_53 _ = happyReduce_27

action_54 (12) = happyShift action_10
action_54 (13) = happyShift action_11
action_54 (14) = happyShift action_25
action_54 (15) = happyShift action_13
action_54 (16) = happyShift action_14
action_54 (17) = happyShift action_15
action_54 (18) = happyShift action_16
action_54 (19) = happyShift action_17
action_54 (20) = happyShift action_18
action_54 (21) = happyShift action_19
action_54 (22) = happyShift action_20
action_54 (23) = happyShift action_21
action_54 (26) = happyShift action_22
action_54 (29) = happyShift action_23
action_54 (2) = happyGoto action_56
action_54 (3) = happyGoto action_3
action_54 (4) = happyGoto action_4
action_54 (5) = happyGoto action_5
action_54 (8) = happyGoto action_6
action_54 (9) = happyGoto action_7
action_54 _ = happyFail

action_55 _ = happyReduce_28

action_56 (25) = happyShift action_42
action_56 (26) = happyShift action_43
action_56 (30) = happyShift action_57
action_56 _ = happyFail

action_57 _ = happyReduce_29

happyReduce_1 = happySpecReduce_3 1 reduction where {
  reduction
	(HappyTerminal (TokenIntnum happy_var_3))
	_
	_
	 =  HappyAbsSyn1
		 (Digits happy_var_3);
  reduction _ _ _  = notHappyAtAll }

happyReduce_2 = happySpecReduce_3 1 reduction where {
  reduction
	(HappyAbsSyn2  happy_var_3)
	_
	(HappyTerminal (TokenVar happy_var_1))
	 =  HappyAbsSyn1
		 (Assign happy_var_1 happy_var_3);
  reduction _ _ _  = notHappyAtAll }

happyReduce_3 = happySpecReduce_1 1 reduction where {
  reduction
	(HappyAbsSyn2  happy_var_1)
	 =  HappyAbsSyn1
		 (Exp1 happy_var_1);
  reduction _  = notHappyAtAll }

happyReduce_4 = happySpecReduce_1 1 reduction where {
  reduction
	_
	 =  HappyAbsSyn1
		 (Exit)}

happyReduce_5 = happySpecReduce_0 1 reduction where {
  reduction
	 =  HappyAbsSyn1
		 (NoExp)}

happyReduce_6 = happySpecReduce_3 2 reduction where {
  reduction
	(HappyAbsSyn3  happy_var_3)
	_
	(HappyAbsSyn2  happy_var_1)
	 =  HappyAbsSyn2
		 (Plus happy_var_1 happy_var_3);
  reduction _ _ _  = notHappyAtAll }

happyReduce_7 = happySpecReduce_3 2 reduction where {
  reduction
	(HappyAbsSyn3  happy_var_3)
	_
	(HappyAbsSyn2  happy_var_1)
	 =  HappyAbsSyn2
		 (Minus happy_var_1 happy_var_3);
  reduction _ _ _  = notHappyAtAll }

happyReduce_8 = happySpecReduce_1 2 reduction where {
  reduction
	(HappyAbsSyn3  happy_var_1)
	 =  HappyAbsSyn2
		 (Term happy_var_1);
  reduction _  = notHappyAtAll }

happyReduce_9 = happySpecReduce_3 3 reduction where {
  reduction
	(HappyAbsSyn4  happy_var_3)
	_
	(HappyAbsSyn3  happy_var_1)
	 =  HappyAbsSyn3
		 (Times happy_var_1 happy_var_3);
  reduction _ _ _  = notHappyAtAll }

happyReduce_10 = happySpecReduce_3 3 reduction where {
  reduction
	(HappyAbsSyn4  happy_var_3)
	_
	(HappyAbsSyn3  happy_var_1)
	 =  HappyAbsSyn3
		 (Div happy_var_1 happy_var_3);
  reduction _ _ _  = notHappyAtAll }

happyReduce_11 = happySpecReduce_3 3 reduction where {
  reduction
	(HappyAbsSyn4  happy_var_3)
	_
	(HappyAbsSyn4  happy_var_1)
	 =  HappyAbsSyn3
		 (Pow happy_var_1 happy_var_3);
  reduction _ _ _  = notHappyAtAll }

happyReduce_12 = happySpecReduce_1 3 reduction where {
  reduction
	(HappyAbsSyn4  happy_var_1)
	 =  HappyAbsSyn3
		 (NFactor happy_var_1);
  reduction _  = notHappyAtAll }

happyReduce_13 = happySpecReduce_2 4 reduction where {
  reduction
	(HappyAbsSyn5  happy_var_2)
	_
	 =  HappyAbsSyn4
		 (Neg happy_var_2);
  reduction _ _  = notHappyAtAll }

happyReduce_14 = happySpecReduce_1 4 reduction where {
  reduction
	(HappyAbsSyn5  happy_var_1)
	 =  HappyAbsSyn4
		 (Factor happy_var_1);
  reduction _  = notHappyAtAll }

happyReduce_15 = happySpecReduce_1 5 reduction where {
  reduction
	(HappyTerminal (TokenNum happy_var_1))
	 =  HappyAbsSyn5
		 (Num happy_var_1);
  reduction _  = notHappyAtAll }

happyReduce_16 = happySpecReduce_1 5 reduction where {
  reduction
	(HappyAbsSyn8  happy_var_1)
	 =  HappyAbsSyn5
		 (Intnum happy_var_1);
  reduction _  = notHappyAtAll }

happyReduce_17 = happySpecReduce_1 5 reduction where {
  reduction
	(HappyAbsSyn9  happy_var_1)
	 =  HappyAbsSyn5
		 (Var happy_var_1);
  reduction _  = notHappyAtAll }

happyReduce_18 = happySpecReduce_2 5 reduction where {
  reduction
	(HappyAbsSyn6  happy_var_2)
	_
	 =  HappyAbsSyn5
		 (Sin happy_var_2);
  reduction _ _  = notHappyAtAll }

happyReduce_19 = happySpecReduce_2 5 reduction where {
  reduction
	(HappyAbsSyn6  happy_var_2)
	_
	 =  HappyAbsSyn5
		 (Cos happy_var_2);
  reduction _ _  = notHappyAtAll }

happyReduce_20 = happySpecReduce_2 5 reduction where {
  reduction
	(HappyAbsSyn6  happy_var_2)
	_
	 =  HappyAbsSyn5
		 (Arctan happy_var_2);
  reduction _ _  = notHappyAtAll }

happyReduce_21 = happySpecReduce_2 5 reduction where {
  reduction
	(HappyAbsSyn6  happy_var_2)
	_
	 =  HappyAbsSyn5
		 (Exponential happy_var_2);
  reduction _ _  = notHappyAtAll }

happyReduce_22 = happySpecReduce_2 5 reduction where {
  reduction
	(HappyAbsSyn6  happy_var_2)
	_
	 =  HappyAbsSyn5
		 (Ln happy_var_2);
  reduction _ _  = notHappyAtAll }

happyReduce_23 = happySpecReduce_2 5 reduction where {
  reduction
	(HappyAbsSyn6  happy_var_2)
	_
	 =  HappyAbsSyn5
		 (Sqrt happy_var_2);
  reduction _ _  = notHappyAtAll }

happyReduce_24 = happySpecReduce_2 5 reduction where {
  reduction
	(HappyAbsSyn7  happy_var_2)
	_
	 =  HappyAbsSyn5
		 (Min happy_var_2);
  reduction _ _  = notHappyAtAll }

happyReduce_25 = happySpecReduce_2 5 reduction where {
  reduction
	(HappyAbsSyn7  happy_var_2)
	_
	 =  HappyAbsSyn5
		 (Max happy_var_2);
  reduction _ _  = notHappyAtAll }

happyReduce_26 = happySpecReduce_1 5 reduction where {
  reduction
	_
	 =  HappyAbsSyn5
		 (Pi)}

happyReduce_27 = happySpecReduce_3 5 reduction where {
  reduction
	_
	(HappyAbsSyn2  happy_var_2)
	_
	 =  HappyAbsSyn5
		 (Brack happy_var_2);
  reduction _ _ _  = notHappyAtAll }

happyReduce_28 = happySpecReduce_3 6 reduction where {
  reduction
	_
	(HappyAbsSyn2  happy_var_2)
	_
	 =  HappyAbsSyn6
		 (happy_var_2);
  reduction _ _ _  = notHappyAtAll }

happyReduce_29 = happyReduce 5 7 reduction where {
  reduction
	(_ :
	(HappyAbsSyn2  happy_var_4) :
	_ :
	(HappyAbsSyn2  happy_var_2) :
	_ :
	happyRest)
	 = HappyAbsSyn7
		 ((happy_var_2, happy_var_4)) : happyRest;
  reduction _ = notHappyAtAll }

happyReduce_30 = happySpecReduce_1 8 reduction where {
  reduction
	(HappyTerminal (TokenIntnum happy_var_1))
	 =  HappyAbsSyn8
		 (An_Intnum happy_var_1);
  reduction _  = notHappyAtAll }

happyReduce_31 = happySpecReduce_1 9 reduction where {
  reduction
	(HappyTerminal (TokenVar happy_var_1))
	 =  HappyAbsSyn9
		 (A_Var happy_var_1);
  reduction _  = notHappyAtAll }

happyNewToken action sts stk [] =
	action 34 34 (error "reading EOF!") (HappyState action) sts stk []

happyNewToken action sts stk (tk:tks) =
	let cont i = action i i tk (HappyState action) sts stk tks in
	case tk of {
	TokenDigits -> cont 10;
	TokenExit -> cont 11;
	TokenNum happy_dollar_dollar -> cont 12;
	TokenIntnum happy_dollar_dollar -> cont 13;
	TokenVar happy_dollar_dollar -> cont 14;
	TokenSin -> cont 15;
	TokenCos -> cont 16;
	TokenArctan -> cont 17;
	TokenLn -> cont 18;
	TokenExp -> cont 19;
	TokenSqrt -> cont 20;
	TokenMax -> cont 21;
	TokenMin -> cont 22;
	TokenPi -> cont 23;
	TokenAssign -> cont 24;
	TokenPlus -> cont 25;
	TokenMinus -> cont 26;
	TokenTimes -> cont 27;
	TokenDiv -> cont 28;
	TokenOB -> cont 29;
	TokenCB -> cont 30;
	TokenPow -> cont 31;
	TokenComma -> cont 32;
	Err happy_dollar_dollar -> cont 33;
	}

happyThen = \m k -> k m
happyReturn = \a tks -> a
parser = happyParse



happyError :: [Token] -> Exp
happyError _ = Error			-- Add an error token

parse = parser . lexer

-- $Id: HappyTemplate,v 1.8 1997/12/04 15:07:21 simonm Exp $

{-
	The stack is in the following order throughout the parse:

	i	current token number
	j	another copy of this to avoid messing with the stack
	tk	current token semantic value
	st	current state
	sts	state stack
	stk	semantic stack
-}

-----------------------------------------------------------------------------

happyParse = happyNewToken action_0 [] []

-- All this HappyState stuff is simply because we can't have recursive
-- types in Haskell without an intervening data structure.

newtype HappyState b c = HappyState
        (Int ->                         -- token number
         Int ->                         -- token number (yes, again)
         b ->                           -- token semantic value
         HappyState b c ->              -- current state
         [HappyState b c] ->            -- state stack
         c)

-----------------------------------------------------------------------------
-- Accepting the parse

happyAccept j tk st sts [ HappyAbsSyn1 ans ] = happyReturn ans
happyAccept j tk st sts _                    = notHappyAtAll

-----------------------------------------------------------------------------
-- Shifting a token

happyShift new_state (-1) tk st sts stk@(HappyErrorToken i : _) =
--     _trace "shifting the error token" $
     new_state i i tk (HappyState new_state) (st:sts) stk

happyShift new_state i tk st sts stk =
     happyNewToken new_state (st:sts) (HappyTerminal tk:stk)

-----------------------------------------------------------------------------
-- Reducing

-- happyReduce is specialised for the common cases.

-- don't allow reductions when we're in error recovery, because this can
-- lead to an infinite loop.

happySpecReduce_0 i fn (-1) tk _ sts stk
     = case sts of
	st@(HappyState action):sts -> action (-1) (-1) tk st sts stk
	_ -> happyError
happySpecReduce_0 i fn j tk st@(HappyState action) sts stk
     = action i j tk st (st:sts) (fn : stk)

happySpecReduce_1 i fn (-1) tk _ (st@(HappyState action):sts) stk
     = action (-1) (-1) tk st sts stk
happySpecReduce_1 i fn j tk _ sts@(st@(HappyState action):_) (v1:stk')
     = action i j tk st sts (fn v1 : stk')
happySpecReduce_1 _ _ _ _ _ _ _
     = notHappyAtAll

happySpecReduce_2 i fn (-1) tk _ (st@(HappyState action):sts) stk
     = action (-1) (-1) tk st sts stk
happySpecReduce_2 i fn j tk _ (_:sts@(st@(HappyState action):_)) (v1:v2:stk')
     = action i j tk st sts (fn v1 v2 : stk')
happySpecReduce_2 _ _ _ _ _ _ _
     = notHappyAtAll

happySpecReduce_3 i fn (-1) tk _ (st@(HappyState action):sts) stk
     = action (-1) (-1) tk st sts stk
happySpecReduce_3 i fn j tk _ (_:_:sts@(st@(HappyState action):_)) 
	(v1:v2:v3:stk')
     = action i j tk st sts (fn v1 v2 v3 : stk')
happySpecReduce_3 _ _ _ _ _ _ _
     = notHappyAtAll

happyReduce k i fn (-1) tk _ (st@(HappyState action):sts) stk
     = action (-1) (-1) tk st sts stk
happyReduce k i fn j tk st sts stk = action i j tk st' sts' (fn stk)
       where sts'@(st'@(HappyState action):_) = drop (k::Int) (st:sts)

happyMonadReduce k i c fn (-1) tk _ sts stk
      = case sts of
	     (st@(HappyState action):sts) -> action (-1) (-1) tk st sts stk
	     [] -> happyError
happyMonadReduce k i c fn j tk st sts stk =
	happyThen (fn stk) (\r -> action i j tk st' sts' (c r : stk'))
       where sts'@(st'@(HappyState action):_) = drop (k::Int) (st:sts)
	     stk' = drop (k::Int) stk

-----------------------------------------------------------------------------
-- Moving to a new state after a reduction

happyGoto action j tk st = action j j tk (HappyState action)

-----------------------------------------------------------------------------
-- Error recovery (-1 is the error token)

-- fail if we are in recovery and no more states to discard
happyFail  (-1) tk st' [] stk = happyError

-- discard a state
happyFail  (-1) tk st' (st@(HappyState action):sts) stk =
--	_trace "discarding state" $
	action (-1) (-1) tk st sts stk

-- Enter error recovery: generate an error token,
-- 			 save the old token and carry on.

-- we push the error token on the stack in anticipation of a shift,
-- and also because this is a convenient place to store the saved token.

happyFail  i tk st@(HappyState action) sts stk =
--	_trace "entering error recovery" $
	action (-1) (-1) tk st sts (HappyErrorToken i : stk)

-- Internal happy errors:

notHappyAtAll = error "Internal Happy error\n"

-- end of Happy Template.
