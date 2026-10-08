-- parser produced by Happy Version 1.5


module IParser (
-- Types
Exp, Exp1, Term, Factor,
Token,

-- Functions
parse

  ) where

import IParseTree
import ILexer


data HappyAbsSyn t1 t2 t3 t4 t5 t6 t7 t8 t9 t10
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
	| HappyAbsSyn10 t10

action_0 (11) = happyShift action_8
action_0 (12) = happyShift action_9
action_0 (13) = happyShift action_10
action_0 (14) = happyShift action_11
action_0 (15) = happyShift action_12
action_0 (16) = happyShift action_13
action_0 (17) = happyShift action_14
action_0 (18) = happyShift action_15
action_0 (19) = happyShift action_16
action_0 (20) = happyShift action_17
action_0 (21) = happyShift action_18
action_0 (22) = happyShift action_19
action_0 (23) = happyShift action_20
action_0 (24) = happyShift action_21
action_0 (25) = happyShift action_22
action_0 (26) = happyShift action_23
action_0 (28) = happyShift action_24
action_0 (31) = happyShift action_25
action_0 (34) = happyShift action_26
action_0 (1) = happyGoto action_1
action_0 (2) = happyGoto action_2
action_0 (3) = happyGoto action_3
action_0 (4) = happyGoto action_4
action_0 (5) = happyGoto action_5
action_0 (9) = happyGoto action_6
action_0 (10) = happyGoto action_7
action_0 _ = happyReduce_5

action_1 (37) = happyAccept
action_1 _ = happyFail

action_2 (30) = happyShift action_48
action_2 (31) = happyShift action_49
action_2 _ = happyReduce_3

action_3 (32) = happyShift action_46
action_3 (33) = happyShift action_47
action_3 _ = happyReduce_8

action_4 _ = happyReduce_11

action_5 _ = happyReduce_13

action_6 _ = happyReduce_16

action_7 _ = happyReduce_15

action_8 (29) = happyShift action_45
action_8 _ = happyFail

action_9 _ = happyReduce_4

action_10 _ = happyReduce_14

action_11 _ = happyReduce_34

action_12 (29) = happyShift action_44
action_12 _ = happyReduce_33

action_13 (34) = happyShift action_38
action_13 (6) = happyGoto action_43
action_13 _ = happyFail

action_14 (34) = happyShift action_38
action_14 (6) = happyGoto action_42
action_14 _ = happyFail

action_15 (34) = happyShift action_38
action_15 (6) = happyGoto action_41
action_15 _ = happyFail

action_16 (34) = happyShift action_38
action_16 (6) = happyGoto action_40
action_16 _ = happyFail

action_17 (34) = happyShift action_38
action_17 (6) = happyGoto action_39
action_17 _ = happyFail

action_18 (34) = happyShift action_38
action_18 (6) = happyGoto action_37
action_18 _ = happyFail

action_19 (34) = happyShift action_35
action_19 (7) = happyGoto action_36
action_19 _ = happyFail

action_20 (34) = happyShift action_35
action_20 (7) = happyGoto action_34
action_20 _ = happyFail

action_21 (34) = happyShift action_31
action_21 (8) = happyGoto action_33
action_21 _ = happyFail

action_22 (34) = happyShift action_31
action_22 (8) = happyGoto action_32
action_22 _ = happyFail

action_23 (34) = happyShift action_31
action_23 (8) = happyGoto action_30
action_23 _ = happyFail

action_24 _ = happyReduce_17

action_25 (13) = happyShift action_10
action_25 (14) = happyShift action_11
action_25 (15) = happyShift action_28
action_25 (16) = happyShift action_13
action_25 (17) = happyShift action_14
action_25 (18) = happyShift action_15
action_25 (19) = happyShift action_16
action_25 (20) = happyShift action_17
action_25 (21) = happyShift action_18
action_25 (22) = happyShift action_19
action_25 (23) = happyShift action_20
action_25 (24) = happyShift action_21
action_25 (25) = happyShift action_22
action_25 (26) = happyShift action_23
action_25 (28) = happyShift action_24
action_25 (34) = happyShift action_26
action_25 (5) = happyGoto action_29
action_25 (9) = happyGoto action_6
action_25 (10) = happyGoto action_7
action_25 _ = happyFail

action_26 (13) = happyShift action_10
action_26 (14) = happyShift action_11
action_26 (15) = happyShift action_28
action_26 (16) = happyShift action_13
action_26 (17) = happyShift action_14
action_26 (18) = happyShift action_15
action_26 (19) = happyShift action_16
action_26 (20) = happyShift action_17
action_26 (21) = happyShift action_18
action_26 (22) = happyShift action_19
action_26 (23) = happyShift action_20
action_26 (24) = happyShift action_21
action_26 (25) = happyShift action_22
action_26 (26) = happyShift action_23
action_26 (28) = happyShift action_24
action_26 (31) = happyShift action_25
action_26 (34) = happyShift action_26
action_26 (2) = happyGoto action_27
action_26 (3) = happyGoto action_3
action_26 (4) = happyGoto action_4
action_26 (5) = happyGoto action_5
action_26 (9) = happyGoto action_6
action_26 (10) = happyGoto action_7
action_26 _ = happyFail

action_27 (30) = happyShift action_48
action_27 (31) = happyShift action_49
action_27 (35) = happyShift action_59
action_27 _ = happyFail

action_28 _ = happyReduce_33

action_29 _ = happyReduce_12

action_30 _ = happyReduce_28

action_31 (13) = happyShift action_10
action_31 (14) = happyShift action_11
action_31 (15) = happyShift action_28
action_31 (16) = happyShift action_13
action_31 (17) = happyShift action_14
action_31 (18) = happyShift action_15
action_31 (19) = happyShift action_16
action_31 (20) = happyShift action_17
action_31 (21) = happyShift action_18
action_31 (22) = happyShift action_19
action_31 (23) = happyShift action_20
action_31 (24) = happyShift action_21
action_31 (25) = happyShift action_22
action_31 (26) = happyShift action_23
action_31 (28) = happyShift action_24
action_31 (31) = happyShift action_25
action_31 (34) = happyShift action_26
action_31 (2) = happyGoto action_58
action_31 (3) = happyGoto action_3
action_31 (4) = happyGoto action_4
action_31 (5) = happyGoto action_5
action_31 (9) = happyGoto action_6
action_31 (10) = happyGoto action_7
action_31 _ = happyFail

action_32 _ = happyReduce_27

action_33 _ = happyReduce_29

action_34 _ = happyReduce_25

action_35 (13) = happyShift action_10
action_35 (14) = happyShift action_11
action_35 (15) = happyShift action_28
action_35 (16) = happyShift action_13
action_35 (17) = happyShift action_14
action_35 (18) = happyShift action_15
action_35 (19) = happyShift action_16
action_35 (20) = happyShift action_17
action_35 (21) = happyShift action_18
action_35 (22) = happyShift action_19
action_35 (23) = happyShift action_20
action_35 (24) = happyShift action_21
action_35 (25) = happyShift action_22
action_35 (26) = happyShift action_23
action_35 (28) = happyShift action_24
action_35 (31) = happyShift action_25
action_35 (34) = happyShift action_26
action_35 (2) = happyGoto action_57
action_35 (3) = happyGoto action_3
action_35 (4) = happyGoto action_4
action_35 (5) = happyGoto action_5
action_35 (9) = happyGoto action_6
action_35 (10) = happyGoto action_7
action_35 _ = happyFail

action_36 _ = happyReduce_26

action_37 _ = happyReduce_24

action_38 (13) = happyShift action_10
action_38 (14) = happyShift action_11
action_38 (15) = happyShift action_28
action_38 (16) = happyShift action_13
action_38 (17) = happyShift action_14
action_38 (18) = happyShift action_15
action_38 (19) = happyShift action_16
action_38 (20) = happyShift action_17
action_38 (21) = happyShift action_18
action_38 (22) = happyShift action_19
action_38 (23) = happyShift action_20
action_38 (24) = happyShift action_21
action_38 (25) = happyShift action_22
action_38 (26) = happyShift action_23
action_38 (28) = happyShift action_24
action_38 (31) = happyShift action_25
action_38 (34) = happyShift action_26
action_38 (2) = happyGoto action_56
action_38 (3) = happyGoto action_3
action_38 (4) = happyGoto action_4
action_38 (5) = happyGoto action_5
action_38 (9) = happyGoto action_6
action_38 (10) = happyGoto action_7
action_38 _ = happyFail

action_39 _ = happyReduce_22

action_40 _ = happyReduce_23

action_41 _ = happyReduce_21

action_42 _ = happyReduce_20

action_43 _ = happyReduce_19

action_44 (13) = happyShift action_10
action_44 (14) = happyShift action_11
action_44 (15) = happyShift action_28
action_44 (16) = happyShift action_13
action_44 (17) = happyShift action_14
action_44 (18) = happyShift action_15
action_44 (19) = happyShift action_16
action_44 (20) = happyShift action_17
action_44 (21) = happyShift action_18
action_44 (22) = happyShift action_19
action_44 (23) = happyShift action_20
action_44 (24) = happyShift action_21
action_44 (25) = happyShift action_22
action_44 (26) = happyShift action_23
action_44 (28) = happyShift action_24
action_44 (31) = happyShift action_25
action_44 (34) = happyShift action_26
action_44 (2) = happyGoto action_55
action_44 (3) = happyGoto action_3
action_44 (4) = happyGoto action_4
action_44 (5) = happyGoto action_5
action_44 (9) = happyGoto action_6
action_44 (10) = happyGoto action_7
action_44 _ = happyFail

action_45 (14) = happyShift action_54
action_45 _ = happyFail

action_46 (13) = happyShift action_10
action_46 (14) = happyShift action_11
action_46 (15) = happyShift action_28
action_46 (16) = happyShift action_13
action_46 (17) = happyShift action_14
action_46 (18) = happyShift action_15
action_46 (19) = happyShift action_16
action_46 (20) = happyShift action_17
action_46 (21) = happyShift action_18
action_46 (22) = happyShift action_19
action_46 (23) = happyShift action_20
action_46 (24) = happyShift action_21
action_46 (25) = happyShift action_22
action_46 (26) = happyShift action_23
action_46 (28) = happyShift action_24
action_46 (31) = happyShift action_25
action_46 (34) = happyShift action_26
action_46 (4) = happyGoto action_53
action_46 (5) = happyGoto action_5
action_46 (9) = happyGoto action_6
action_46 (10) = happyGoto action_7
action_46 _ = happyFail

action_47 (13) = happyShift action_10
action_47 (14) = happyShift action_11
action_47 (15) = happyShift action_28
action_47 (16) = happyShift action_13
action_47 (17) = happyShift action_14
action_47 (18) = happyShift action_15
action_47 (19) = happyShift action_16
action_47 (20) = happyShift action_17
action_47 (21) = happyShift action_18
action_47 (22) = happyShift action_19
action_47 (23) = happyShift action_20
action_47 (24) = happyShift action_21
action_47 (25) = happyShift action_22
action_47 (26) = happyShift action_23
action_47 (28) = happyShift action_24
action_47 (31) = happyShift action_25
action_47 (34) = happyShift action_26
action_47 (4) = happyGoto action_52
action_47 (5) = happyGoto action_5
action_47 (9) = happyGoto action_6
action_47 (10) = happyGoto action_7
action_47 _ = happyFail

action_48 (13) = happyShift action_10
action_48 (14) = happyShift action_11
action_48 (15) = happyShift action_28
action_48 (16) = happyShift action_13
action_48 (17) = happyShift action_14
action_48 (18) = happyShift action_15
action_48 (19) = happyShift action_16
action_48 (20) = happyShift action_17
action_48 (21) = happyShift action_18
action_48 (22) = happyShift action_19
action_48 (23) = happyShift action_20
action_48 (24) = happyShift action_21
action_48 (25) = happyShift action_22
action_48 (26) = happyShift action_23
action_48 (28) = happyShift action_24
action_48 (31) = happyShift action_25
action_48 (34) = happyShift action_26
action_48 (3) = happyGoto action_51
action_48 (4) = happyGoto action_4
action_48 (5) = happyGoto action_5
action_48 (9) = happyGoto action_6
action_48 (10) = happyGoto action_7
action_48 _ = happyFail

action_49 (13) = happyShift action_10
action_49 (14) = happyShift action_11
action_49 (15) = happyShift action_28
action_49 (16) = happyShift action_13
action_49 (17) = happyShift action_14
action_49 (18) = happyShift action_15
action_49 (19) = happyShift action_16
action_49 (20) = happyShift action_17
action_49 (21) = happyShift action_18
action_49 (22) = happyShift action_19
action_49 (23) = happyShift action_20
action_49 (24) = happyShift action_21
action_49 (25) = happyShift action_22
action_49 (26) = happyShift action_23
action_49 (28) = happyShift action_24
action_49 (31) = happyShift action_25
action_49 (34) = happyShift action_26
action_49 (3) = happyGoto action_50
action_49 (4) = happyGoto action_4
action_49 (5) = happyGoto action_5
action_49 (9) = happyGoto action_6
action_49 (10) = happyGoto action_7
action_49 _ = happyFail

action_50 (32) = happyShift action_46
action_50 (33) = happyShift action_47
action_50 _ = happyReduce_7

action_51 (32) = happyShift action_46
action_51 (33) = happyShift action_47
action_51 _ = happyReduce_6

action_52 _ = happyReduce_10

action_53 _ = happyReduce_9

action_54 _ = happyReduce_1

action_55 (30) = happyShift action_48
action_55 (31) = happyShift action_49
action_55 _ = happyReduce_2

action_56 (30) = happyShift action_48
action_56 (31) = happyShift action_49
action_56 (35) = happyShift action_62
action_56 _ = happyFail

action_57 (27) = happyShift action_61
action_57 (30) = happyShift action_48
action_57 (31) = happyShift action_49
action_57 _ = happyFail

action_58 (27) = happyShift action_60
action_58 (30) = happyShift action_48
action_58 (31) = happyShift action_49
action_58 _ = happyFail

action_59 _ = happyReduce_18

action_60 (15) = happyShift action_28
action_60 (9) = happyGoto action_64
action_60 _ = happyFail

action_61 (13) = happyShift action_10
action_61 (14) = happyShift action_11
action_61 (15) = happyShift action_28
action_61 (16) = happyShift action_13
action_61 (17) = happyShift action_14
action_61 (18) = happyShift action_15
action_61 (19) = happyShift action_16
action_61 (20) = happyShift action_17
action_61 (21) = happyShift action_18
action_61 (22) = happyShift action_19
action_61 (23) = happyShift action_20
action_61 (24) = happyShift action_21
action_61 (25) = happyShift action_22
action_61 (26) = happyShift action_23
action_61 (28) = happyShift action_24
action_61 (31) = happyShift action_25
action_61 (34) = happyShift action_26
action_61 (2) = happyGoto action_63
action_61 (3) = happyGoto action_3
action_61 (4) = happyGoto action_4
action_61 (5) = happyGoto action_5
action_61 (9) = happyGoto action_6
action_61 (10) = happyGoto action_7
action_61 _ = happyFail

action_62 _ = happyReduce_30

action_63 (30) = happyShift action_48
action_63 (31) = happyShift action_49
action_63 (35) = happyShift action_66
action_63 _ = happyFail

action_64 (27) = happyShift action_65
action_64 _ = happyFail

action_65 (13) = happyShift action_10
action_65 (14) = happyShift action_11
action_65 (15) = happyShift action_28
action_65 (16) = happyShift action_13
action_65 (17) = happyShift action_14
action_65 (18) = happyShift action_15
action_65 (19) = happyShift action_16
action_65 (20) = happyShift action_17
action_65 (21) = happyShift action_18
action_65 (22) = happyShift action_19
action_65 (23) = happyShift action_20
action_65 (24) = happyShift action_21
action_65 (25) = happyShift action_22
action_65 (26) = happyShift action_23
action_65 (28) = happyShift action_24
action_65 (31) = happyShift action_25
action_65 (34) = happyShift action_26
action_65 (2) = happyGoto action_67
action_65 (3) = happyGoto action_3
action_65 (4) = happyGoto action_4
action_65 (5) = happyGoto action_5
action_65 (9) = happyGoto action_6
action_65 (10) = happyGoto action_7
action_65 _ = happyFail

action_66 _ = happyReduce_31

action_67 (27) = happyShift action_68
action_67 (30) = happyShift action_48
action_67 (31) = happyShift action_49
action_67 _ = happyFail

action_68 (13) = happyShift action_10
action_68 (14) = happyShift action_11
action_68 (15) = happyShift action_28
action_68 (16) = happyShift action_13
action_68 (17) = happyShift action_14
action_68 (18) = happyShift action_15
action_68 (19) = happyShift action_16
action_68 (20) = happyShift action_17
action_68 (21) = happyShift action_18
action_68 (22) = happyShift action_19
action_68 (23) = happyShift action_20
action_68 (24) = happyShift action_21
action_68 (25) = happyShift action_22
action_68 (26) = happyShift action_23
action_68 (28) = happyShift action_24
action_68 (31) = happyShift action_25
action_68 (34) = happyShift action_26
action_68 (2) = happyGoto action_69
action_68 (3) = happyGoto action_3
action_68 (4) = happyGoto action_4
action_68 (5) = happyGoto action_5
action_68 (9) = happyGoto action_6
action_68 (10) = happyGoto action_7
action_68 _ = happyFail

action_69 (30) = happyShift action_48
action_69 (31) = happyShift action_49
action_69 (35) = happyShift action_70
action_69 _ = happyFail

action_70 _ = happyReduce_32

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

happyReduce_11 = happySpecReduce_1 3 reduction where {
  reduction
	(HappyAbsSyn4  happy_var_1)
	 =  HappyAbsSyn3
		 (NFactor happy_var_1);
  reduction _  = notHappyAtAll }

happyReduce_12 = happySpecReduce_2 4 reduction where {
  reduction
	(HappyAbsSyn5  happy_var_2)
	_
	 =  HappyAbsSyn4
		 (Neg happy_var_2);
  reduction _ _  = notHappyAtAll }

happyReduce_13 = happySpecReduce_1 4 reduction where {
  reduction
	(HappyAbsSyn5  happy_var_1)
	 =  HappyAbsSyn4
		 (Factor happy_var_1);
  reduction _  = notHappyAtAll }

happyReduce_14 = happySpecReduce_1 5 reduction where {
  reduction
	(HappyTerminal (TokenNum happy_var_1))
	 =  HappyAbsSyn5
		 (Num happy_var_1);
  reduction _  = notHappyAtAll }

happyReduce_15 = happySpecReduce_1 5 reduction where {
  reduction
	(HappyAbsSyn10  happy_var_1)
	 =  HappyAbsSyn5
		 (Intnum happy_var_1);
  reduction _  = notHappyAtAll }

happyReduce_16 = happySpecReduce_1 5 reduction where {
  reduction
	(HappyAbsSyn9  happy_var_1)
	 =  HappyAbsSyn5
		 (Var happy_var_1);
  reduction _  = notHappyAtAll }

happyReduce_17 = happySpecReduce_1 5 reduction where {
  reduction
	_
	 =  HappyAbsSyn5
		 (Pi)}

happyReduce_18 = happySpecReduce_3 5 reduction where {
  reduction
	_
	(HappyAbsSyn2  happy_var_2)
	_
	 =  HappyAbsSyn5
		 (Brack happy_var_2);
  reduction _ _ _  = notHappyAtAll }

happyReduce_19 = happySpecReduce_2 5 reduction where {
  reduction
	(HappyAbsSyn6  happy_var_2)
	_
	 =  HappyAbsSyn5
		 (Sin happy_var_2);
  reduction _ _  = notHappyAtAll }

happyReduce_20 = happySpecReduce_2 5 reduction where {
  reduction
	(HappyAbsSyn6  happy_var_2)
	_
	 =  HappyAbsSyn5
		 (Cos happy_var_2);
  reduction _ _  = notHappyAtAll }

happyReduce_21 = happySpecReduce_2 5 reduction where {
  reduction
	(HappyAbsSyn6  happy_var_2)
	_
	 =  HappyAbsSyn5
		 (Arctan happy_var_2);
  reduction _ _  = notHappyAtAll }

happyReduce_22 = happySpecReduce_2 5 reduction where {
  reduction
	(HappyAbsSyn6  happy_var_2)
	_
	 =  HappyAbsSyn5
		 (Exponential happy_var_2);
  reduction _ _  = notHappyAtAll }

happyReduce_23 = happySpecReduce_2 5 reduction where {
  reduction
	(HappyAbsSyn6  happy_var_2)
	_
	 =  HappyAbsSyn5
		 (Ln happy_var_2);
  reduction _ _  = notHappyAtAll }

happyReduce_24 = happySpecReduce_2 5 reduction where {
  reduction
	(HappyAbsSyn6  happy_var_2)
	_
	 =  HappyAbsSyn5
		 (Sqrt happy_var_2);
  reduction _ _  = notHappyAtAll }

happyReduce_25 = happySpecReduce_2 5 reduction where {
  reduction
	(HappyAbsSyn7  happy_var_2)
	_
	 =  HappyAbsSyn5
		 (Min happy_var_2);
  reduction _ _  = notHappyAtAll }

happyReduce_26 = happySpecReduce_2 5 reduction where {
  reduction
	(HappyAbsSyn7  happy_var_2)
	_
	 =  HappyAbsSyn5
		 (Max happy_var_2);
  reduction _ _  = notHappyAtAll }

happyReduce_27 = happySpecReduce_2 5 reduction where {
  reduction
	(HappyAbsSyn8  happy_var_2)
	_
	 =  HappyAbsSyn5
		 (FMax happy_var_2);
  reduction _ _  = notHappyAtAll }

happyReduce_28 = happySpecReduce_2 5 reduction where {
  reduction
	(HappyAbsSyn8  happy_var_2)
	_
	 =  HappyAbsSyn5
		 (FMin happy_var_2);
  reduction _ _  = notHappyAtAll }

happyReduce_29 = happySpecReduce_2 5 reduction where {
  reduction
	(HappyAbsSyn8  happy_var_2)
	_
	 =  HappyAbsSyn5
		 (Integrate happy_var_2);
  reduction _ _  = notHappyAtAll }

happyReduce_30 = happySpecReduce_3 6 reduction where {
  reduction
	_
	(HappyAbsSyn2  happy_var_2)
	_
	 =  HappyAbsSyn6
		 (happy_var_2);
  reduction _ _ _  = notHappyAtAll }

happyReduce_31 = happyReduce 5 7 reduction where {
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

happyReduce_32 = happyReduce 9 8 reduction where {
  reduction
	(_ :
	(HappyAbsSyn2  happy_var_8) :
	_ :
	(HappyAbsSyn2  happy_var_6) :
	_ :
	(HappyAbsSyn9  happy_var_4) :
	_ :
	(HappyAbsSyn2  happy_var_2) :
	_ :
	happyRest)
	 = HappyAbsSyn8
		 ((happy_var_2, happy_var_4, happy_var_6, happy_var_8)) : happyRest;
  reduction _ = notHappyAtAll }

happyReduce_33 = happySpecReduce_1 9 reduction where {
  reduction
	(HappyTerminal (TokenVar happy_var_1))
	 =  HappyAbsSyn9
		 (A_Var happy_var_1);
  reduction _  = notHappyAtAll }

happyReduce_34 = happySpecReduce_1 10 reduction where {
  reduction
	(HappyTerminal (TokenIntnum happy_var_1))
	 =  HappyAbsSyn10
		 (An_Intnum happy_var_1);
  reduction _  = notHappyAtAll }

happyNewToken action sts stk [] =
	action 37 37 (error "reading EOF!") (HappyState action) sts stk []

happyNewToken action sts stk (tk:tks) =
	let cont i = action i i tk (HappyState action) sts stk tks in
	case tk of {
	TokenDigits -> cont 11;
	TokenExit -> cont 12;
	TokenNum happy_dollar_dollar -> cont 13;
	TokenIntnum happy_dollar_dollar -> cont 14;
	TokenVar happy_dollar_dollar -> cont 15;
	TokenSin -> cont 16;
	TokenCos -> cont 17;
	TokenArctan -> cont 18;
	TokenLn -> cont 19;
	TokenExp -> cont 20;
	TokenSqrt -> cont 21;
	TokenMax -> cont 22;
	TokenMin -> cont 23;
	TokenIntegr -> cont 24;
	TokenFMax -> cont 25;
	TokenFMin -> cont 26;
	TokenComma -> cont 27;
	TokenPi -> cont 28;
	TokenAssign -> cont 29;
	TokenPlus -> cont 30;
	TokenMinus -> cont 31;
	TokenTimes -> cont 32;
	TokenDiv -> cont 33;
	TokenOB -> cont 34;
	TokenCB -> cont 35;
	Err happy_dollar_dollar -> cont 36;
	}

happyThen = \m k -> k m
happyReturn = \a tks -> a
parser = happyParse



happyError :: [Token] -> Exp
happyError _ = Error        -- Add error token

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
