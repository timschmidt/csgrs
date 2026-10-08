{-|
    Module      :  Numeric.AERN.Basics.Laws.Utilities
    Description :  utilities for easier definitions of algebraic laws 
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Utilities for easier definitions of algebraic laws.
-}
module Numeric.AERN.Basics.Laws.Utilities where

import Numeric.AERN.Basics.Effort

type Expr1 t = t -> t
type Expr2 t = t -> t -> t
type Expr3 t = t -> t -> t -> t

type Expr1Eff ei t = ei -> t -> t
type Expr2Eff ei t = ei -> t -> t -> t
type Expr3Eff ei t = ei -> t -> t -> t -> t

type UnaryOp t = t -> t
type Op t = t -> t -> t
type PartOp t = t -> t -> Maybe t

type UnaryOpEff ei t = ei -> t -> t
type OpEff ei t = ei -> t -> t -> t
type PartOpEff ei t = ei -> t -> t -> Maybe t

type Rel t = t -> t -> Bool
type PRel t = t -> t -> Maybe Bool
type PRelEff ei t = ei -> t -> t -> Maybe Bool

type Expr1Op1 t = (Op t) -> Expr1 t
type Expr1Op2 t = (Op t) -> Expr2 t
type Expr1Op3 t = (Op t) -> Expr3 t
type Expr2Op1 t = (Op t) -> (Op t) -> Expr1 t
type Expr2Op2 t = (Op t) -> (Op t) -> Expr2 t
type Expr2Op3 t = (Op t) -> (Op t) -> Expr3 t

type Expr1UnaryOp1Eff ei t = (UnaryOpEff ei t) -> Expr1Eff ei t
type Expr1Op1Eff ei t = (OpEff ei t) -> Expr1Eff ei t
type Expr1Op2Eff ei t = (OpEff ei t) -> Expr2Eff ei t
type Expr1Op3Eff ei t = (OpEff ei t) -> Expr3Eff ei t
type Expr2Op1Eff ei1 ei2 t = (OpEff ei1 t) -> (OpEff ei2 t) -> Expr1Eff (ei1, ei2) t
type Expr2Op2Eff ei1 ei2 t = (OpEff ei1 t) -> (OpEff ei2 t) -> Expr2Eff (ei1, ei2) t
type Expr2Op3Eff ei1 ei2 t = (OpEff ei1 t) -> (OpEff ei2 t) -> Expr3Eff (ei1, ei2) t
