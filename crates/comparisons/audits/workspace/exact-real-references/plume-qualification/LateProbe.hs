module Main where

import Data.Ratio
import System.Environment (getArgs)
import qualified Tests as T
import qualified SBinStream as S
import qualified SBinFloat as F
import qualified SBinDec as D
import qualified Convert as C
import qualified Integr as J
import qualified X_Repn as X

pow2 n | n>=0 = fromInteger (2^n)
       | otherwise = 1 % (2^(-n))
digits delayed x = d : digits delayed (2*x-fromIntegral d)
  where d | (if delayed then x>1/2 else x>=1/2) = 1
          | (if delayed then x< -1/2 else x<= -1/2) = -1
          | otherwise = 0
asFloat delayed x = go 0 x
  where go e q | abs q<=1 = (e,digits delayed q)
               | otherwise = go (e+1) (q/2)
prefix = foldr (\d r -> (fromIntegral d+r)/2) 0
emit bits (e,m) = do
  let n=fromInteger (max 1 (toInteger bits+e))
      ds=take n m
      center=pow2 e*prefix ds
      radius=pow2 (e-toInteger n)
  if n>4096 || any (\d->abs d>1) ds then error "invalid prefix/scale" else
    putStrLn (unwords (map show [numerator center,denominator center,numerator radius,denominator radius,e,toInteger n]))

logistic impl x n = case impl of
  "sb-float" -> T.log_mapSB x n
  "cross-float" -> T.log_mapX x n
  "dy-float" -> T.log_mapDyWrap x n
  "sb-unnormalized" -> iterate T.log_mapSB' x !! n
  "sb-stream" -> (0,T.log_mapSBs stream n)
  "cross-stream" -> (0,T.log_mapXs stream n)
  "dy-stream" -> (0,T.log_mapDysWrap stream n)
  _ -> error "unknown logistic variant"
  where stream = S.sbShift (snd x) (negate (fst x))

functional name bits = case name of
  "quadratic-min" -> emit bits (J.fnmin (\x->F.sbfSub (F.sbfMul x x) x) (q 0) (q 1))
  "quadratic-max" -> emit bits (J.fnmax (\x->F.sbfSub (F.sbfAdd (D.decSbf "0.23") (F.sbfMul (D.decSbf "1.1") x)) (F.sbfMul x x)) (q 0) (q 1))
  "square-integral" -> emit bits (J.realintegrate (\x->F.sbfMul x x) (q 0) (q 1))
  "reciprocal-max" -> emit bits (J.fnmax (\x->F.sbfSub (F.sbfSub (q 30) (X.sbfDiv (q 1) x)) (F.sbfMul (q 60) x)) (D.decSbf "0.129099") (D.decSbf "0.1291"))
  _ -> error "unknown functional"
  where q = asFloat False

main = do
  args<-getArgs
  case args of
    ["logistic",impl,rep,a,b,k,p] -> do
      let q=read a % read b
          x=case rep of
            "greedy" -> asFloat False q
            "delayed" -> asFloat True q
            "decimal-0.1" -> D.decSbf "0.1"
            "decimal-0.5467" -> D.decSbf "0.5467"
            _ -> error "unknown representation"
      if q<0 || q>1 then error "logistic stream domain" else
        emit (read p) (logistic impl x (read k))
    ["functional",name,p] -> functional name (read p)
    ["first-digit",rep,k] -> do
      let x=asFloat (rep=="delayed") (1/2)
          (a,b)=head (T.log_mapDys (C.sbsToDys (snd x)) (read k::Int))
      putStrLn (show a++" "++show b)
    _ -> error "usage: logistic VARIANT REP A B K BITS | functional NAME BITS | first-digit REP K"
