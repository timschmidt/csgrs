{-# LANGUAGE BangPatterns #-}
module Main where
import qualified Data.Approximate.ApproximateField as A
import qualified Data.Approximate.Interval as I
import Data.Approximate.Floating.Dyadic
import qualified Data.Reals.Staged as S
import Data.Bits
import Data.List (foldl')
import Data.Ratio
import Control.Exception
import Control.Monad

rat :: Dyadic -> Rational
rat (Dyadic m e) | e >= 0 = fromInteger (m `shiftL` e)
                 | otherwise = m % (1 `shiftL` negate e)
rat q = error ("nonfinite: " ++ show q)

report :: String -> [(Bool,String)] -> IO ()
report label cases = do
  let (!n,!bad,examples) = foldl' step (0::Int,0::Int,[]) cases
      step (!n,!b,ex) (ok,msg) = (n+1,b+if ok then 0 else 1,
                                 if not ok && b < 4 then ex ++ [msg] else ex)
  putStrLn $ label ++ "\tchecks=" ++ show n ++ "\tfailures=" ++ show bad
  mapM_ (putStrLn . ("  " ++)) examples

directed :: A.Stage -> Rational -> Dyadic -> Bool
directed s x q = case A.rounding s of
  A.RoundDown -> rat q <= x
  A.RoundUp -> rat q >= x

newtype Defaults = Defaults Dyadic deriving (Eq,Ord,Show)
instance A.ApproximateField Defaults where
  appFromInteger = Defaults . A.appFromInteger
  appFromRational_ _ _ = error "unused rational conversion"
  appAdd s (Defaults a) (Defaults b) = Defaults (A.appAdd s a b)
  appSub s (Defaults a) (Defaults b) = Defaults (A.appSub s a b)
  appMul s (Defaults a) (Defaults b) = Defaults (A.appMul s a b)
  appInv s (Defaults a) = Defaults (A.appInv s a)
  appDiv s (Defaults a) (Defaults b) = Defaults (A.appDiv s a b)
  appNeg s (Defaults a) = Defaults (A.appNeg s a)
  appAbs s (Defaults a) = Defaults (A.appAbs s a)
instance A.DyadicField Defaults where
  posInf = Defaults PositiveInfinity
  negInf = Defaults NegativeInfinity
  naN = Defaults NaN
  isUnordered (Defaults a) (Defaults b) = A.isUnordered a b
  appGetExp (Defaults a) = A.appGetExp a
  appPrec (Defaults a) = A.appPrec a
  -- Intentionally exercise the donor's default appMul2 implementation.

main :: IO ()
main = do
  let vals = [Dyadic m e | m <- [-15..15], e <- [-3,0,3]]
      stages = [A.Stage p r | p <- [0,1,2,4,8], r <- [A.RoundDown,A.RoundUp]]
      binary = [("add", A.appAdd, (+)),("sub",A.appSub,(-)),
                ("mul",A.appMul,(*)),("div",A.appDiv,(/))]
  forM_ binary $ \(name,f,oracle) -> report ("dyadic-" ++ name)
    [(directed s exact got, show (s,x,y,got,exact)) |
       s <- stages, x <- vals, y <- vals, name /= "div" || rat y /= 0,
       let exact = oracle (rat x) (rat y), let got = f s x y]
  report "dyadic-inv"
    [(directed s exact got, show (s,x,got,exact)) |
       s <- stages, x <- vals, rat x /= 0,
       let exact = recip (rat x), let got = A.appInv s x]
  report "dyadic-mul2"
    [(directed s exact got, show (s,x,k,got,exact)) |
       s <- stages, x <- vals, k <- [-8,-1,0,1,8],
       let exact = rat x * rat (Dyadic 1 k), let got = A.appMul2 s x k]
  let pairs = [(a,b) | a <- [-4..4], b <- [a..4]]
      interval (a,b) = I.Interval (fromInteger a) (fromInteger b)
      endpoints (a,b) = [fromInteger a,fromInteger b] :: [Rational]
      enclosed z xs = rat (I.lower z) <= minimum xs && rat (I.upper z) >= maximum xs
  forM_ binary $ \(name,f,oracle) -> report ("proper-interval-" ++ name)
    [(enclosed got exact, show (ab,cd,got,exact)) |
       ab <- pairs, cd@(c,d) <- pairs, name /= "div" || c*d > 0,
       let got = case name of
             "add" -> A.appAdd (A.precDown 128) (interval ab) (interval cd)
             "sub" -> A.appSub (A.precDown 128) (interval ab) (interval cd)
             "mul" -> A.appMul (A.precDown 128) (interval ab) (interval cd)
             _ -> A.appDiv (A.precDown 128) (interval ab) (interval cd),
       let exact = [oracle x y | x <- endpoints ab, y <- endpoints cd]]
  report "interval-abs-enclosure"
    [(enclosed got [abs (fromInteger a),abs (fromInteger b)], show (a,b,got)) |
       ab@(a,b) <- pairs, let got = A.appAbs (A.precDown 128) (interval ab)]
  report "interval-abs-point-refinement"
    [(rat (I.upper got) - rat (I.lower got) <= 1 % (2^p), show (a,p,got)) |
       a <- [-4..4], p <- [4,16,64,256],
       let got = A.appAbs (A.precDown p) (interval (a,a))]
  report "dyadic-abs-infinity" [(abs NegativeInfinity == PositiveInfinity,show (abs NegativeInfinity))]
  report "default-power-of-two"
    [(rat z == rat x * rat (Dyadic 1 k), show (x,k,z)) |
       x <- [Dyadic 3 0, Dyadic (-5) 0], k <- [-4,-1,0,1,4],
       let Defaults z = A.appMul2 (A.precDown 128) (Defaults x) k]
  report "completion-function-stage"
    [(got == p,show (p,got)) | p <- [0,1,2,10,100],
       let got = S.approximate (S.limit A.precision :: S.StagedWithFun Int) (A.precDown p)]
  report "completion-list-stage"
    [(got == p,show (p,got)) | p <- [0,1,2,10,100],
       let got = S.approximate (S.limit A.precision :: S.StagedWithList Int) (A.precDown p)]
  forM_ [A.RoundDown,A.RoundUp] $ \r -> do
    let wrapped = S.lift1 (\_ x -> x) (S.limit A.precision :: S.StagedWithList Int)
                   :: S.StagedWithList Int
    putStrLn $ "identity-lift-stage\t" ++ show (r,S.approximate wrapped (A.prec r 10))
  let q = I.Interval (Dyadic 0 0) (Dyadic 1 0)
  putStrLn $ "wide-interval-Ord-reflexivity\t" ++ show (q == q,q <= q,compare q q)
