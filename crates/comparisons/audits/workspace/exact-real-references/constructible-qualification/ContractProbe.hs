module Main where
import Control.Exception (ArithException(..),SomeException,evaluate,fromException,try)
import Data.Complex.Generic (Complex(..))
import Data.Real.Constructible (Construct,ConstructException(..),fromConstruct)
import Data.Ratio ((%))
import System.Environment (getArgs)
import System.Exit (exitWith,ExitCode(..))
import System.IO (hSetBuffering,BufferMode(..),stdout)
import System.CPUTime (getCPUTime)

data Expected = TrueValue | ConstructError ConstructException | ArithmeticError ArithException
contracts :: [(String,Expected,Bool)]
contracts =
  [("dyadic-"++show d++"-"++show n,TrueValue,
    (fromInteger (2^(2^d)) :: Construct) ** fromRational (n % 2^d) == fromRational (2 ^^ n :: Rational))
    | d <- [0..4 :: Integer], n <- [-8..8]] ++
  [("negative-integer-"++show n,TrueValue,(-2 :: Construct) ** fromInteger n == fromRational ((-2) ^^ n :: Rational)) | n <- [-8..8]] ++
  [("zero-zero",TrueValue,(0 :: Construct)**0==1),
   ("zero-positive",TrueValue,(0 :: Construct)**(1/4)==0),
   ("sqrt-negative",ConstructError ConstructSqrtNegative,sqrt (-1 :: Construct)==0),
   ("sqrt-negative-dyadic",ConstructError ConstructSqrtNegative,(-4 :: Construct)**(1/2)==0),
   ("inverse-zero",ArithmeticError RatioZeroDenominator,recip (0 :: Construct)==0),
   ("zero-negative",ArithmeticError RatioZeroDenominator,(0 :: Construct)**(-1)==0),
   ("irrational-rational",ConstructError ConstructIrrational,toRational (sqrt 2 :: Construct)==0),
   ("irrational-exponent",ConstructError ConstructIrrational,(2 :: Construct)**sqrt 2==0),
   ("nondyadic-exponent",ConstructError (Unconstructible "(** non-dyadic-rational)"),(8 :: Construct)**(1/3)==0)] ++
  [("unsupported-"++name,ConstructError (Unconstructible name),f (2 :: Construct)==0)
    | (name,f) <- [("exp",exp),("log",log),("sin",sin),("cos",cos),("tan",tan),("asin",asin),("acos",acos),("atan",atan),("sinh",sinh),("cosh",cosh),("tanh",tanh),("asinh",asinh),("acosh",acosh),("atanh",atanh)]] ++
  [("unsupported-pi",ConstructError (Unconstructible "pi"),(pi :: Construct)==0)]

doc :: String -> Bool
doc "golden" = and [(((1+sqrt 5)/2)^n-((1-sqrt 5)/2)^n :: Construct) == fromInteger f*sqrt 5 | (n,f)<-zip [1..10 :: Integer] [1,1,2,3,5,8,13,21,34,55] :: [(Integer,Integer)]]
doc "agm" =
  let f (a,b,t,p)=((a+b)/2,sqrt(a*b),t-p*((a-b)/2)^2,2*p)
      (a,b,t,_)=f.f.f.f $ (1,1/sqrt 2,1/4,1 :: Construct)
  in floor (((a+b)^2/(4*t))*10^40) == (31415926535897932384626433832795028841971 :: Integer)
doc "unity17" =
  let qf (p,q)=((p+sqrt(p^2-4*q))/2,(p-sqrt(p^2-4*q))/2 :: Construct)
      (v,w)=qf(-1,-4);(x,_)=qf(v,-1);(y,_)=qf(w,-1);(z,_)=qf(x,y)
  in ((z/2 :+ sqrt(1-(z/2)^2))^17 :: Complex Construct)==1
doc "conversion" =
  let x=sum(map sqrt [7,14,39,70,72,76,85])-sum(map sqrt [13,16,46,55,67,73,79]) :: Construct
      a=fromConstruct x :: Double
  -- Merely a broad finite-range check for the documented close radical sum.
  -- Independent directed-MPFR qualification is still required for accuracy.
  in a>1e-19 && a<3e-19 && not(isNaN a || isInfinite a)
doc _ = error "doc"

run :: (String,Expected,Bool) -> IO Bool
run (label,expected,value)=do
  putStrLn ("START\t"++label)
  t<-getCPUTime
  result<-try(evaluate value) :: IO(Either SomeException Bool)
  end<-getCPUTime
  let ok=case (expected,result) of
        (TrueValue,Right True)->True
        (ConstructError e,Left actual)->fromException actual==Just e
        (ArithmeticError e,Left actual)->fromException actual==Just e
        _->False
  putStrLn ((if ok then "PASS" else "FAIL")++"\t"++label++"\t"++show(end-t)++"\t"++either show show result)
  pure ok
main :: IO ()
main=do
  hSetBuffering stdout LineBuffering
  [mode]<-getArgs
  let rows=if mode=="contracts" then contracts else [(mode,TrueValue,doc mode)]
  good<-mapM run rows
  putStrLn ("SUMMARY\t"++show(length(filter id good))++"\t"++show(length good))
  if and good then pure() else exitWith(ExitFailure 2)
