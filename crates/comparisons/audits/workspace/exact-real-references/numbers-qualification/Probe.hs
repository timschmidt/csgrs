{-# LANGUAGE ScopedTypeVariables, BangPatterns #-}
module Main where
import qualified AuditCReal as A
import qualified Data.Number.CReal as P
import qualified Data.Number.Dif as D
import qualified Data.Number.FixedFunctions as F
import qualified Data.Number.BigFloat as B
import qualified Data.Number.Symbolic as S
import qualified Data.Number.Interval as I
import Data.Ratio
import Data.List (foldl')
import System.Environment
import System.IO

raw :: A.CReal -> Int -> Integer
raw (A.CR f) = f
within :: Int -> Integer -> Rational -> Bool
within p a q = abs (fromInteger a - q*fromInteger (2^p)) <= 1
summarize :: String -> [(String,Bool)] -> IO ()
summarize name cases = do
  let (!n,!bad) = foldl' (\(n,b) (_,ok)->(n+1, b+if ok then 0 else 1)) (0::Int,0::Int) cases
  mapM_ (putStrLn . (("FAIL "++name++" ")++) . fst) (take 12 (filter (not.snd) cases))
  putStrLn $ "TOTAL "++name++" checks="++show n++" failures="++show bad

grid :: IO ()
grid = do
  let qs=[n%8 | n<-[-24..24]]
      ps=[0,1,2,4,8,16,64,160]
      ops=[("add",(+),(+)),("sub",(-),(-)),("mul",(*),(*)),("min",min,min),("max",max,max)]
      pair name op exact = [(show (q,r,p),within p (raw (op (fromRational q) (fromRational r)) p) (exact q r))|q<-qs,r<-qs,p<-ps]
  mapM_ (\(name,op,exact)->summarize name (pair name op exact)) ops
  summarize "division" [(show(q,r,p),within p (raw (fromRational q/fromRational r) p) (q/r))|q<-qs,r<-qs,r/=0,p<-ps]
  summarize "unary" [(show(q,p),within p (raw (abs (fromRational q)) p) (abs q))|q<-qs,p<-ps]
  summarize "sqrt" [(show(q,p,a),rootOK p a q)|q<-[n%8|n<-[0..256]],p<-ps,let a=raw (sqrt (fromRational q)) p]
  where rootOK p a q = let lo=(a-1)%2^p;hi=(a+1)%2^p in hi>=0 && hi*hi>=q && (lo<=0||lo*lo<=q)

decisions :: IO ()
decisions = do
  let qs=[s%2^k|k<-[32,64,136,137,138,140,160,256],s<-[-1,1]]
  summarize "public-decisions" [(show(q,name),ok)|q<-qs,let x=fromRational q::P.CReal,
    (name,ok)<-[("eq",(x==0)==(q==0)),("lt",(x<0)==(q<0)),("gt",(x>0)==(q>0)),
                ("sign",P.showCReal 0 (signum x)==show (signum (numerator q)))]]
  summarize "properFraction-integer" [(show(q,n,P.showCReal 8 y),n==truncate q)
    |q<-[n%8|n<-[-64..64]],let (n,y)=properFraction (fromRational q::P.CReal)::(Integer,P.CReal)]
  summarize "public-bridge" [(show(q,p),P.showCReal p (atan (fromRational q))==A.showCReal p (atan (fromRational q)))
    |q<-[-5%4,-1%2,0,1%2,5%4],p<-[0,10,30,60]]

derivatives :: IO ()
derivatives = do
  summarize "polynomial-derivatives" [(show(n,k,q),actual==expected)|n<-[0..8],k<-[0..10],q<-[(-3)%2,0,2%3],
    let actual=D.val (iterate D.df (D.dVar q ^ n) !! k),
    let expected=if k>n then 0 else fromInteger (product [toInteger (n-k+1)..toInteger n])*q^(n-k)]
  summarize "atan-derivatives" [(show(q,a),within 120 a (1/(1+q*q)))|q<-[-2,-1%2,0,1%2,2],
    let a=raw (D.deriv atan (fromRational q)) 120]
  let tiny=fromRational (1%2^180)::A.CReal
      got=D.deriv (\x->D.dCon tiny*x) 2
  summarize "approximate-zero-pruning" [(show(raw got 240),within 240 (raw got 240) (1%2^180))]

boundary :: IO ()
boundary = do
  let z=0::B.BigFloat B.Prec50
  summarize "BigFloat-scaled-zero" [(show k,(scaleFloat k z==z) && compare (scaleFloat k z) z==EQ)|k<-[-8..8]]
  let x=S.var "x"::S.Sym Rational
      erased=S.unSym (S.subst "x" 0 (x/x))
  putStrLn $ "OBS symbolic zero-substitution x/x="++show erased
  let interval=I.ival (0::Rational) 1
  putStrLn $ "OBS interval self Eq="++show(interval==interval)++" self Ne="++show(interval/=interval)

elementary :: String -> Rational -> Int -> IO ()
elementary name q p = do
  let x=fromRational q::A.CReal
      f=case name of
        "sqrt"->sqrt;"exp"->exp;"log"->log;"sin"->sin;"cos"->cos;"atan"->atan
        "asin"->asin;"acos"->acos;"asinh"->asinh;"acosh"->acosh;"atanh"->atanh
        "atan-deriv"->D.deriv atan; _->error "unknown elementary"
  putStrLn $ unwords [name,show(numerator q),show(denominator q),show p,show(raw (f x) p)]

fixed :: String -> Rational -> Int -> IO ()
fixed name q p = do
  let f=case name of
        "sqrt"->F.sqrt;"exp"->F.exp;"log"->F.log;"sin"->F.sin;"cos"->F.cos;"atan"->F.atan
        "asin"->F.asin;"acos"->F.acos;"asinh"->F.asinh;"acosh"->F.acosh;"atanh"->F.atanh
        _->error "unknown fixed"
      result=f (1%2^p) q
  putStrLn $ unwords [name,show(numerator q),show(denominator q),show p,show(numerator result),show(denominator result)]

main :: IO ()
main = do
  hSetBuffering stdout LineBuffering
  args<-getArgs
  case args of
    ["grid"]->grid
    ["decisions"]->decisions
    ["derivatives"]->derivatives
    ["boundary"]->boundary
    ["elementary",f,n,d,p]->elementary f (read n%read d) (read p)
    ["fixed",f,n,d,p]->fixed f (read n%read d) (read p)
    _->error "usage"
