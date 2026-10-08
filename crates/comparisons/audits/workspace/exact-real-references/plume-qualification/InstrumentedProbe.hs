{-# LANGUAGE ScopedTypeVariables #-}
module Main where
import Control.Exception
import Control.Monad
import Data.IORef
import Data.List (foldl')
import qualified Data.Map.Strict as M
import Data.Ratio
import System.Environment
import System.IO
import System.CPUTime
import System.Mem (performGC)
import GHC.Stats
import qualified LookInt as L
import qualified SBinStreamM as S
import qualified SBinFloatM as F
import qualified DyDigitM as D
import qualified DyStreamM as Y
import qualified ConvertM as C
import qualified X_RepnM as X
import qualified IRealM as I
import qualified FunctionsM as E
import qualified SBinDecM as Dec
import qualified TestsM as T
import qualified Testing as T2
import qualified IReal as Old

pow2 :: Integer -> Rational
pow2 n | n>=0 = fromInteger (2^n)
       | otherwise = 1 % (2^(-n))
value :: [Rational] -> Rational
value = foldr (\d x -> (d+x)/2) 0
digits :: Rational -> [Int]
digits q = let d=if q>=1/2 then 1 else if q<=(-1/2) then -1 else 0
           in d:digits (2*q-fromIntegral d)
tag :: [Int] -> [L.Lookint]
tag xs = zipWith L.g xs [1..]

asFloat :: Rational -> F.SBinFloat
asFloat q = go 0 q where
  go e x | abs x<=1 = (e,tag (digits x))
         | otherwise = go (e+1) (x/2)

encloses :: Int -> Rational -> S.SBinStream -> Bool
encloses n q x = let xs=map toRational (take n x)
                in length xs==n && all ((<=1).abs) xs && abs (value xs-q)<=pow2 (-toInteger n)

floatResult :: Int -> F.SBinFloat -> (Rational,Rational)
floatResult p (e,m) =
  let n=fromInteger (max 1 (toInteger p+e))
      xs=map toRational (take n m)
  in if n>10000 || any ((>1).abs) xs then error "invalid output"
     else (pow2 e*value xs,pow2 (e-toInteger n))
floatOK p q x = let (c,r)=floatResult p x in abs (c-q)<=r

dyOK :: Int -> Rational -> Y.DyStream -> Bool
dyOK n q x =
  let xs=map (\(a,b,_)->fromInteger a*pow2 (-b)) (take n x)
  in length xs==n && all ((<=1).abs) xs && abs (value xs-q)<=pow2 (-toInteger n)

decimalValue :: String -> Rational
decimalValue ('-':s) = negate (decimalValue s)
decimalValue s = case break (=='.') s of
  (a,[]) -> fromInteger (read a)
  (a,_:b) -> fromInteger (read a)+read b % (10^length b)

type Counts=M.Map String (Int,Int,Int)
check :: IORef Counts -> String -> String -> Bool -> IO ()
check ref name label answer = do
  r<-try (evaluate answer)::IO (Either SomeException Bool)
  old<-readIORef ref
  let (n,b,e)=M.findWithDefault (0,0,0) name old
      pass=either (const False) id r
  writeIORef ref $ M.insert name (n+1,b+if pass then 0 else 1,e+either (const 1) (const 0) r) old
  when (not pass && b<3) $ putStrLn $ unwords ["FAIL",name,label,show r]

inputs :: [(String,Rational,[Int])]
inputs=[(show (p,t),foldr (\d q->(fromIntegral d+q)/2) (fromIntegral t) p,p++repeat t)
        | p<-replicateM 3 [-1,0,1],t<-[-1,0,1]]

grid :: IO ()
grid = do
  ref<-newIORef M.empty
  let ck=check ref
  forM_ inputs $ \(label,q,raw)->forM_ [4,12,24] $ \n->do
    let x=tag raw
    ck "negate" label (encloses n (-q) (S.sbNegate x))
    ck "absolute" label (encloses n (abs q) (S.sbAbs x))
    ck "capped-double" label (encloses n (max (-1) (min 1 (2*q))) (S.cappedDouble x))
    ck "convert" label (encloses n q (C.dysToSbs (C.sbsToDys x)))
    forM_ [-7,-3,-2,-1,1,2,3,7] $ \d->
      ck (if d<0 then "int-div-negative" else "int-div-positive") (show (label,d,n))
        (encloses n (q/fromIntegral d) (S.sbIntDiv x (L.g d 0)))
  forM_ inputs $ \(a,q,rawx)->forM_ inputs $ \(b,r,rawy)->do
    let x=tag rawx;y=tag rawy;label=show (a,b)
    forM_ [4,12,24] $ \n->do
      forM_ [("average",S.sbAv,(q+r)/2),("average3",S.sbAv3,(q+r)/2),
             ("average-neg",S.sbAvNy,(q-r)/2),("multiply",S.sbMul,q*r),
             ("cross-multiply",X.xsbMul,q*r),("minimum",S.sbMin,min q r),
             ("maximum",S.sbMax,max q r)] $ \(name,f,w)->ck name label (encloses n w (f x y))
      when (abs (q+r)<=1) $ ck "add" label (encloses n (q+r) (S.sbAdd x y))
      when (abs (q-r)<=1) $ ck "subtract" label (encloses n (q-r) (S.sbSub x y))
    when (q/=r) $ ck "strict-comparison" label (S.sbGTE x y==(q>=r))
    when (r/=0) $ forM_ [8,24,48] $ \p->
      ck "representation-float-div" label (floatOK p (q/r) (X.sbfDiv (0,x) (0,y)))
  forM_ [-4,0,4] $ \e->forM_ [-4,0,4] $ \f->forM_ [-3,-1,0,1,3] $ \a->forM_ [-3,-1,1,3] $ \b->do
    let q=fromIntegral a/4*pow2 e;r=fromIntegral b/4*pow2 f
        x=(e,tag (digits (fromIntegral a/4)));y=(f,tag (digits (fromIntegral b/4)))
    forM_ [8,24,48] $ \p->ck "float-div" (show (e,f,a,b,p)) (floatOK p (q/r) (X.sbfDiv x y))
  forM_ [-4..4] $ \a->forM_ [1..4] $ \k->forM_ [4,16,32] $ \p->do
    let q=a%4
        intervals=[(asFloat (q-pow2 (-i)),asFloat (q+pow2 (-i))) | i<-[k..]]
    -- Avoid initial upper=0, an explicitly separate nonproductive case.
    when (q+pow2 (-k)/=0) $ ck "nested-limit" (show (a,k,p)) (floatOK p q (I.sbfLimit intervals))
  result<-readIORef ref
  putStrLn "operation checks failures exceptions"
  forM_ (M.toAscList result) $ \(name,(n,b,e))->putStrLn $ unwords [name,show n,show b,show e]
  putStrLn $ "TOTAL "++show (foldl' (\(a,b,c) (x,y,z)->(a+x,b+y,c+z)) (0,0,0) (M.elems result))

extra :: IO ()
extra = do
  ref<-newIORef M.empty
  let ck=check ref
  forM_ [-32..32] $ \a->forM_ [1,10,100] $ \d->do
    let q=a%d
        frac=show (abs a `mod` d)
        width=if d==10 then 1 else 2
        s=show (abs a `div` d)++"."++replicate (width-length frac) '0'++frac
        input=(if a<0 then "-" else "")++if d==1 then show (abs a) else s
    ck "decimal-import" input (floatOK 32 q (Dec.decSbf input))
    forM_ [1,4,9] $ \n->do
      let output=Dec.sbfDec (asFloat q) n
      ck "decimal-output" (show (q,n)) (abs (decimalValue output-q)<=1%(10^n))
  forM_ [0..16] $ \a->forM_ [1..3] $ \k->forM_ [8,24] $ \n->do
    let q=a%16; x=tag (digits q);dy=C.sbsToDys x
        expected=iterate (\v->4*v*(1-v)) q!!k
        missingFour=iterate (\v->v*(1-v)) q!!k
        (e,m)=T.log_mapDyf (0,dy) k
    ck "logistic-signed-stream" (show (a,k,n)) (encloses n expected (T.log_mapSBs x k))
    ck "logistic-signed-testing" (show (a,k,n)) (encloses n expected (T2.it_lmap x k))
    ck "logistic-signed-float" (show (a,k,n)) (floatOK n expected (T.log_mapSbf (0,x) k))
    ck "logistic-dyadic-stream" (show (a,k,n)) (dyOK n expected (T.log_mapDys dy k))
    ck "logistic-dyadic-missing-four-control" (show (a,k,n)) (dyOK n missingFour (T.log_mapDys dy k))
    ck "logistic-dyadic-float" (show (a,k,n)) (dyOK n (expected/pow2 e) m)
  forM_ [-4..4] $ \a->forM_ [1..4] $ \k->forM_ [4,16,32] $ \p->do
    let q=a%4
        plain t=let (e,m)=asFloat t in (e,map fromIntegral m)
        intervals=[(plain (q-pow2 (-i)),plain (q+pow2 (-i))) | i<-[k..]]
        (e,m)=Old.sbfLimit intervals
    when (q+pow2 (-k)/=0) $ ck "old-nested-limit" (show (a,k,p)) (floatOK p q (e,tag m))
  forM_ [(minBound,maxBound),(maxBound,minBound),(0,minBound),(minBound,0)] $ \(a,b)->
    ck "lookint-order-overflow" (show (a,b)) (compare (L.g a 0) (L.g b 0)==compare a b)
  putStrLn "operation checks failures exceptions"
  result<-readIORef ref
  forM_ (M.toAscList result) $ \(name,(n,b,e))->putStrLn $ unwords [name,show n,show b,show e]
  putStrLn $ "TOTAL "++show (foldl' (\(a,b,c) (x,y,z)->(a+x,b+y,c+z)) (0,0,0) (M.elems result))

-- Test actual digit VALUE demand by making values beyond a prefix undefined.
-- List spines and tags stay available: this does not mistake spine pattern
-- matching for forcing a numeric digit. No unsafe IO or donor formula changes.
bounded :: Int -> [Int] -> S.SBinStream
bounded cutoff = zipWith (\i d->L.g (if i<=cutoff then d else error "input digit beyond bound") i) [1..]
force :: Int -> S.SBinStream -> Integer
force n=foldl' (\a d->a+toInteger d) 0 . take n

demand :: IO ()
demand = do
  let samples=inputs
      ops=[("average",S.sbAv),("average3",S.sbAv3),("add",S.sbAdd),
           ("subtract",S.sbSub),("multiply",S.sbMul),("cross-multiply",X.xsbMul),
           ("min",S.sbMin),("max",S.sbMax)]
  putStrLn "operation left right digits reported actual classification"
  forM_ ops $ \(name,op)->forM_ (zip [0::Int ..] samples) $ \(ai,(_,q,a))->
    forM_ (zip [0::Int ..] samples) $ \(bi,(_,r,b))->forM_ [1,3,8] $ \n->
    when ((name/="add" || abs (q+r)<=1) && (name/="subtract" || abs (q-r)<=1)) $ do
      let output=take n (op (tag a) (tag b))
          expected=case name of
            "average"->(q+r)/2;"average3"->(q+r)/2;"add"->q+r
            "subtract"->q-r;"multiply"->q*r;"cross-multiply"->q*r
            "min"->min q r;"max"->max q r;_->error "unknown demand operation"
          reported=maximum (0:map L.e output)
          works k=do r<-try (evaluate (force n (op (bounded k a) (bounded k b))))::IO (Either SomeException Integer)
                     pure (either (const False) (const True) r)
          search lo hi | hi-lo<=1 = pure hi
                       | otherwise = let mid=(lo+hi) `div` 2 in do
                           yes<-works mid
                           if yes then search lo mid else search mid hi
      evaluate (force n output)
      unless (encloses n expected output) $ error "incorrect numerical demand probe output"
      upper<-works 128
      actual<-if upper then search (-1) 128 else pure 129
      putStrLn $ unwords [name,show ai,show bi,show n,show reported,show actual,
                          if actual>128 then "UNRESOLVED" else if reported<actual then "UNDER" else if reported==actual then "EXACT" else "OVER"]

emit :: String -> Rational -> Int -> IO ()
emit op q p = do
  let x=asFloat q
      y=case op of
        "exp"->E.sbfExpFn x;"ln"->E.sbfLn x;"sin"->E.sbfSin x
        "cos"->E.sbfCos x;"atan"->E.sbfArctan x;"pi"->E.sbfPi
        _->error "unknown operation"
      (c,r)=floatResult p y
  putStrLn $ unwords [op,show (numerator q),show (denominator q),show p,
    show (numerator c),show (denominator c),show (numerator r),show (denominator r)]

bench :: String -> String -> Int -> Int -> IO ()
bench name family bits batch = do
  let op=case name of "average"->S.sbAv;"carry-average"->S.sbAv3;_->error "unknown average"
      inputs=[let (q,r)=case family of
                         "dense"->(1/3+fromIntegral i/8192,-2/3+fromIntegral i/4096)
                         "sparse"->(pow2 (-toInteger (bits `div` 2))*(1/3+fromIntegral i/8192),1/3)
                         "terminating"->((fromIntegral i+3)/1024,(fromIntegral i+5)/2048)
                         _->error "unknown family"
              in (q,r,tag (digits q),tag (digits r)) | i<-[1..batch]]
      forcePrefix=foldl' (\a d->2*a+toInteger d) 0 . take bits
  evaluate $ sum [sum [toInteger d+toInteger (L.e d) | d<-take (bits+8) x++take (bits+8) y] | (_,_,x,y)<-inputs]
  performGC
  before<-getRTSStats
  start<-getCPUTime
  let results=[forcePrefix (op x y) | (_,_,x,y)<-inputs]
  checksum<-evaluate (sum results)
  stop<-getCPUTime
  performGC
  after<-getRTSStats
  unless (and [abs (fromInteger k*pow2 (-toInteger bits)-(q+r)/2)<=pow2 (-toInteger bits)
               | ((q,r,_,_),k)<-zip inputs results]) $ error "benchmark failed exact Rational oracle"
  putStrLn $ unwords [name,family,show bits,show batch,show (stop-start),
    show (allocated_bytes after-allocated_bytes before),show checksum,"PASS"]

examples :: IO ()
examples = do
  putStrLn $ "division expected=-8 enclosure="++show (floatResult 24 (X.sbfDiv (asFloat (-1)) (asFloat (1/8))))
  putStrLn $ "capped-double expected=-3/4 prefix24="++show (value (map toRational (take 24 (S.cappedDouble (tag ([-1,0,1]++repeat 0))))))
  putStrLn $ "negative-limit expected=-3/4 enclosure="++show (floatResult 24 (I.sbfLimit [(asFloat (-3/4-pow2 (-i)),asFloat (-3/4+pow2 (-i))) | i<-[1..]]))
  forM_ [-32,-2,-1,0,1,2,32] $ \q->putStrLn $ "decimal "++show (q,Dec.sbfDec (asFloat q) 4)
  let out=take 8 (S.sbIntDiv (tag (digits (3/4))) (L.g 3 1000))
  putStrLn $ "divisor-tag-1000 output="++show out++" correct="++show (encloses 8 (1/4) out)
  r<-try (evaluate (force 8 (S.sbIntDiv (tag (digits (3/4))) (L.g (error "divisor value forced") 1000))))::IO (Either SomeException Integer)
  putStrLn $ "undefined-divisor-control "++show r

boundary :: String -> IO ()
boundary name = print $ case name of
  "float-int-div-zero"->fst (F.sbfIntDiv (asFloat (1/2)) (L.g 0 0))
  "limit-upper-zero"->fst (I.sbfLimit [(asFloat (-1/2-pow2 (-i)),asFloat (-1/2+pow2 (-i))) | i<-[1..]])
  "dyadic-norm-cap"->fst (Y.dysNormMax (replicate 20 D.dyd_Zero++Y.dys_One) (2::Int))
  _->error "unknown boundary"

main :: IO ()
main = do
  hSetBuffering stdout LineBuffering
  args<-getArgs
  case args of
    []->grid
    ["extra"]->extra
    ["demand"]->demand
    ["examples"]->examples
    ["boundary",name]->boundary name
    ["bench",name,family,p,batch]->bench name family (read p) (read batch)
    ["emit",op,a,b,p]->emit op (read a%read b) (read p)
    _->error "usage: probe [extra | demand | examples | boundary NAME | bench NAME FAMILY BITS BATCH | emit OP NUM DEN BITS]"
