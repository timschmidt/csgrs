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
import System.Timeout
import System.CPUTime
import System.Mem (performGC)
import GHC.Stats
import qualified DyDigit as D
import qualified DyStream as Y
import qualified DyFloat as Z
import qualified SBinStream as S
import qualified SBinFloat as F
import qualified SBinDec as T
import qualified Convert as C
import qualified X_Repn as X
import qualified IReal as I
import qualified Functions as E
import qualified Integr as J

-- Oracle arithmetic is Data.Ratio Integer, independent of donor kernels.
pow2 :: Integer -> Rational
pow2 e | e >= 0 = fromInteger (2^e)
       | otherwise = 1 % (2^(-e))

dval :: D.DyDigit -> Rational
dval (a,b) = fromInteger a * pow2 (-b)

prefix :: [Rational] -> Rational
prefix = foldr (\d r -> (d+r)/2) 0

digits :: Rational -> S.SBinStream
digits q = let d = if q >= 1/2 then 1 else if q <= -1/2 then -1 else 0
           in d : digits (2*q-fromIntegral d)

asFloat :: Rational -> F.SBinFloat
asFloat q = go 0 q where
  go e r | abs r <= 1 = (e,digits r)
         | otherwise = go (e+1) (r/2)

encloses :: Int -> Rational -> [Rational] -> Bool
encloses n q ds = length xs == n && all ((<=1).abs) xs
                  && abs (prefix xs-q) <= pow2 (negate (toInteger n))
  where xs = take n ds

sbOK n q = encloses n q . map fromIntegral
dyOK n q = encloses n q . map dval

floatResult :: Int -> F.SBinFloat -> (Rational,Rational)
floatResult p (e,m) =
  let n = fromInteger (max 1 (toInteger p+e))
      xs = take n m
  in if n > 10000 || any (\d -> abs d > 1) xs then error "invalid output digits/scale"
     else (pow2 e * prefix (map fromIntegral xs), pow2 (e-toInteger n))

floatOK p q x = let (c,r) = floatResult p x in abs (c-q) <= r

type Counts = M.Map String (Int,Int,Int)
check :: IORef Counts -> String -> String -> Bool -> IO ()
check ref name label answer = do
  result <- try (evaluate answer) :: IO (Either SomeException Bool)
  old <- readIORef ref
  let (count,bad,exc) = M.findWithDefault (0,0,0) name old
      pass = either (const False) id result
      exception = either (const 1) (const 0) result
  writeIORef ref $ M.insert name (count+1,bad+if pass then 0 else 1,exc+exception) old
  when (not pass && bad < 3) $ putStrLn $ "FAIL\t"++name++"\t"++label++"\t"++show result

streamInputs :: [(String,Rational,S.SBinStream)]
streamInputs = [(show (p,t),foldr (\a r -> (fromIntegral a+r)/2) (fromIntegral t) p,
                 p++repeat t) | p <- replicateM 3 [-1,0,1], t <- [-1,0,1]]

canonical :: Integer -> Integer -> D.DyDigit
canonical 0 _ = (0,0)
canonical a b | b>0 && even a = canonical (a `div` 2) (b-1)
              | otherwise = (a,b)

kernel :: IO ()
kernel = do
  ref <- newIORef M.empty
  let ck = check ref
      ds = [canonical a 4 | a <- [-16..16]]
  forM_ ds $ \a -> forM_ ds $ \b -> do
    let x = dval a; y = dval b; label = show (a,b)
    forM_ [("dydAv",D.dydAv,(x+y)/2),("dydAvNy",D.dydAvNy,(x-y)/2),
           ("dydAdd",D.dydAdd,x+y),("dydSub",D.dydSub,x-y),
           ("dydMul",D.dydMul,x*y)] $ \(name,f,q) -> ck name label (dval (f a b)==q)
    forM_ [("dydEq",D.dydEq,x==y),("dydGTE",D.dydGTE,x>=y),
           ("dydGT",D.dydGT,x>y),("dydLEQ",D.dydLEQ,x<=y),
           ("dydLT",D.dydLT,x<y)] $ \(name,f,q) -> ck name label (f a b==q)
    forM_ [("dydAddRC",D.dydAddRC,x+y),("dydSubRC",D.dydSubRC,x-y)] $ \(name,f,q) ->
      let (a',b') = f a b in ck name label (dval a'+dval b'==q && abs (dval a')<=1 && abs (dval b')<=1)
  forM_ [1..8] $ \b -> do
    ck "constructor-canonical-equality" (show b) (D.dydEq (D.dyd (2, b+1)) (D.dyd (1,b)))
    ck "shift-zero-equality" (show b) (D.dydEq (D.dydShr D.dyd_Zero b) D.dyd_Zero)
  forM_ streamInputs $ \(label,q,x) -> forM_ [4,12,24] $ \n -> do
    ck "sbNegate" label (sbOK n (-q) (S.sbNegate x))
    ck "sbAbs" label (sbOK n (abs q) (S.sbAbs x))
    ck "cappedDouble" label (sbOK n (max (-1) (min 1 (2*q))) (S.cappedDouble x))
    ck "convert-roundtrip" label (sbOK n q (C.dysToSbs (C.sbsToDys x)))
    when (abs q<=1/2) $ ck "dyShl" label (dyOK n (2*q) (Y.dysShl (C.sbsToDys x) (1::Int)))
    forM_ [-7,-3,-2,-1,1,2,3,7] $ \d ->
      ck (if d<0 then "sbIntDiv-negative" else "sbIntDiv-positive") (show (label,d,n))
         (sbOK n (q/fromIntegral d) (S.sbIntDiv x d))
  forM_ streamInputs $ \(a,q,x) -> forM_ streamInputs $ \(b,r,y) -> do
    let label = show (a,b)
    forM_ [4,12,24] $ \n -> do
      forM_ [("sbAv",S.sbAv,(q+r)/2),("sbAvNy",S.sbAvNy,(q-r)/2),
             ("sbMul",S.sbMul,q*r),("xsbMul",X.xsbMul,q*r),
             ("sbMin",S.sbMin,min q r),("sbMax",S.sbMax,max q r)] $ \(name,f,w) ->
        ck name label (sbOK n w (f x y))
      when (abs (q+r)<=1) $ do
        ck "sbAdd" label (sbOK n (q+r) (S.sbAdd x y))
        ck "dyAdd" label (dyOK n (q+r) (Y.dysAdd (C.sbsToDys x) (C.sbsToDys y)))
      when (abs (q-r)<=1) $ do
        ck "sbSub" label (sbOK n (q-r) (S.sbSub x y))
        ck "dySub" label (dyOK n (q-r) (Y.dysSub (C.sbsToDys x) (C.sbsToDys y)))
    when (q/=r) $ ck "sbGTE-strict" label (S.sbGTE x y == (q>=r))
  forM_ [-4,0,4] $ \e -> forM_ [-4,0,4] $ \f ->
    forM_ [-3,-1,0,1,3] $ \a -> forM_ [-3,-1,1,3] $ \b -> do
      let q = fromIntegral a/4 * pow2 e; r = fromIntegral b/4 * pow2 f
          x = (e,digits (fromIntegral a/4)); y = (f,digits (fromIntegral b/4))
          label = show (e,f,a,b)
      forM_ [8,24,48] $ \p ->
        forM_ [("sbfAdd",F.sbfAdd,q+r),("sbfSub",F.sbfSub,q-r),
               ("sbfMul",F.sbfMul,q*r),("sbfDiv",X.sbfDiv,q/r),
               ("sbfMin",F.sbfMin,min q r),("sbfMax",F.sbfMax,max q r)] $ \(name,op,w) ->
          ck name label (floatOK p w (op x y))
  forM_ [("sbfFour",4,F.sbfFour),("sbfTwo",2,F.sbfTwo),
         ("dyfQuarter",1/4,C.dyfToSbf Z.dyf_Quarter),
         ("dyfMinusQuarter",-1/4,C.dyfToSbf Z.dyf_minusQuarter),
         ("dysMinusThird",-1/3,(0,C.dysToSbs Y.dys_minusThird))] $ \(name,q,x) ->
    ck name "constant" (floatOK 32 q x)
  forM_ [-3..3] $ \a -> forM_ [1..6] $ \k -> do
    let q = a%4
        intervals = [(asFloat (q-pow2 (-i)),asFloat (q+pow2 (-i))) | i <- [k..]]
    ck "nested-limit" (show (a,k)) (floatOK 24 q (I.sbfLimit intervals))
  result <- readIORef ref
  putStrLn "operation\tchecks\tfailures\texceptions"
  forM_ (M.toAscList result) $ \(name,(n,b,e)) -> putStrLn $ name++"\t"++show n++"\t"++show b++"\t"++show e
  let (n,b,e) = foldl' (\(a,b,c) (x,y,z) -> (a+x,b+y,c+z)) (0,0,0) (M.elems result)
  putStrLn $ "TOTAL\t"++show n++"\t"++show b++"\t"++show e

boundary :: String -> IO ()
boundary name = do
  let forced = case name of
        "dy-norm-zero" -> fst (Y.dysNormMax Y.dys_Zero (1::Int))
        "dy-norm-cap" -> fst (Y.dysNormMax (replicate 20 D.dyd_Zero++Y.dys_One) (2::Int))
        "mul-zero-500" -> fst (F.sbfMul (250,S.sbZero) (250,S.sbOne))
        "mul-zero-499" -> toInteger (head (snd (F.sbfMul (249,S.sbZero) (250,S.sbOne))))
        "sin-zero" -> numerator (fst (floatResult 16 (E.sbfSin F.sbfZero)))
        "cos-zero" -> numerator (fst (floatResult 16 (E.sbfCos F.sbfZero)))
        "atan-zero" -> numerator (fst (floatResult 16 (E.sbfArctan F.sbfZero)))
        "sqrt-zero" -> numerator (fst (floatResult 16 (E.sbfSqrt F.sbfZero)))
        "exp-zero" -> numerator (fst (floatResult 16 (E.sbfExpFn F.sbfZero)))
        "between-zero" -> fst (I.sbfBetween (repeat (F.sbfZero,F.sbfZero)))
        "gte-equal" -> if S.sbGTE S.sbZero S.sbZero then 1 else 0
        _ -> error "unknown boundary case"
  result <- timeout 1000000 (try (evaluate forced) :: IO (Either SomeException Integer))
  putStrLn $ name++"\t"++show result

emit :: String -> Rational -> Int -> IO ()
emit op q p = emitValue op q p (asFloat q)

emitValue :: String -> Rational -> Int -> F.SBinFloat -> IO ()
emitValue op q p x = do
  let y = case op of
        "exp" -> E.sbfExpFn x; "ln" -> E.sbfLn x; "sqrt" -> E.sbfSqrt x
        "sin" -> E.sbfSin x; "cos" -> E.sbfCos x; "atan" -> E.sbfArctan x
        "pi" -> E.sbfPi
        _ -> error "unknown elementary operation"
      (c,r) = floatResult p y
  putStrLn $ unwords [op,show (numerator q),show (denominator q),show p,
                      show (numerator c),show (denominator c),show (numerator r),show (denominator r)]

bench :: String -> String -> Int -> Int -> IO ()
bench name family bits batch = do
  let op = case name of "signed" -> S.sbMul; "dyadic" -> X.xsbMul; _ -> error "unknown multiply"
      inputs = [let (q,r) = case family of
                         "dense" -> (1/3+fromIntegral i/8192,-2/3+fromIntegral i/4096)
                         "sparse" -> (pow2 (negate (toInteger (bits `div` 2)))*(1/3+fromIntegral i/8192),1/3)
                         "terminating" -> ((fromIntegral i+3)/64,(fromIntegral i+5)/128)
                         _ -> error "unknown family"
                in (q,r,digits q,digits r) | i <- [1..batch]]
      forcePrefix = foldl' (\a d -> 2*a+toInteger d) 0 . take bits
  -- Inputs are pre-forced equally; measurement concerns fresh output kernels.
  evaluate $ sum [sum (take (bits+8) x)+sum (take (bits+8) y) | (_,_,x,y)<-inputs]
  performGC
  startStats <- getRTSStats
  start <- getCPUTime
  let results = [forcePrefix (op x y) | (_,_,x,y)<-inputs]
  checksum <- evaluate (sum results)
  stop <- getCPUTime
  performGC
  endStats <- getRTSStats
  let valid = and [abs (fromInteger k*pow2 (negate (toInteger bits))-q*r)<=pow2 (negate (toInteger bits))
                  | ((q,r,_,_),k)<-zip inputs results]
  unless valid $ error "benchmark output failed independent exact Rational oracle"
  putStrLn $ unwords [name,family,show bits,show batch,show (stop-start),
    show (allocated_bytes endStats-allocated_bytes startStats),show checksum,"PASS"]

extra :: IO ()
extra = do
  ref <- newIORef M.empty
  let ck = check ref
      dyInputs = [(show (p,t),foldr (\a r -> (dval a+r)/2) (dval t) p,p++repeat t)
                  | p<-replicateM 2 [(1,0),(-1,0),(0,0),(1,1),(-1,2)],
                    t<-[(1,0),(0,0),(-1,0)]]
  forM_ dyInputs $ \(label,q,x) -> forM_ [8,20] $ \n -> do
    ck "dy-convert" label (sbOK n q (C.dysToSbs x))
    ck "dy-negate" label (dyOK n (-q) (Y.dysNegate x))
    forM_ [canonical a 4 | a<-[-16..16]] $ \d ->
      ck "dy-digit-mul" label (dyOK n (q*dval d) (Y.dysDigitMul d x))
  forM_ dyInputs $ \(a,q,x) -> forM_ dyInputs $ \(b,r,y) -> forM_ [8,20] $ \n ->
    forM_ [("dy-average",Y.dysAv,(q+r)/2),("dy-average-neg",Y.dysAvNy,(q-r)/2),
           ("dy-multiply",Y.dysMul,q*r)] $ \(name,op,w) -> ck name (show (a,b)) (dyOK n w (op x y))
  forM_ [minBound,maxBound,2^62,2^62+1,negate (2^62+1)] $ \d ->
    forM_ [-4..4] $ \a -> forM_ [64,80,128] $ \n -> do
      let q=fromIntegral a/4; want=q/fromIntegral (d::Int)
      ck "int-div-overflow" (show (a,d,n)) (sbOK n want (S.sbIntDiv (digits q) d))
      ck "float-int-div-overflow" (show (a,d,n)) (floatOK n want (F.sbfIntDiv (asFloat q) d))
  forM_ [-32..32] $ \a -> forM_ [1,10,100] $ \d -> do
    let q=a%d
        s=show (abs a `div` d)++"."++ replicate (width d-length frac) '0'++frac
        frac=show (abs a `mod` d)
        width 1=0; width 10=1; width _=2
        input=(if a<0 then "-" else "")++if d==1 then show (abs a) else s
    ck "decimal-import" input (floatOK 32 q (T.decSbf input))
  putStrLn "operation\tchecks\tfailures\texceptions"
  results<-readIORef ref
  forM_ (M.toAscList results) $ \(name,(n,b,e))->putStrLn (unwords [name,show n,show b,show e])
  putStrLn $ "TOTAL "++show (foldl' (\(a,b,c) (x,y,z)->(a+x,b+y,c+z)) (0,0,0) (M.elems results))

functional :: IO ()
functional = do
  let families = [("zero",const F.sbfZero,0,0,0),
                  ("one",const F.sbfOne,0,0,1),
                  ("identity",id,0,1,0),
                  ("negative",F.sbfNegate,0,-1,0),
                  ("square",\x->F.sbfMul x x,1,0,0),
                  ("affine",\x->F.sbfAdd (F.sbfShl x 1) F.sbfOne,0,2,1)]
      value a b c x=a*x*x+b*x+c
      primitive a b c x=a*x*x*x/3+b*x*x/2+c*x
  forM_ families $ \(name,f,a,b,c) -> forM_ [(0,1),(-1,1),(1,2),(2,-1),(0,0)] $ \(l,u) ->
    forM_ [4,8,12] $ \p -> do
      let points=[l,u]++[-b/(2*a) | a/=0 && min l u<=(-b/(2*a)) && (-b/(2*a))<=max l u]
          vals=map (value a b c) points
      forM_ [("integral",J.realintegrate,primitive a b c u-primitive a b c l),
             ("maximum",J.fnmax,maximum vals),("minimum",J.fnmin,minimum vals)] $ \(op,g,want) -> do
        result<-try (timeout 1000000 (evaluate (floatOK p want (g f (asFloat l) (asFloat u)))))
                    :: IO (Either SomeException (Maybe Bool))
        putStrLn $ unwords [name,show l,show u,show p,op,show result]

main :: IO ()
main = do
  hSetBuffering stdout LineBuffering
  args <- getArgs
  case args of
    [] -> kernel
    ["boundary",name] -> boundary name
    ["emit",op,a,b,p] -> emit op (read a % read b) (read p)
    ["bench",op,family,p,batch] -> bench op family (read p) (read batch)
    ["extra"] -> extra
    ["functional"] -> functional
    ["representation",rep,k,p] -> do
      let e=read k; q=pow2 e
          x=case rep of
            "endpoint" -> (e,S.sbOne)
            "terminating" -> (e+1,S.sbHalf)
            "redundant" -> (e+1,1:(-1):S.sbOne)
            "decimal" -> T.decSbf (show (numerator q))
            _ -> error "unknown representation"
      emitValue "ln" q (read p) x
    _ -> error "usage: probe [boundary NAME | emit OP NUM DEN BITS]"
