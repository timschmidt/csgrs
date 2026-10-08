{-# LANGUAGE ScopedTypeVariables #-}
module Main where
import Control.Exception
import Control.Monad
import Data.Char (isAscii, isDigit)
import Data.IORef
import Data.List (foldl')
import qualified Data.Map.Strict as M
import Data.Ratio
import System.Environment
import qualified SBinStream as S
import qualified SBinFloat as F
import qualified SBinDec as D
import qualified DyStream as Y
import qualified Convert as C

-- Audit-only exact Rational oracles; donor numerical modules are unchanged.
pow2 :: Integer -> Rational
pow2 n | n >= 0 = fromInteger (2^n)
       | otherwise = 1 % (2^(-n))
value :: [Int] -> Rational
value = foldr (\d q -> (fromIntegral d+q)/2) 0
clamp :: Rational -> Rational
clamp = max (-1) . min 1
digits :: Bool -> Rational -> [Int]
digits delayed q = d : digits delayed (2*q-fromIntegral d) where
  d | if delayed then q>1/2 else q>=1/2 = 1
    | if delayed then q<(-1/2) else q<=(-1/2) = -1
    | otherwise = 0
encloses :: Int -> Rational -> [Int] -> Bool
encloses n q stream = length xs==n && all ((<=1).abs) xs && abs(value xs-q)<=pow2 (-toInteger n)
  where xs=take n stream
periodic :: [Int] -> Rational
periodic xs = value xs / (1-pow2 (-toInteger (length xs)))
inputs :: [(String,Rational,[Int])]
inputs = [(show (p,t),foldr (\d q -> (fromIntegral d+q)/2) (periodic t) p,p++cycle t)
         | p<-replicateM 3 [-1,0,1],t<-[[-1],[0],[1],[1,-1],[-1,1]]]

type Counts = M.Map String (Int,Int,Int)
check :: IORef Counts -> String -> String -> Bool -> IO ()
check ref name label answer = do
  result<-try (evaluate answer)::IO (Either SomeException Bool)
  counts<-readIORef ref
  let (n,b,e)=M.findWithDefault (0,0,0) name counts
      ok=either (const False) id result
  writeIORef ref $ M.insert name (n+1,b+if ok then 0 else 1,e+either (const 1) (const 0) result) counts
  when (not ok && b<4) $ putStrLn $ unwords ["FAIL",name,label,show result]
report :: IORef Counts -> IO ()
report ref = do
  counts<-readIORef ref
  putStrLn "operation checks failures exceptions"
  forM_ (M.toAscList counts) $ \(name,(n,b,e))->putStrLn $ unwords [name,show n,show b,show e]
  putStrLn $ "TOTAL "++show (foldl' (\(a,b,c) (x,y,z)->(a+x,b+y,c+z)) (0,0,0) (M.elems counts))

grid :: IO ()
grid = do
  ref<-newIORef M.empty
  let ck=check ref
  forM_ inputs $ \(label,q,x)->forM_ [8,24] $ \n->do
    ck "clipped-add-one" label (encloses n (min 1 (q+1)) (S.one_plus_negx x))
    ck "clipped-sub-one" label (encloses n (max (-1) (q-1)) (S.mone_plus_posx x))
    forM_ [-1,0,1] $ \d->do
      ck "prefix-clipped-extension" (show (label,d)) (encloses n (clamp (2*q-fromIntegral d)) (S.p d x))
      when (abs(2*q-fromIntegral d)<=1) $
        ck "prefix-valid-identity" (show (label,d)) (encloses n q (d:S.p d x))
    forM_ [0,1,2,4,8,32::Int] $ \cap->
      forM_ [("bounded-normal",S.sbNormMax),("bounded-extra",S.sbNormMaxExtra)] $ \(name,f)->do
        let (shift,out)=f x cap
        ck name (show (label,cap,n)) (shift>=0 && shift<=toInteger cap && encloses n (q*pow2 shift) out)
  -- Avoid exact zero in the donor's known unbounded dyadic normalizer.
  -- Numeric preservation and compliance with the supplied cap are separate.
  forM_ [1..8::Int] $ \zeros->forM_ [0,1,2,4::Int] $ \cap->do
    let x=replicate zeros 0++repeat 1
        (shift,out)=Y.dysNormMax (C.sbsToDys x) cap
        expected=pow2 (shift-toInteger zeros)
        zs=map (\(a,b)->fromInteger a*pow2 (-b)) (take 24 out)
        midpoint=foldr (\d q->(d+q)/2) 0 zs
    ck "dy-normal-value" (show (zeros,cap))
      (length zs==24 && all ((<=1).abs) zs && abs(midpoint-expected)<=pow2 (-24))
    ck "dy-normal-cap" (show (zeros,cap)) (shift>=0 && shift<=toInteger cap)
  report ref

parseDecimal :: String -> Maybe Rational
parseDecimal ('-':s) = negate <$> parseUnsigned s
parseDecimal s = parseUnsigned s
parseUnsigned :: String -> Maybe Rational
parseUnsigned s = case break (=='.') s of
  (a,[]) | good a -> Just (fromInteger (read a))
  (a,'.':b) | good a && good b -> Just (fromInteger (read a)+read b % (10^length b))
  _ -> Nothing
  where good a=not(null a) && all (\c->isAscii c && isDigit c) a

formatGrid :: IO ()
formatGrid = do
  ref<-newIORef M.empty
  forM_ [-32..32] $ \a->forM_ [False,True] $ \delayed->forM_ [-4,0,1,4] $ \e->
    forM_ [0,1,3,6::Int] $ \places->do
      let q=(a%32)*pow2 e
          text=D.sbfDec (e,digits delayed (a%32)) places
          label=show (a,delayed,e,places)
      -- Fully force output inside the exception handler before parsing.
      result<-try (evaluate (length text))::IO (Either SomeException Int)
      case result of
        Left ex -> check ref "decimal-output" label (throw ex)
        Right _ -> do
          let valid=case parseDecimal text of
                Just actual -> abs(actual-q)<=1%(10^places)
                Nothing -> False
          check ref "decimal-output" (show (a,delayed,e,places,text,q)) valid
          putStrLn $ unwords ["ROW",show a,show delayed,show e,show places,show text,show (numerator q),show (denominator q)]
  report ref

printed :: IO ()
printed = do
  -- These are report equations, NOT repaired or executed donor code.
  let tails=[a%8 | a<-[-8..8]]
      g a t=let x=(fromIntegral a+t)/2 in max (-1) (x-1)
      correctG a t=case a of {1->(-1+t)/2;0->(-1+max (-1) (t-1))/2;_-> -1}
      printedG a t=case a of {1-> -1;0->(-1+max (-1) (t-1))/2;_->(-1+t)/2}
      gs=[(g a t,correctG a t,printedG a t)|a<-[-1,0,1],t<-tails]
      ps=[(clamp (fromIntegral a+t-fromIntegral d),clamp (1-t))
         |(d,a)<-[(0,1),(-1,0)],t<-tails]
  unless (all (\(want,correct,_)->want==correct) gs) $ error "audit clipped-sub identity"
  let badG=length [()|(want,_,bad)<-gs,want/=bad]
      badP=length [()|(want,bad)<-ps,want/=bad]
  unless (badG>0 && badP>0) $ error "missing printed counterexample"
  putStrLn $ unwords ["PRINTED",show (length gs),show badG,show (length ps),show badP]
  -- Exact counterexample to dyadic closure: reduced denominator is3.
  let quotient=1/(3%4)
  unless (quotient==4%3 && denominator quotient==3) $ error "audit dyadic example"
  putStrLn "PASS exact dyadic nonclosure counterexample 4/3"

emit :: F.SBinFloat -> IO ()
emit (e,m) = print (e,take 24 m)
main :: IO ()
main = do
  args<-getArgs
  case args of
    ["grid"] -> grid
    ["format"] -> formatGrid
    ["printed"] -> printed
    ["literal",text] -> emit (D.decSbf text)
    ["productive","sb-max",s] -> let (k,x)=S.sbNormMax S.sbZero (read s::Int) in emit (-k,x)
    ["productive","sb-extra",s] -> let (k,x)=S.sbNormMaxExtra S.sbZero (read s::Int) in emit (-k,x)
    ["productive","dy-max",s] -> let (k,x)=Y.dysNormMax Y.dys_Zero (read s::Int) in emit (-k,C.dysToSbs x)
    ["productive","sb-unbounded",_] -> emit (F.sbfNorm (0,S.sbZero))
    ["productive","no-neg",s] -> emit (F.sbfNormNoNegExp (read s,S.sbZero))
    ["productive","no-neg-extra",s] -> emit (F.sbfNormNoNegExpExtra (read s,S.sbZero))
    ["productive","mul-zero",s] -> emit (F.sbfMul (read s,S.sbZero) (0,S.sbHalf))
    ["productive","mul-nonzero",s] -> emit (F.sbfMul (read s,S.sbHalf) (0,S.sbHalf))
    _ -> error "usage: grid | format | printed | literal TEXT | productive OP N"
