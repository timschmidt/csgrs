{-# LANGUAGE CPP #-}
module Main where
import Control.DeepSeq (force)
import Control.Exception (SomeException,evaluate,try)
import Control.Monad (forM_)
import Data.Ratio ((%))
import Data.CDAR
import System.IO (stdout,hSetBuffering,BufferMode(LineBuffering))
import System.Timeout (timeout)

make :: Integer -> Integer -> Int -> Approx
#ifdef MBOUND
make = approxMB 80
#else
make = Approx
#endif

-- Independent rational endpoint interpretation; no donor interval operations.
bounds :: Approx -> Maybe (Rational,Rational)
bounds Bottom = Nothing
#ifdef MBOUND
bounds (Approx _ m e s) = Just (fromInteger (m-e)*2^^s,fromInteger (m+e)*2^^s)
#else
bounds (Approx m e s) = Just (fromInteger (m-e)*2^^s,fromInteger (m+e)*2^^s)
#endif

encloses :: Approx -> (Rational,Rational) -> Bool
encloses result (l,u) = case bounds result of
  Nothing -> True
  Just (a,b) -> a<=l && u<=b

report :: String -> [(String,Bool)] -> IO ()
report label cases = do
  let failures=[s | (s,False)<-cases]
  putStrLn (label++": "++show (length cases)++" cases, "++show (length failures)++" violations")
  mapM_ (putStrLn . ("  "++)) (take 5 failures)

probe :: String -> String -> IO ()
probe label value = do
  result <- timeout 1500000 (try (evaluate (force value)) :: IO (Either SomeException String))
  putStrLn (label++": "++case result of
    Nothing -> "TIMEOUT"
    Just (Left e) -> "EXCEPTION "++show e
    Just (Right s) -> s)

main :: IO ()
main = do
  hSetBuffering stdout LineBuffering
  let grid=[make m e s | m<-[-4..4],e<-[0..3],s<-[-2..2]]
  forM_ [("add",(+),(\l u a b -> (l+a,u+b))),
         ("multiply",(*),(\l u a b -> (minimum [l*a,l*b,u*a,u*b],maximum [l*a,l*b,u*a,u*b])))] $
    \(label,op,oracle) -> report label
      [(show (x,y,result),encloses result (oracle l u a b)) |
       x<-grid,y<-grid,Just (l,u)<-[bounds x],Just(a,b)<-[bounds y],let result=op x y]
  let wide=[make m e s | m<-[-2049,-1025,-1,0,1,1025,2049],
                        e<-[0,1,1023,1024,1025,2047,2048,2049],s<-[-8,0,8]]
  report "boundErrorTerm" [(show(x,y),encloses y b)|x<-wide,Just b<-[bounds x],let y=boundErrorTerm x]
  report "limitSize" [(show(x,p,y),encloses y b)|x<-wide,p<-[-3,0,3,20],Just b<-[bounds x],let y=limitSize p x]
  report "divAInteger positive denominator"
    [(show(x,n,y),encloses y (l/fromInteger n,u/fromInteger n)) |
      m<-[-9..9],e<-[0..2],s<-[-3..3],n<-[1..9],let x=make m e s,
      Just(l,u)<-[bounds x],let y=divAInteger x n]
#ifndef MBOUND
  let rounded=[make m e s|m<-[-4..4],e<-[0..2],s<-[-2..4]]
  report "floorA" [(show(x,y),encloses y (fromInteger(floor l),fromInteger(floor u))) |
    x<-rounded,Just(l,u)<-[bounds x],let y=floorA x]
  report "ceilingA" [(show(x,y),encloses y (fromInteger(ceiling l),fromInteger(ceiling u))) |
    x<-rounded,Just(l,u)<-[bounds x],let y=ceilingA x]
#endif
  forM_ [("sqrtRec [1,9]",make 5 4 0,[1,3]),
         ("sqrtRec [0,2]",make 1 1 0,[1%2,1]),
         ("sqrtRec exact 4",make 4 0 0,[2])] $ \(label,x,roots) ->
    probe label (let y=sqrtRecA 40 x in show y++", contains reciprocal roots "++
      show [(q,encloses y (1/q,1/q))|q<-roots])
  probe "poly uncertain constant [0,2] at exact zero"
    (let y=poly [make 1 1 0] (make 0 0 0) in show y++", contains full coefficient="++show(encloses y (0,2)))
  probe "poly x+x^2 on [1,2]"
    (let y=poly [0,1,1] (make 3 1 (-1)) in show y++", contains endpoint image="++show(encloses y (2,6)))
  report "poly exact integer quadratic coefficients"
    [(show(a,b,c,x,y),encloses y (minimum vs,maximum vs)) |
      a<-[-2,0,1,2],b<-[-2,0,1,2],c<-[-2,0,1,2],
      m<-[-4..4],e<-[0,1,3],s<-[-4,-1,0,1,4],
      let x=make m e s,Just(l,u)<-[bounds x],
      let f q=fromInteger a+fromInteger b*q+fromInteger c*q*q,
      let vertex=if c==0 then l else (-b)%(2*c),
      let vs=map f ([l,u]++[vertex|l<=vertex,vertex<=u]),
      let y=poly (map fromInteger [a,b,c]) x]
  forM_ [20,100,500] $ \p -> do
    probe ("CR polynomial [1/3] at 0, requested "++show p)
      (let y=require p (polynomial [1/3] 0) in show y++", contains 1/3="++show(encloses y (1%3,1%3)))
    probe ("CR polynomial [0,1/3] at 3, requested "++show p)
      (let y=require p (polynomial [0,1/3] 3) in show y++", contains 1="++show(encloses y (1,1)))
  probe "negate positive infinity" (show (negate (PosInf :: Extended Rational)))
  probe "abs negative infinity" (show (abs (NegInf :: Extended Rational)))
