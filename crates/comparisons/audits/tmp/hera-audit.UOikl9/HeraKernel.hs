{-# LANGUAGE BangPatterns #-}
module Main where
import qualified Data.Number.MPFR as D
import qualified Data.Number.Ball as B
import qualified Data.Number.Real as R
import Data.Order
import Data.Bits
import Data.Ratio
import Data.List (foldl')
import Control.Exception
import Control.Monad
import System.Environment
import System.IO

rat :: D.Dyadic -> Rational
rat d | d == D.zero = 0
      | D.isNaN d || D.isInfinite d = error "nonfinite result"
      | e >= 0 = fromInteger (m `shiftL` e)
      | otherwise = m % (1 `shiftL` negate e)
  where (m,e) = D.decompose d
two :: Int -> Rational
two e | e >= 0 = fromInteger (1 `shiftL` e)
      | otherwise = 1 % (1 `shiftL` negate e)
ends :: B.Ball -> [Rational]
ends (B.Ball c r) = [rat c-rat r,rat c+rat r]
encloses :: B.Ball -> [Rational] -> Bool
encloses b xs = rat (B.radius b)>=0 && head (ends b)<=minimum xs && last (ends b)>=maximum xs
report :: String -> [(Bool,String)] -> IO ()
report label cases = do
  let (!n,!bad,ex) = foldl' step (0::Int,0::Int,[]) cases
      step (!n,!b,ex) (ok,msg) = (n+1,b+if ok then 0 else 1,
                              if not ok && b<3 then ex++[msg] else ex)
  putStrLn $ label++"\tchecks="++show n++"\tfailures="++show bad
  mapM_ (putStrLn . ("  "++)) ex
direction :: D.RoundMode -> Rational -> Rational -> Bool
direction D.Down z q = z<=q
direction D.Up z q = z>=q
direction D.Zero z q = abs z<=abs q && z*q>=0
direction D.Near _ _ = True -- Nearest is separately bracketed by directed results.
ternary :: Int -> Rational -> Rational -> Bool
ternary t z q = compare t 0 == compare z q
ps :: [Word]
ps = [2,3,4,8,16,31,32,53,64,65,128,256]
ms :: [D.RoundMode]
ms = [D.Down,D.Up,D.Zero,D.Near]
inputs :: [(D.Dyadic,Rational)]
inputs = [(D.int2i D.Near 256 m e,fromIntegral m*two e) |
          m<-[-17,-7,-3,-1,0,1,3,7,17],e<-[-127,-17,-1,0,3,129]]
dy :: Int -> Int -> D.Dyadic
dy m e = D.int2i D.Near 512 m e
stagePoint :: Int -> Word -> D.Dyadic
stagePoint m n = dy m (negate (fromIntegral n))

main :: IO ()
main = do
  hSetBuffering stdout LineBuffering
  args <- getArgs
  case args of
    ["integer-grid"] -> report "Num-fromInteger-grid"
      [(rat d==fromInteger i,show(n,offset,sgn,D.getPrec d)) |
       n<-[0,1,2,16,32,63,64,128,256,512,1000,1023,1024,1025,2048,4096],
       offset<-[-17,-1,0,1,17],sgn<-[-1,1],
       let i=sgn*(2^n+offset),let d=fromInteger i :: D.Dyadic]
    ["division-witness"] -> forM_ [16,32,64,128] $ \p -> do
      let r=dy 1 (negate (fromIntegral p `quot` 2+4))
          a=B.Ball (dy (-1) 0) r
          b=B.Ball (dy 1 0) r
          out=B.div p a b :: Maybe B.Ball
      print(p,ends a,ends b,fmap ends out)
    ["finite-sum"] -> print $ R.pCompare (R.infSum (\n -> if n==0 then 3 else 0) (const 0)) 3 16
    ["finite-sum-rec"] -> print $ R.pCompare (R.infSumRec 3 (\_ _ -> (0,0))) 3 16
    ["integer",s] -> do
      let n=read s :: Int
          i=2^n+1 :: Integer
      report "Num-fromInteger" [(rat (fromInteger (sgn*i)) == fromInteger (sgn*i),show (sgn,n)) | sgn<-[-1,1]]
    _ -> kernels

kernels :: IO ()
kernels = do
  report "decompose-independent-inputs" [(rat d==q,show q) | (d,q)<-inputs]
  forM_ [("add",D.add_,(+)),("sub",D.sub_,(-)),("mul",D.mul_,(*)),("div",D.div_,(/))] $ \(name,f,op) ->
    report ("MPFR-"++name++"-direction-ternary-bracket")
      [(direction r z q && ternary t z q && lo<=z && z<=hi,show (p,fromEnum r,xq,yq,z,q,t)) |
       p<-ps,r<-ms,(x,xq)<-inputs,(y,yq)<-inputs,name/="div" || yq/=0,
       let (d,t)=f r p x y,let z=rat d,let q=op xq yq,
       let lo=rat(fst(f D.Down p x y)),let hi=rat(fst(f D.Up p x y))]
  forM_ [("neg",D.neg_,negate),("abs",D.absD_,abs),("sqr",D.sqr_,\x->x*x),("set",D.set_,id)] $ \(name,f,op) ->
    report ("MPFR-"++name)
      [(direction r z q && ternary t z q,show (p,fromEnum r,xq,z,q,t)) |
       p<-ps,r<-ms,(x,xq)<-inputs,let (d,t)=f r p x,let z=rat d,let q=op xq]
  report "MPFR-fma"
    [(direction r z q && ternary t z q,show (p,fromEnum r,xq,yq,zq,z,q,t)) |
     p<-ps,r<-ms,(x,xq)<-inputs,(y,yq)<-take 12 inputs,(zz,zq)<-take 3 inputs,
     let (d,t)=D.fma_ r p x y zz,let z=rat d,let q=xq*yq+zq]
  report "MPFR-binary-scaling"
    [(direction r z q && ternary t z q,show (p,fromEnum r,xq,e,z,q,t)) |
     p<-ps,r<-ms,(x,xq)<-inputs,e<-[-129,-1,0,1,127],
     (f,qe)<-[(D.mul2i_,e),(D.div2i_,negate e)],
     let (d,t)=f r p x e,let z=rat d,let q=xq*two qe]
  report "MPFR-sqrt-rational-square-oracle"
    [(z>=0 && compare t 0==compare (z*z) q &&
       (case r of D.Down -> z*z<=q; D.Up -> z*z>=q; D.Zero -> z*z<=q; _ -> True),
      show (p,fromEnum r,q,z,t)) |
     p<-ps,r<-ms,(x,q)<-inputs,q>=0,let(d,t)=D.sqrt_ r p x,let z=rat d]
  forM_ [("exact-add",(+),(+)),("exact-sub",(-),(-)),("exact-mul",(*),(*))] $ \(name,f,op) ->
    report name [(rat(f x y)==op xq yq,show(xq,yq)) | (x,xq)<-inputs,(y,yq)<-inputs]
  let balls p = [B.Ball (dy c e) (dy (abs c) (e-k)) |
                  c<-[-17,-3,-1,1,3,17],e<-[-100,0,100],
                  k<-[fromIntegral p `quot` 2+4,fromIntegral p+4,2*fromIntegral p+8]]
  report "ball-small-radius-div"
    [(maybe False (\b->encloses b qs) z,show(p,ends a,ends b,fmap ends z)) |
     p<-[4,16,32,64,128,256],a<-balls p,b<-balls p,
     let z=B.div p a b :: Maybe B.Ball,let qs=[x/y|x<-ends a,y<-ends b]]
  report "ball-sqrt-square-oracle"
    [(maybe False valid z,show(p,ends b,fmap ends z)) |
     p<-[4,16,32,64,128,256],b<-balls p,head(ends b)>=0,
     let z=B.sqrt p b :: Maybe B.Ball,
     let valid out = let [lo,hi]=ends out; [a,b']=ends b in
           rat(B.radius out)>=0 && (lo<=0 || lo*lo<=a) && hi>=0 && hi*hi>=b']
  forM_ [("lim",R.lim (\n -> R.fromDyadic (dy 2 0 + stagePoint 1 n)) (\n -> R.fromDyadic(stagePoint 1 n))),
         ("limRat",R.limRat (\n -> dy 2 0+stagePoint 1 n) (stagePoint 1)),
         ("infSum",R.infSum (\n -> R.fromDyadic(stagePoint 1 n)) (\n -> R.fromDyadic(stagePoint 1 n))),
         ("infSumRec",R.infSumRec 1 (\a _ -> let b=a/2 in (b,b)))] $ \(name,x) -> do
    report ("real-"++name++"-no-false-strict")
      [(c==Incomparable,show(n,c))|n<-[2,4,8,16,32,64,128],let c=R.pCompare x 2 n]
    report ("real-"++name++"-requested-accuracy")
      [(case R.approx x n of Right d->abs(rat d-2)<=1%(10^n); Left _ -> False,show n)|n<-[2,4,8,16,32]]
