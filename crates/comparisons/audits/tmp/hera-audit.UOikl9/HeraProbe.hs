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

rat :: D.Dyadic -> Rational
rat d | d == D.zero = 0
      | D.isNaN d || D.isInfinite d = error "nonfinite result"
      | e >= 0 = fromInteger (m `shiftL` e)
      | otherwise = m % (1 `shiftL` negate e)
  where (m,e) = D.decompose d

point :: Integer -> D.Dyadic
point n = D.fromIntegerA D.Near 256 n
mk :: Integer -> Integer -> B.Ball
mk c r = B.Ball (point c) (D.div2i D.Near 256 (point r) 3)
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

guarded :: String -> IO () -> IO ()
guarded label action = catch action $ \e -> putStrLn (label++"\tEXCEPTION "++displayException (e::SomeException))

main :: IO ()
main = do
  let balls = [mk c r | c <- [-3..3], r <- [0,1,8,32]]
      operations = [("add",B.add,(+)),("sub",B.sub,(-)),("mul",B.mul,(*))]
  forM_ operations $ \(name,f,oracle) -> guarded name $ report ("ball-"++name)
    [(encloses z xs,show (p,ends a,ends b,ends z,xs)) |
       p <- [4,16,64,128], a <- balls,b <- balls,
       let z=f p a b,let xs=[oracle x y | x<-ends a,y<-ends b]]
  guarded "div" $ report "ball-div"
    [(maybe False (\z->encloses z xs) z,show (p,ends a,ends b,fmap ends z,xs)) |
       p <- [4,16,64,128],a <- balls,b <- balls,
       head (ends b)*last (ends b)>0,
       let z=B.div p a b :: Maybe B.Ball,
       let xs=[x/y | x<-ends a,y<-ends b]]
  guarded "abs" $ report "ball-abs"
    [(encloses z xs,show (p,ends b,ends z)) |
       p <- [4,16,64,128],b <- balls,
       let es=ends b,let xs=map abs es ++ [0 | head es<=0 && last es>=0],let z=B.absB p b]
  guarded "exp" $ report "ball-exp-known-zero"
    [(encloses z [1],show (p,ends b,ends z)) |
       p <- [16,64,128],b <- balls,head (ends b)<=0 && last (ends b)>=0,
       let z=B.exp p b]
  guarded "log" $ report "ball-log-known-one"
    [(maybe False (\z -> encloses z [0]) z,show (p,ends b,fmap ends z)) |
       p <- [16,64,128],b <- balls,head (ends b)>0 && head (ends b)<=1 && last (ends b)>=1,
       let z=B.log p b :: Maybe B.Ball]
  guarded "constants" $ report "named-MPFR-constants"
    [(lo<=q && q<=hi,show (name,p,q)) |
       (name,f,lo,hi) <- [("log2",D.log2c,69%100,70%100),
                          ("euler",D.euler,57%100,58%100),
                          ("catalan",D.catalan,91%100,92%100)],
       p <- [32,64,128],let q=rat(f D.Near p)]
  guarded "zero" $ report "MPFR-isZero" [(D.isZero D.zero,"zero predicate returned false")]
  guarded "compose" $ report "compose-decompose"
    [(rat(D.compose D.Near 256 (D.decompose x)) == rat x,show (rat x)) |
       m <- [-7,-3,1,3,7],e <- [-10,0,10],let x=D.int2i D.Near 128 m e]
  guarded "word" $ report "Ball-fromWord"
    [(encloses (B.fromWord 128 w) [fromIntegral w],show w) | w <- [0,1,2^31,2^63,maxBound]]
  forM_ [("fromInt",R.fromInt(2^40+1),fromInteger(2^40+1)),
         ("fromWord",R.fromWord(2^40+1),fromInteger(2^40+1)),
         ("decimal",R.fromString "0.1",1%10)] $ \(name,x,q) -> guarded name $
      report ("real-import-"++name)
        [(abs(rat d-q)<=1%(10^digits),show (digits,rat d,q)) |
            digits<-[3,8,16],let d=either fst id (R.approx x digits)]
  guarded "limRec" $ do
    let f _ n = (R.fromInt 10,R.fromDyadic (D.int2i D.Near 64 10 (1-fromIntegral n)))
        x = R.limRec (R.fromInt 0) f
    report "limRec-valid-eventually-constant"
      [(result==Incomparable,show (n,result)) | n<-[1,2,4,16,64],
        let result=R.pCompare x (R.fromInt 10) n]
