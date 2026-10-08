{-# LANGUAGE BangPatterns #-}
module EscardoProbe where
import qualified Escardo as E
import Data.Ratio
import Data.List (foldl')
import Control.Monad
import Control.Exception
import System.Environment
import System.IO

prefix :: Int -> [Int] -> Rational
prefix n xs = foldr (\d q -> (fromIntegral d+q)/2) 0 (take n xs)
digits :: Rational -> E.I
digits q | q == 0 = E.zero
         | q >= 1/2 = 1:digits(2*q-1)
         | q <= -1/2 = -1:digits(2*q+1)
         | otherwise = 0:digits(2*q)
report :: String -> [(Bool,String)] -> IO ()
report label cases=do
  let (!n,!bad,examples)=foldl' step (0::Int,0::Int,[]) cases
      step (!n,!b,es) (ok,msg)=(n+1,b+if ok then 0 else 1,
                                 if not ok && b<3 then es++[msg] else es)
  putStrLn(label++"\tchecks="++show n++"\tfailures="++show bad)
  mapM_ (putStrLn . ("  "++)) examples
check :: Int -> E.I -> Rational -> Bool
check n xs q = let ds=take n xs in all (\d -> abs d<=1) ds
                           && abs(prefix n ds-q)<=1%(2^n)
clamp :: Rational -> Rational
clamp=max(-1).min 1
streams :: Int -> [(E.I,Rational)]
streams n=[(ds++repeat t,prefix n ds+fromIntegral t/(2^n))|
           ds<-replicateM n [-1,0,1],t<-[-1,0,1]]

main :: IO ()
main=do
  hSetBuffering stdout LineBuffering
  args<-getArgs
  case args of
    ["sqrt-zero"] -> print(take 32(E.squareRoot E.zero))
    ["negative-zero"] -> print(E.negative E.zero)
    ["decimal-half"] -> print(take 4(E.decimal E.half))
    ["bisection-zero"] -> print(take 4(E.bisection id))
    ["bug-examples"] -> print(E.example8,E.example9,E.example11)
    ["witnesses"] -> do
      forM_ [("mul2",E.mul E.half (1:1:E.zero),3%8),
             ("imin",E.imin (-1:1:E.zero) (0:(-1):(-1):E.zero),(-3)%8),
             ("sqrt0",E.squareRoot E.zero,0),
             ("sqrt1/16",E.squareRoot(0:0:0:1:E.zero),1%4),
             ("fromDouble0",E.fromDouble 0,0),
             ("fromDouble1",E.fromDouble 1,1)] $ \(label,x,q) ->
        putStrLn(label++"\t"++show(prefix 128 x)++"\texpected="++show q)
    ["functionals"] -> functionals
    ["small-roots"] -> report "squareRoot exact rational enclosures"
      [(let y=prefix n ds;e=1%(2^n) in all (\d->abs d<=1) ds
             && y+e>=0 && (y-e<=0 || (y-e)^2<=q) && q<=(y+e)^2,show(q,n,ds))|
       q<-[0,1%64,1%16,1%4,1%2,1],n<-[4,8,16],let ds=take n(E.squareRoot(digits q))]
    _ -> ordinary

ordinary :: IO ()
ordinary=do
  let ss=streams 3
      ns=[2,5,12]
  forM_ [("compl",E.compl,negate),("addOne",E.addOne,clamp . (+1)),
         ("subOne",E.subOne,clamp . subtract 1),("oneMinus",E.oneMinus,clamp . (1-)),
         ("mulBy2",E.mulBy2,clamp . (*2)),("mulBy4",E.mulBy4,clamp . (*4)),
         ("sqr",E.sqr,(^2)),("iabs",E.iabs,abs),("pabs",E.pabs,abs),
         ("znorm",E.znorm,id),("idAffine",E.idAffine,id),
         ("complAffine",E.complAffine,negate)] $ \(label,f,truth)->
    report label [(check n (f x) (truth q),show(q,n))|(x,q)<-ss,n<-ns]
  forM_ [("mid",E.mid,\a b->(a+b)/2),("tadd",E.tadd,\a b->clamp(a+b)),
         ("imin",E.imin,min),("imax",E.imax,max),("pmax",E.pmax,max),
         ("mul0",E.mul_version0,(*)),("mul1",E.mul_version1,(*)),
         ("mul2",E.mul_version2,(*)),("mul3",E.mul_version3,(*))] $ \(label,f,truth)->
    report label [(check n (f x y) (truth a b),show(a,b,n))|(x,a)<-ss,(y,b)<-ss,n<-ns]
  report "divByInt" [(check k (E.divByInt x n) (q/fromIntegral n),show(q,n,k))|
                    (x,q)<-ss,n<-[1,2,3,7,31,1024],k<-ns]
  report "tMulByInt" [(check k (E.tMulByInt x n) (clamp(q*fromIntegral n)),show(q,n,k))|
                     (x,q)<-ss,n<-[0,1,2,3,7,31,1024],k<-ns]
  report "mulByInt" [(let(a,y)=E.mulByInt x n in check k y (q*fromIntegral n-fromIntegral a),show(q,n,k))|
                    (x,q)<-ss,n<-[1,2,3,7,10,31,1024],k<-ns]
  report "bigMid-constant-periodic" [(check k (E.bigMid(cycle xs)) truth,show(qs,k))|
    qs<-[[-1],[-1,1],[1,0,0],[-1%3,1%4,5%8],[0,0,0],[1]],
    let xs=map digits qs,let l=length qs,let truth=sum[ q/2^i|(q,i)<-zip qs [1..l]]/(1-1/(2^l)),k<-ns]
  report "fromDouble-exact-import" [(check k (E.fromDouble d) (toRational d),show(d,k,prefix k(E.fromDouble d)))|
    exponent<-[0,-1,-8,-52,-53,-54,-55,-56,-64,-128,-1022,-1074],s<-[-1,1],
    let d=s*encodeFloat 1 exponent::Double,k<-[16,55,64,128,1100]]

functionals :: IO ()
functionals=do
  let fs=[("constant",const(digits(3%8)),3%8,3%8,3%8),
          ("identity",id,1,-1,0),
          ("square",E.sqr,1,0,1%3),
          ("abs",E.pabs,1,0,1%2),
          ("negative-square",E.compl . E.sqr,0,-1,(-1)%3),
          ("linear-quarter",\x->E.mid(0:x)(digits((-1)%2)),0,(-1)%2,(-1)%4)]
  forM_ fs $ \(label,f,hi,lo,integral)->do
    forM_ [("sup",E.supremum f,hi),("inf",E.infimum f,lo),
           ("integral",E.halfIntegral f,integral)] $ \(op,x,q)->
      report (label++"-"++op) [(check n x q,show(n,prefix n x,q))|n<-[1,2,4,6]]
  report "quantifier-finite-prefix"
    [(E.forSomeI p==any p allBinary && E.forEveryI p==all p allBinary &&
      E.forSomeI' p==any p allSigned && E.forEveryI' p==all p allSigned,show(n,t))|
      n<-[1..6],t<-[-n..n],let p x=sum(take n x)>t,
      let allBinary=map(++E.zero)(replicateM n [-1,1]),
      let allSigned=map(++E.zero)(replicateM n [-1,0,1])]
  report "trisection-affine-rational-roots"
    [(check n (E.trisection(\x->E.mid x(E.compl(digits q)))) q,show(q,n))|
     q<-[-1,-3%4,-1%2,-1%3,0,1%3,1%2,3%4,1],n<-[2,8,16,32]]
