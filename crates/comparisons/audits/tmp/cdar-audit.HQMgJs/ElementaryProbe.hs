{-# LANGUAGE CPP, ForeignFunctionInterface #-}
module Main where
import Control.Applicative (getZipList)
import Control.DeepSeq (force)
import Control.Exception (SomeException,evaluate,try)
import Control.Monad (forM,forM_)
import Data.Bits (shiftL)
import Data.List (nub)
import Data.Ratio ((%),numerator,denominator)
import Data.CDAR
import Foreign.C.String (CString,withCString)
import Foreign.C.Types (CInt(..))
import System.IO (stdout,hSetBuffering,BufferMode(LineBuffering))
import System.Timeout (timeout)

foreign import ccall unsafe "cdar_oracle" oracle
  :: CString -> CString -> CString -> CString -> IO CInt

make :: Int -> Integer -> Integer -> Int -> Approx
#ifdef MBOUND
make = approxMB
#else
make _ = Approx
#endif

bounds :: Approx -> Maybe (Rational,Rational)
bounds Bottom = Nothing
#ifdef MBOUND
bounds (Approx _ m e s) = Just (fromInteger (m-e)*2^^s,fromInteger (m+e)*2^^s)
#else
bounds (Approx m e s) = Just (fromInteger (m-e)*2^^s,fromInteger (m+e)*2^^s)
#endif

contains :: Approx -> (Rational,Rational) -> Bool
contains a (l,u) = maybe False (\(x,y)->x<=l && u<=y) (bounds a)

bounded :: Approx -> IO (Either String Approx)
bounded value = do
  result <- try (timeout 300000 (evaluate (force value))) :: IO (Either SomeException (Maybe Approx))
  pure $ case result of
    Left e -> Left ("EXCEPTION "++show e)
    Right Nothing -> Left "TIMEOUT"
    Right (Just a) -> Right a

qString :: Rational -> String
qString q = show(numerator q)++"/"++show(denominator q)

check :: String -> Approx -> [Rational] -> IO String
check _ Bottom _ = pure "BOTTOM"
check op value points = case bounds value of
  Nothing -> pure "BOTTOM"
  Just(l,u) -> do
    codes <- forM points $ \q -> withCString op $ \o ->
      withCString (qString q) $ \x -> withCString (qString l) $ \a ->
      withCString (qString u) $ \b -> oracle o x a b
    pure $ if 1 `elem` codes then "FAIL" else
      if 2 `elem` codes then "UNRESOLVED" else "PASS"

report :: String -> [(String,Approx,[Rational])] -> String -> IO ()
report label cases op = do
  answers <- forM cases $ \(tag,value,points) -> do
    result <- bounded value
    case result of
      Left err -> pure (err,tag)
      Right a -> do
        status <- check op a points
        pure (status,tag++" => "++show a)
  let counts=[(s,length [()|(s',_)<-answers,s==s'])|s<-nub(map fst answers)]
  putStrLn(label++" "++show counts)
  forM_ (take 4 [(s,t)|(s,t)<-answers,s/="PASS"]) $ \x -> print x

approxFns :: Int -> [(String,Approx->Approx)]
#ifdef MBOUND
approxFns p = [("exp",expA),("log",logA),("sin",sinA),("cos",cosA),
               ("sqrt",sqrtA),("rsqrt",sqrtRecA p),("atan",atanA p)]
#else
approxFns p = [("exp",expA p),("log",logA p),("sin",sinA p),("cos",cosA p),
               ("sqrt",sqrtA p),("rsqrt",sqrtRecA p),("atan",atanA p)]
#endif

valid :: String -> (Rational,Rational) -> Bool
valid "log" (l,_) = l>0
valid "sqrt" (l,_) = l>=0
valid "rsqrt" (l,_) = l>0
valid "asin" (l,u) = l>=(-1) && u<=1
valid "acos" (l,u) = l>=(-1) && u<=1
valid _ _ = True

capChecks :: IO ()
capChecks = do
#ifdef MBOUND
  let ms=nub [sgn*((1 `shiftL` b)+offset)|sgn<-[-1,1],b<-[0..12]++[31,53,80,128],offset<-[-1,0,1]]
      raw=[(mb,m,e,s)|mb<-[2,3,5,8,10,53,80],m<-ms,e<-[0,1,3,255,1023,1024,65537],s<-[-100,0,100]]
      checks=[(tag,a,b)|tag@(mb,m,e,s)<-raw,let a=make mb m e s,
               let b=(fromInteger(m-e)*2^^s,fromInteger(m+e)*2^^s)]
      capOK (Approx mb m _ _) = abs m<=2^mb
      capOK Bottom = False
      failed=[(tag,a)| (tag,a,b)<-checks,not(contains a b && capOK a)]
  putStrLn("cap containment+budget "++show(length checks)++" cases, failures="++show(length failed))
  mapM_ print (take 5 failed)
  forM_ [("boundErrorTerm",boundErrorTerm),("boundErrorTermMB",boundErrorTermMB),
         ("limitSize -8",limitSize (-8)),("limitSize 8",limitSize 8)] $ \(label,f) -> do
    let bad=[(tag,a,y)|(tag,a,_)<-checks,Just b<-[bounds a],let y=f a,not(contains y b)]
    putStrLn(label++" "++show(length checks)++" cases, containment failures="++show(length bad))
    mapM_ print (take 3 bad)
  forM_ [2,3,5,8,10] $ \mb -> do
    let xs=[make mb m e s|m<-[-1025,-33,-3,0,3,33,1025],e<-[0,1,33],s<-[-10,0,10]]
    forM_ [("add",(+),(\l u a b->(l+a,u+b))),
           ("mul",(*),(\l u a b->(minimum[l*a,l*b,u*a,u*b],maximum[l*a,l*b,u*a,u*b])))] $
      \(label,op,ref) -> do
        let bad=[(x,y,z)|x<-xs,y<-xs,Just(l,u)<-[bounds x],Just(a,b)<-[bounds y],let z=op x y,not(contains z (ref l u a b))]
        putStrLn("mb="++show mb++" "++label++" "++show(length xs^2)++" cases, failures="++show(length bad))
        mapM_ print (take 3 bad)
#else
  putStrLn "master: no mantissa cap API"
#endif

main :: IO ()
main = do
  hSetBuffering stdout LineBuffering
  capChecks
  forM_ [32,100] $ \p -> do
    let grid=[make p m e s|m<-[-8..8],e<-[0,1],s<-[-3,0]]
    forM_ (approxFns p) $ \(op,f) ->
      report ("Approx "++op++" p="++show p)
        [(show x,f x,[l,l+(u-l)/4,(l+u)/2,u-(u-l)/4,u])|
          x<-grid,Just(l,u)<-[bounds x],valid op (l,u)] op
  let fns=[("exp",exp),("log",log),("sin",sin),("cos",cos),
           ("sqrt",sqrt),("atan",atan),("asin",asin),("acos",acos)]
  forM_ [20,100,300] $ \p -> forM_ fns $ \(op,f) ->
    report ("CR "++op++" p="++show p)
      [(show q,require p (f (fromRational q)),[q])|
        q<-nub([n%8|n<-[-16..16]]++[-8,8,128]),valid op(q,q)] op
  forM_ [("sqrtCR","sqrt",sqrtCR),("atanCR","atan",atanCR),
         ("expCR","exp",expCR),("sinCR","sin",sinCR),("cosCR","cos",cosCR)] $ \(name,op,f) ->
    report name [(show q,require 30 (f(fromRational q)),[q])|
      q<-[-2,-1,-1%2,0,1%2,1,2],valid op(q,q)] op
  forM_ [("pi",pi),("piMachinCR",piMachinCR),("piBorweinCR",piBorweinCR),
         ("piCRMachin",piCRMachin)] $ \(name,x) ->
    report name [(show p,require p x,[0])|p<-[20,100,300]] "pi"
  forM_ [("infinity",1/0),("negative infinity",-1/0),("NaN",0/0),
         ("zero",0),("negative zero",-0),("largest finite",encodeFloat (2^53-1) 971)] $ \(name,x) ->
    forM_ [("fromDouble",fromDouble),("fromDoubleAsExactValue",fromDoubleAsExactValue)] $ \(label,f) -> do
      value <- bounded (head(getZipList(unCR(f x))))
      putStrLn(label++" "++name++" "++show value)
