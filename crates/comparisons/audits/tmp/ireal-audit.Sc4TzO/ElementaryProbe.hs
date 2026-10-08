{-# LANGUAGE ForeignFunctionInterface #-}
module Main where
import Data.Number.IReal
import Data.Number.IReal.IReal (appr)
import Data.Number.IReal.IntegerInterval
import qualified Erf
import Data.Ratio
import Data.Bits
import Control.Exception
import Control.Monad
import System.Timeout
import Foreign.C.String
import Foreign.C.Types
import Data.IORef
import System.IO

foreign import ccall unsafe "cdar_oracle" oracle :: CString -> CString -> CString -> CString -> IO CInt
rat q = show (numerator q) ++ "/" ++ show (denominator q)
ops :: [(String, IReal -> IReal)]
ops = [("exp",exp),("log",log),("sin",sin),("cos",cos),("atan",atan),
       ("asin",asin),("acos",acos),("sqrt",sqrt),("erf",Erf.erf)]
valid op x = case op of
  "log" -> x>0
  "sqrt" -> x>=0
  "asin" -> abs x<=1
  "acos" -> abs x<=1
  "exp" -> abs x<=64
  "erf" -> abs x<=32
  _ -> True
points = [n%8 | n<-[-24..24]] ++ [s*(1%bit k) | s<-[-1,1],k<-[20,100,300]]
         ++ [s*(1-1%bit k) | s<-[-1,1],k<-[20,100]]
         ++ [s*(bit k%1) | s<-[-1,1],k<-[5,12,50]]
intervals = [(m-r,m+r) | m<-[-4,-1,0,1,4],r<-[1%16,1%2,2]]
ps = [0,1,2,8,53,128,512]
forceBounds x p = let I(l,u)=appr x p in evaluate (l+u) >> return (l%bit p,u%bit p)
check op q l u = withCString op $ \a -> withCString (rat q) $ \b ->
                withCString (rat l) $ \c -> withCString (rat u) $ \d -> oracle a b c d
run stats label op qs x p = do
  attempt <- try (timeout 200000 $ do
    (l,u)<-forceBounds x p
    codes<-mapM (\q->check op q l u) qs
    evaluate (if 1 `elem` codes then 1 else maximum codes) >>= \c -> return (c,l,u))
    :: IO (Either SomeException (Maybe (CInt,Rational,Rational)))
  let (status,detail)=case attempt of
        Left e -> ("EXCEPTION",show e)
        Right Nothing -> ("TIMEOUT","")
        Right (Just (c,l,u)) -> (if c==0 then "PASS" else if c==1 then "FAIL" else "UNRESOLVED",rat l++" "++rat u)
  modifyIORef' stats (status:)
  when (status/="PASS") $ putStrLn $ unwords [status,label,op,show p,show (map rat qs),detail]
main = do
  hSetBuffering stdout LineBuffering
  stats<-newIORef []
  forM_ ops $ \(op,f)-> do
    forM_ (filter (valid op) points) $ \q-> do
      let x=f (fromRational q)
      forM_ ps $ run stats "point" op [q] x
      forM_ [128,2,0] $ run stats "warm-coarse" op [q] x
    forM_ (filter (\(l,u)->valid op l&&valid op u) intervals) $ \(l,u)-> do
      let x=f (((l+u)/2) +- ((u-l)/2))
          samples=[l+(u-l)*k/16 | k<-[0..16]]
      forM_ [0,2,8,53,128] $ run stats "interval" op samples x
    putStrLn ("DONE "++op)
  forM_ ps $ run stats "constant" "pi" [0] pi
  counts<-readIORef stats
  forM_ ["PASS","FAIL","TIMEOUT","EXCEPTION","UNRESOLVED"] $ \s->
    putStrLn ("TOTAL "++s++" "++show (length (filter (==s) counts)))
