{-# LANGUAGE DataKinds, ForeignFunctionInterface #-}
module Main where
import Data.CReal.Internal
import qualified Data.Bits as B
import Data.Ratio
import Data.List (nub)
import Control.Exception
import Control.Concurrent.MVar
import Control.Monad
import Data.IORef
import Foreign.C.String
import Foreign.C.Types
import System.Timeout
import System.IO

type R = CReal 30
foreign import ccall unsafe "creal_oracle" oracle :: CString -> CString -> CString -> CString -> IO CInt
rat q = show (numerator q) ++ "/" ++ show (denominator q)
ops :: [(String,String,R -> R)]
ops = [(n,n,f) | (n,f) <- [("exp",exp),("log",log),("sqrt",sqrt),
  ("sin",sin),("cos",cos),("tan",tan),("asin",asin),("acos",acos),("atan",atan),
  ("sinh",sinh),("cosh",cosh),("tanh",tanh),("asinh",asinh),("acosh",acosh),("atanh",atanh)]]
  ++ [("expBounded","exp",expBounded),("logBounded","log",logBounded),
      ("sinBounded","sin",sinBounded),("cosBounded","cos",cosBounded),
      ("atanBounded","atan",atanBounded)]
valid name q = case name of
  "log" -> q>0
  "sqrt" -> q>=0
  "asin" -> abs q<=1
  "acos" -> abs q<=1
  "acosh" -> q>=1
  "atanh" -> abs q<1
  "exp" -> abs q<=64
  "sinh" -> abs q<=64
  "cosh" -> abs q<=64
  "tanh" -> abs q<=64
  "logBounded" -> q>=2/3 && q<=2
  _ | name `elem` ["expBounded","sinBounded","cosBounded","atanBounded"] -> abs q<=1
  _ -> True
points = nub $ [n%8 | n<-[-24..24]] ++
  [s*(1%B.bit k) | s<-[-1,1],k<-[20,100,300]] ++
  [s*(1-1%B.bit k) | s<-[-1,1],k<-[20,100]] ++
  [s*(B.bit k%1) | s<-[-1,1],k<-[5,12,50]]
ps = [0,1,2,8,53,128,512]

main = do
  hSetBuffering stdout LineBuffering
  counts <- newIORef ([] :: [String])
  let run label op q value p = do
        result <- try (timeout 200000 $ do
          n <- evaluate (atPrecision value p)
          let lo=(n-1)%B.bit p; hi=(n+1)%B.bit p
          code <- withCString op $ \a -> withCString (rat q) $ \b ->
            withCString (rat lo) $ \c -> withCString (rat hi) $ \d -> oracle a b c d
          pure (code,n)) :: IO (Either SomeException (Maybe (CInt,Integer)))
        let (status,detail)=case result of
              Left e -> ("EXCEPTION",show e)
              Right Nothing -> ("TIMEOUT","")
              Right (Just (c,n)) -> (if c==0 then "PASS" else if c==1 then "FAIL" else "UNRESOLVED",show n)
        modifyIORef' counts (status:)
        when (status/="PASS") $ print (status,label,op,q,p,detail)
      -- Fresh result cache isolates cold requests even if the compiler shares
      -- a pure expression thunk. Child caches remain valid approximations.
      fresh (CR _ fn) = do cache<-newMVar Never; pure (CR cache fn)
  forM_ ops $ \(name,op,f) -> do
    forM_ (filter (valid name) points) $ \q -> do
      forM_ ps $ \p -> do
        v<-fresh (f (fromRational q))
        run (name++" cold-root") op q v p
      ascending<-fresh (f (fromRational q))
      forM_ ps $ run (name++" ascending") op q ascending
      descending<-fresh (f (fromRational q))
      forM_ [512,128,2,0] $ run (name++" descending") op q descending
    putStrLn ("DONE "++name)
  forM_ ps $ \p -> fresh (pi :: R) >>= \v -> run "pi" "pi" 0 v p
  statuses<-readIORef counts
  forM_ ["PASS","FAIL","TIMEOUT","EXCEPTION","UNRESOLVED"] $ \s ->
    putStrLn ("TOTAL "++s++" "++show (length (filter (==s) statuses)))
