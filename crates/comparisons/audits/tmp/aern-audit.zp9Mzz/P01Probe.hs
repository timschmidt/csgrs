module P01Probe where
import qualified AuditP01Hmpfr as H
import qualified AuditP01Simple as S
import qualified Numeric.AERN.MPFRBasis.Interval as I
import Numeric.AERN.RealArithmetic.Basis.MPFR ()
import Data.Number.MPFR.Instances.Up ()
import Control.DeepSeq (force)
import Control.Exception (SomeException, evaluate, try)
import Control.Monad (forM_, forM)
import Data.Ratio ((%))
import System.Environment (getArgs)
import System.IO (stdout,hSetBuffering,BufferMode(LineBuffering))
import System.Timeout (timeout)

fromQ p q = q I.|<*> (I.setPrecOut p 1) :: I.MI
check name q n value = do
  let (lo,hi) = I.getEndpoints value
      l = toRational lo
      u = toRational hi
      valid = 0 <= l && l <= u && l^n <= q && q <= u^n
  putStrLn $ show (name,q,n,"valid",valid,"bounds",I.doubleBounds value)
  pure valid
bounded name value = do
  result <- timeout 1500000 (try (evaluate (force value)) :: IO (Either SomeException String))
  putStrLn (name ++ ": " ++ case result of
    Nothing -> "TIMEOUT"
    Just (Left e) -> "EXCEPTION " ++ show e
    Just (Right s) -> s)

main = do
  hSetBuffering stdout LineBuffering
  args <- getArgs
  case args of
    ["boundary"] -> do
      bounded "H zero numerator/zero denominator" (show (head (H.benchmark (H.Params 20 0 0 2))))
      bounded "H uncertain input bisection" (show (H.bisectUntilClose (I.fromEndpoints (1,4)) 2))
      bounded "S negative odd root" (show (S.approximation (S.realRootFinder (S.realFromRational (-1)) 3) 0))
      bounded "S zero root" (show (S.approximation (S.realRootFinder (S.realFromRational 0) 2) 0))
    _ -> do
      results <- forM [(p,q,n) | p <- [50,128,256], q <- [1,2,3,5%2,1%16,100], n <- [2,3,7,16]] $ \(p,q,n) -> do
        let x = fromQ p q
            seed = fromQ p (q+1)
        a <- check ("H direct p=" ++ show p) q n (fst (H.nthRootNewton n x seed))
        b <- check ("S direct p=" ++ show p) q n (S.newton x seed n)
        pure (a,b)
      putStrLn ("direct failures H/S " ++ show (length (filter (not.fst) results), length (filter (not.snd) results)))
      forM_ [(a,b,n) | (a,b) <- [(2,1),(5,2),(1,16),(100,1)], n <- [2,3,7,16]] $ \(a,b,n) -> do
        let q = toInteger a % toInteger b
            (_,_,v,_) = last (H.benchmark (H.Params 30 a b n))
            s = S.approximation (S.realRootFinder (S.realFromRational q) n) 3
        _ <- check "H benchmark" q n v
        _ <- check "S realRootFinder" q n s
        pure ()
