{-# LANGUAGE DataKinds #-}
module Main where
import Data.CReal.Internal
import Data.CReal.Converge
import Data.Ratio
import qualified Data.Bits as B
import Control.Exception
import Control.Concurrent.MVar
import Control.Monad
import Data.IORef
import Data.List (nub)
import System.IO
import System.Timeout

type R = CReal 30
ps :: [Int]
ps = [0,1,2,8,53,128,256]
qs :: [Rational]
qs = nub [n % d | n <- [-12..12], d <- [1,2,3,7,16]]

main :: IO ()
main = do
  hSetBuffering stdout LineBuffering
  counts <- newIORef (0::Int,0::Int)
  let check label expected value p = do
        n <- evaluate (atPrecision value p)
        let ok = abs (fromInteger n - expected * fromInteger (B.bit p)) <= 1
        (total,failures) <- readIORef counts
        writeIORef counts (total+1,failures + if ok then 0 else 1)
        when (not ok && failures < 30) $
          print ("FAIL",label,p,expected,n)
      unary label exact op = forM_ qs $ \q -> forM_ ps $ \p ->
        check (label, q, q) (exact q) (op (fromRational q :: R)) p
      binary label exact op = forM_ qs $ \q -> forM_ qs $ \r -> forM_ ps $ \p ->
        check (label,q,r) (exact q r) (op (fromRational q :: R) (fromRational r)) p
  binary "add" (+) (+)
  binary "sub" (-) (-)
  binary "mul" (*) (*)
  binary "min" min min
  binary "max" max max
  unary "square" (\q -> q*q) square
  unary "negate" negate negate
  unary "abs" abs abs
  forM_ (filter (/=0) qs) $ \q -> forM_ ps $ \p ->
    check ("recip",q,q) (recip q) (recip (fromRational q :: R)) p
  forM_ qs $ \q -> forM_ [-32,-8,-1,0,1,8,32] $ \s -> forM_ ps $ \p ->
    check ("shift",q,fromIntegral s) (q * 2^^s)
      (shiftL (fromRational q :: R) s) p
  forM_ qs $ \q -> do
    let value = square (fromRational q :: R) + fromRational (q/3)
    forM_ [256,0,128,2,53,1,8] $ check ("history",q,q) (q*q+q/3) value
  readIORef counts >>= print . (,) "ARITHMETIC total,failures"
  -- Disjoint enclosures at two precisions refute the existence of one
  -- retained real value, independently of donor approximate Eq/Ord.
  let histories label make = forM_ [[2,16,2],[16,2,16]] $ \order -> do
        CR _ fn <- evaluate make
        cache <- newMVar Never
        let value = CR cache fn
        ns <- mapM (evaluate . atPrecision value) order
        let intervals = zipWith (\p n -> ((n-1)%B.bit p,(n+1)%B.bit p)) order ns
        print (label,order,ns,intervals)
  histories "signum ascending" (signum (fromRational (1%1024) :: R))
  histories "atan2 tiny diagonal" (atan2 (fromRational (1%1024) :: R) (fromRational (1%1024)))
  let plateau = replicate 2 (0 :: R) ++ repeat 1
  case converge plateau of
    Nothing -> error "unexpected empty convergence"
    Just value -> print ("CONVERGE plateau whose true limit is 1",atPrecision value 128)
  forM_ [("recip zero",recip 0),("log zero",log 0),("sqrt negative",sqrt (-1))] $ \(name,v) -> do
    result <- try (timeout 200000 (evaluate (atPrecision (v :: R) 32)))
      :: IO (Either SomeException (Maybe Integer))
    print (name,result)
