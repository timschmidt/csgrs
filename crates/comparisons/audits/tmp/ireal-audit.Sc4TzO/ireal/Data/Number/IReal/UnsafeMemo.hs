module Data.Number.IReal.UnsafeMemo where

import Data.Number.IReal.Scalable

import Control.Concurrent.MVar (newMVar, readMVar, modifyMVar_)
import System.IO.Unsafe (unsafePerformIO)

-- | Memoizing a function which satisfies the property @f (p-r) = scale (f p) (-r)@ for all @r > 0@.
-- Unsafe; based on ideas from Lennart Augustsson's uglymemo.
unsafeMemo :: Scalable a => (Int -> a) -> Int -> a
unsafeMemo f = unsafePerformIO . unsafePerformIO (memoIO f)


memoIO :: Scalable a => (Int -> a) -> IO (Int -> IO a)
memoIO f = do 
  v <- newMVar Nothing
  return (\p -> 
    if p<0 
    then error ("Excessive precision required: " ++ show (fromIntegral p + 2^32))  
    else do
        m <- readMVar v
        case m of
          Just (q,y)
            |p <= q -> return (scale y (p - q))
          _ ->  do let z = f p 
                   modifyMVar_ v (const (return (Just (p,z))))
                   return z)
