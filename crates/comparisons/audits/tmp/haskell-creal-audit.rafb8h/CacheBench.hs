{-# LANGUAGE DataKinds #-}
module Main where
import Data.CReal.Internal
import Control.Concurrent.MVar
import Control.Exception
import Control.Monad
import System.CPUTime
import System.Environment
import System.Mem
import GHC.Stats

type R = CReal 30
main :: IO ()
main = do
  [name,variant,bitsText] <- getArgs
  let bits=read bitsText
      input=sin (fromRational (17/32) :: R)
      op=case name of
        "negate" -> negate
        "abs" -> abs
        "integer-add" -> flip plusInteger 7
        "square" -> squareBounded
        _ -> error "unknown operation"
  _ <- evaluate (atPrecision input (bits+16))
  inputBox <- newMVar input
  let action = do
        -- The IO read keeps each constructor application inside the measured
        -- action; otherwise a pure op input thunk could be shared by repeats.
        argument <- readMVar inputBox
        result@(CR _ fn) <- evaluate (op argument)
        value <- case variant of
          "seeded" -> pure result
          "cleared" -> do cache<-newMVar Never; pure (CR cache fn)
          _ -> error "unknown cache variant"
        evaluate (atPrecision value bits)
  _ <- action
  performGC
  statsBefore<-getRTSStats
  start<-getCPUTime
  let loop n = do
        replicateM_ 64 action
        stop<-getCPUTime
        if stop-start>=200000000000 then pure (n+64,stop)
          else loop (n+64)
  (count,stop)<-loop (0::Integer)
  performGC
  statsAfter<-getRTSStats
  putStrLn $ unwords [name,variant,show bits,show count,
    show (fromIntegral (stop-start) / (1000*fromIntegral count) :: Double),
    show (fromIntegral (allocated_bytes statsAfter-allocated_bytes statsBefore) / fromIntegral count :: Double)]
