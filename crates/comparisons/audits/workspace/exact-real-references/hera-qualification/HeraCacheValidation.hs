module Validation where
import qualified Main as Bench
import Control.Monad
main :: IO ()
main = do
  forM_ [0..12] $ \offset -> do
    let x=Bench.build offset
    forM_ [32,8,32,256,8,256,1024,8,1024,32,8,1024] $ \digits -> Bench.validate x digits offset
  putStrLn "PASS 156 independently directed-MPFR radical-sum/history checks"
