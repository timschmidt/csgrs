module Main where
import qualified Data.Reals.Lipschitz as L
import qualified Data.Reals.Space as P
import qualified Data.Reals.Staged as S
import qualified Data.Approximate.ApproximateField as A
import Control.Arrow
import Control.Monad

metric :: (Float,Float) -> (Float,Float) -> Float
metric (a,b) (c,d) = max (abs (a-c)) (abs (b-d))
main :: IO ()
main = do
  forM_ [("first",first),("second",second)] $ \(name,liftArrow) -> do
    let f = liftArrow (L.Lipschitz (const 0) (const 0))
        x = (0,0)
        y = (1,1)
        bound = L.scaling f x * metric x y
        distance = metric (L.apply f x) (L.apply f y)
    putStrLn $ name ++ "-Lipschitz-bound\t" ++ show (distance,bound,distance<=bound)
  forM_ [0,1,16,256] $ \n -> do
    let trueAfter = S.limit (\s -> case A.rounding s of
                       A.RoundDown -> A.precision s >= n
                       A.RoundUp -> True) :: S.StagedWithFun Bool
        falseAfter = S.limit (\s -> case A.rounding s of
                       A.RoundDown -> False
                       A.RoundUp -> A.precision s < n) :: S.StagedWithFun Bool
    putStrLn $ "partial-force\t" ++ show (n,P.force trueAfter,P.force falseAfter)
