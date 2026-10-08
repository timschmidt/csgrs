module Main where

import Prelude
import Control.Monad (forM_, unless)
import Data.Ratio ((%), numerator, denominator)
import qualified MixedTypesNumPrelude as M
import qualified Numeric.CollectErrors as CN
import qualified Control.CollectErrors as CE
import qualified AERN2.Real as R
import qualified AERN2.MP as B
import qualified AERN2.MP.Float as F
import System.Exit (exitFailure)
import System.Environment (getArgs)

-- All containment checks use exact Prelude Rational arithmetic, independent
-- of AERN2's contains/getAccuracy predicates and outward endpoint routines.
bounds :: B.MPBall -> (Rational, Rational)
bounds b = (c-e,c+e)
  where
    c = M.rational (B.ball_value b)
    e = M.rational (B.ball_error b)

encloses :: B.MPBall -> Rational -> Bool
encloses b x = l <= x && x <= u
  where (l,u) = bounds b

acceptable :: CN.CN B.MPBall -> Rational -> Bool
acceptable b x = not (CN.hasError b) && maybe False (`encloses` x) (CE.getMaybeValue b)

checkGroup :: String -> [(String,Bool)] -> IO ()
checkGroup name checks = do
  let bad = [label | (label,False) <- checks]
  putStrLn (name ++ ": " ++ show (length checks - length bad) ++ "/" ++ show (length checks))
  forM_ (take 5 bad) (putStrLn . ("  failed: " ++))
  unless (null bad) exitFailure

pow2 :: Int -> Rational
pow2 n | n >= 0 = fromInteger (2^n)
       | otherwise = 1 % (2^(-n))

samples :: [Rational]
samples = [n % d | n <- [-37,-3,-1,0,1,3,37], d <- [1,3,7,19]]
       ++ [s * (pow2 k + 1%3) | s <- [-1,1], k <- [-100,-10,10,100]]

precisions :: [Integer]
precisions = [10,24,53,100,200]

numerical :: IO ()
numerical = do
  checkGroup "rational ball imports"
    [(show (p,x), encloses (B.mpBallP (B.prec p) x) x) | p <- precisions, x <- samples]
  forM_ [("add",(+),(+)),("sub",(-),(-)),("mul",(*),(*))] $ \(name,ballOp,ratOp) ->
    checkGroup ("rational ball " ++ name)
      [(show (p,x,y), encloses (ballOp (B.mpBallP (B.prec p) x) (B.mpBallP (B.prec p) y)) (ratOp x y))
       | p <- precisions, x <- samples, y <- samples]
  checkGroup "rational ball division"
    [(show (p,x,y), encloses (a/b) (x/y))
     | p <- precisions, x <- samples, y <- samples, y /= 0,
       let a = B.mpBallP (B.prec p) x, let b = B.mpBallP (B.prec p) y,
       let (l,u) = bounds b, l > 0 || u < 0]
  checkGroup "ball lowering precision keeps original interval"
    [(show (p,x), let (l,u) = bounds original
                 in encloses reduced l && encloses reduced u)
     | p <- [10,24,53], x <- samples,
       let original = B.mpBallP (B.prec (200 :: Integer)) x,
       let reduced = B.setPrecision (B.prec (p :: Integer)) original]
  checkGroup "sqrt exact rational enclosure"
    [(show (p,x), u >= 0 && u*u >= x && (l <= 0 || l*l <= x))
     | p <- precisions, x <- samples, x > 0,
       let (l,u) = bounds (sqrt (B.mpBallP (B.prec p) x))]
  checkGroup "CReal rational algebra with error status"
    [(show (p,x,y), acceptable ((a*a-b*b) R.? B.bits p) (x*x-y*y))
     | p <- [10,53,150 :: Integer], x <- samples, y <- take 12 samples,
       let a = R.creal x, let b = R.creal y]
  let directed = [("add",(F.+.),(F.+^),(+)), ("sub",(F.-.),(F.-^),(-)),
                  ("mul",(F.*.),(F.*^),(*)), ("div",(F./.),(F./^),(/))]
  forM_ directed $ \(name,down,up,exact) -> checkGroup ("directed MPFloat " ++ name)
    [(show (p,x,y), M.rational (down a b) <= target && target <= M.rational (up a b))
     | p <- precisions, x <- samples, y <- samples, name /= "div" || y /= 0,
       let a = F.ceduCentre (F.fromRationalCEDU (B.prec p) x),
       let b = F.ceduCentre (F.fromRationalCEDU (B.prec p) y),
       let target = exact (M.rational a) (M.rational b)]
  putStrLn "Diagnostic accuracy metadata (rough Accuracy is not a certified radius bound):"
  forM_ ([1%4,3%4,1,3%2,3,7] :: [Rational]) $ \radius -> do
    let eb = B.errorBound radius
        ac = B.getAccuracy eb
        strict = M.rational eb <= pow2 (negate (fromInteger (B.fromAccuracy ac)))
    putStrLn (show (radius,ac,strict))
  let exactBall = B.mpBallP (B.prec (100 :: Integer)) (3%2 :: Rational)
      coarseBall = B.MPBall (F.mpFloat (0 :: Integer)) (B.errorBound (1 :: Integer))
  putStrLn ("getApproximate accuracy flags, exact and coarse at bits 20: " ++
    show (snd (B.getApproximate (B.bits (20 :: Integer)) exactBall),
          snd (B.getApproximate (B.bits (20 :: Integer)) coarseBall)))
  let x = R.pi - R.pi + R.creal (1 % (10^30) :: Rational)
      branch = M.ifThenElse (x M.> (0 :: Integer)) (R.creal (1 :: Integer)) (R.creal (0 :: Integer))
      continuous = M.ifThenElse (x M.> (0 :: Integer)) x (-x)
  forM_ [5,20,100 :: Integer] $ \p -> do
    putStrLn ("nonzero partial branch bits " ++ show p ++ ": " ++ show (branch R.? B.bits p))
    putStrLn ("continuous abs branch bits " ++ show p ++ ": " ++ show (continuous R.? B.bits p))

-- Exact endpoints exported for a separate directed-MPFR oracle. Arithmetic
-- and enclosure extraction here do not call the oracle or compare doubles.
exportElementary :: IO ()
exportElementary = forM_ precisions $ \p -> do
  forM_ [("sqrt",sqrt),("exp",exp),("log",log),("sin",sin),("cos",cos)] $ \(name,op) ->
    forM_ ([n%d | n <- [-100,-7,-1,0,1,7,100], d <- [1,3,19]] :: [Rational]) $ \x ->
      unless ((name == "sqrt" && x < 0) || (name == "log" && x <= 0)) $ do
        let (l,u) = bounds (op (B.mpBallP (B.prec p) x))
        putStrLn (unwords [name, show p, show (numerator x),show (denominator x),
                          show (numerator l),show (denominator l),show (numerator u),show (denominator u)])

main :: IO ()
main = do
  args <- getArgs
  case args of
    ["export"] -> exportElementary
    _ -> numerical
