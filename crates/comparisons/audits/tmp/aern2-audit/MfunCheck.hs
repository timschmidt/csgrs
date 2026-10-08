{-# LANGUAGE NoRebindableSyntax #-}
module Main where
import Prelude
import Control.Monad (forM_)
import Control.Exception (evaluate)
import Control.DeepSeq (force)
import Data.Ratio ((%))
import qualified MixedTypesNumPrelude as M
import qualified Numeric.CollectErrors as CN
import qualified Control.CollectErrors as CE
import qualified AERN2.MP as B
import qualified AERN2.AD.Differential as D
import qualified AERN2.Linear.Matrix.Type as A
import qualified AERN2.Linear.Matrix.Inverse as I
import qualified AERN2.Linear.Vector.Type as V
import qualified AERN2.BoxFun.Type as F
import qualified AERN2.BoxFun.TestFunctions as T
import System.IO (hSetBuffering, stdout, BufferMode(LineBuffering))
import System.Timeout (timeout)
import System.Environment (getArgs)
import System.Exit (exitSuccess)
import Control.Monad (when)

type Jet = D.Differential (M.CN B.MPBall)
type Coeffs = [Rational]

has :: M.CN B.MPBall -> Rational -> Bool
has cb x = not (CN.hasError cb) && maybe False inside (CE.getMaybeValue cb)
  where inside b = let c = M.rational (B.ball_value b); e = M.rational (B.ball_error b)
                  in c-e <= x && x <= c+e

ball :: Integer -> Rational -> M.CN B.MPBall
ball p = M.cn . B.mpBallP (B.prec p)

jet :: Integer -> Coeffs -> Jet
jet p [x,a,b,c] = D.OrderTwo (ball p x) (ball p a) (ball p b) (ball p c)
jet _ _ = error "internal audit coefficient count"

hasJet :: Jet -> Coeffs -> Bool
hasJet (D.OrderTwo x a b c) exact = and (zipWith has [x,a,b,c] exact)
hasJet _ _ = False

-- Independent polynomial quotient in Q[s,t]/(s^2,t^2), solving b*c=a
-- coefficient by coefficient. Multiplication is a generic convolution.
indices :: [(Int,Int)]
indices = [(0,0),(1,0),(0,1),(1,1)]
times :: Coeffs -> Coeffs -> Coeffs
times a b = [sum [x*y | (ij,x) <- zip indices a, (kl,y) <- zip indices b,
                       (fst ij+fst kl,snd ij+snd kl) == target] | target <- indices]
quotient :: Coeffs -> Coeffs -> Coeffs
quotient a b = foldl step [] [0..3]
  where step prior k = prior ++ [(a!!k - sum
            [b!!i * prior!!j | i <- [1..3], j <- [0..k-1],
             let (x,y) = indices!!i; (u,v) = indices!!j,
             (x+u,y+v) == indices!!k]) / head b]

power :: Coeffs -> Integer -> Coeffs
power a n | n < 0 = quotient [1,0,0,0] (power a (-n))
          | otherwise = iterate (`times` a) [1,0,0,0] !! fromInteger n

report :: String -> [Bool] -> IO ()
report label tests = putStrLn (label ++ ": " ++ show (length (filter id tests)) ++ "/" ++ show (length tests))

main :: IO ()
main = do
  hSetBuffering stdout LineBuffering
  args <- getArgs
  when (not (null args)) $ do
    let [sp,si] = args
    putStrLn (inverseClassify (read sp) (matrices !! read si))
    exitSuccess
  forM_ [24,53,100] $ \p -> do
    let coeff s = [toInteger (s+3)%7, toInteger (s-5)%11,
                   toInteger (s*2-7)%13, toInteger (s+1)%17]
        cases = [(coeff s, coeff (s+9)) | s <- [0..24]]
    report ("jets ring+division p=" ++ show p)
      [hasJet (op (jet p a) (jet p b)) (oracle a b)
       | (a,b) <- cases, (op,oracle) <- [(M.add,zipWith (+)),(M.sub,zipWith (-)),
                                       (M.mul,times),(M.divide,quotient)]]
    report ("jets integer powers p=" ++ show p)
      [hasJet (M.pow (jet p a) n) (power a n) | (a,_) <- cases, n <- [-3..8]]
    putStrLn ("jets polynomial powers at zero p=" ++ show p ++ " " ++ show
      [(n,hasJet (M.pow (jet p [0,1,2,3]) n) (power [0,1,2,3] n)) | n <- [0..5]])
    report ("elementary jets exact special values p=" ++ show p)
      [hasJet actual expected | s <- [1..12],
       let a = toInteger s%7; b = toInteger (s+1)%9; c = toInteger (s-3)%11,
       (actual,expected) <-
         [(M.exp (jet p [0,a,b,c]), [1,a,b,c+a*b]),
          (M.sin (jet p [0,a,b,c]), [0,a,b,c]),
          (M.cos (jet p [0,a,b,c]), [1,0,0,-a*b]),
          (M.sqrt (jet p [4,a,b,c]), [2,a/4,b/4,c/4-a*b/32])]]
    report ("Rosenbrock value/gradient/Hessian p=" ++ show p)
      [and (zipWith has actual expected) | s <- [0..24],
       let x = toInteger (s-10)%7; y = toInteger (s*3-20)%11,
       let (v,g,h) = F.valueGradientHessian T.rosenbrock (V.fromList [ball p x,ball p y]),
       let actual = [v] ++ V.toList g ++ V.toList (A.entries h),
       let expected = [(1-x)^2+100*(y-x*x)^2, 2*(x-1)-400*x*(y-x*x),200*(y-x*x),
                       2-400*y+1200*x*x,-400*x,-400*x,200]]
  let v = V.fromList (map (ball 53) [-3,-2])
      mixed = V.fromList [ball 53 0, M.cn (B.updateRadius (const (B.errorBound (1%4 :: Rational))) (B.mpBall (1 :: Integer)))]
  putStrLn ("vector norm encloses exact max-absolute 3: " ++ show (has (V.inftyNorm v) 3))
  putStrLn ("mixed vector reported accuracy: " ++ show (B.getAccuracy mixed) ++
            "; entries: " ++ show (map B.getAccuracy (V.toList mixed)))

-- Valid nonsingular matrices; each inverse is qualified separately so a
-- missing result cannot prevent unrelated derivative checks from completing.
matrices :: [[[Rational]]]
matrices = [[[d,0],[0,e]] | d <- [-4,-2,-1,1,2,4], e <- [-3,-1,1,3]] ++
           [[[4,t],[u,3]] | t <- [-1,0,1], u <- [-1,0,1]]
inverseClassify :: Integer -> [[Rational]] -> String
inverseClassify p [[a,b],[c,d]] =
  let det = a*d-b*c; expected = [d/det,-b/det,-c/det,a/det]
      input = A.Matrix 2 (V.fromList (map (ball p) [a,b,c,d]))
  in case I.inverse input of
    Nothing -> "no certificate"
    Just out | CN.hasError out -> "returned error-bearing matrix"
             | and (zipWith has (V.toList (A.entries out)) expected) -> "valid certificate"
             | otherwise -> "incorrect enclosure"
inverseClassify _ _ = error "internal audit matrix dimensions"
