{-# LANGUAGE NoRebindableSyntax #-}
module Main where
import Prelude
import Data.Ratio ((%))
import qualified MixedTypesNumPrelude as M
import qualified AERN2.MP as B
import qualified AERN2.MP.Ball as Ball
import qualified AERN2.MP.Dyadic as Dy

bounds :: B.MPBall -> (Rational,Rational)
bounds b = (c-e,c+e)
  where c = M.rational (B.ball_value b); e = M.rational (B.ball_error b)

-- Isolated arithmetic-expression qualification, NOT a build/pass of the
-- obsolete univariate package. Formula copied structurally from Power/Eval
-- evalLip; the tested polynomial is f(x)=x and its exact Lipschitz bound is 1.
main :: IO ()
main = do
  let one = B.mpBall (1 :: Integer)
      half = B.mpBallP (B.prec 53) (1%2 :: Rational)
      unit = Ball.hullMPBall (B.mpBall (-1 :: Integer)) one
      sourceErr = M.mul (M.mul one (Dy.dyadic (B.ball_error unit))) half
      sourceRange = M.add (B.centreAsBall unit) (Ball.hullMPBall (M.negate sourceErr) sourceErr)
      (l,u) = bounds sourceRange
  putStrLn ("Power.evalLip identity on [-1,1]: bounds=" ++ show (l,u) ++
            ", contains both exact endpoint values=" ++ show (l <= -1 && 1 <= u))
  -- Isolated UnaryBallDFun midpoint remainder, same operation order.
  -- Smooth f(x)=sqrt(x^2+(1/16)^2) has derivative in [-1,1], f(0)=1/16,
  -- and integral over [-1,1] >= integral |x| = 1. This last bound is exact
  -- and independent of any donor interval-integrator or numerical quadrature.
  let deriv = unit
      width = B.mpBall (2 :: Integer)
      variation = M.divide (M.sub deriv deriv) width
      widthHalfSquared = M.pow (M.mul width half) (2 :: Integer)
      remainder = M.mul (M.mul variation widthHalfSquared) half
      midpointArea = M.mul (B.mpBallP (B.prec 53) (1%16 :: Rational)) width
      area = M.add midpointArea remainder
      areaBounds@(_,areaUpper) = bounds area
  putStrLn ("UnaryBallDFun smooth midpoint formula: bounds=" ++ show areaBounds ++
            ", upper below independent integral lower bound 1=" ++ show (areaUpper < 1))
