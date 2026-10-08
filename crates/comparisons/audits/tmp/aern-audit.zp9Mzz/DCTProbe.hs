module Main where
import qualified FnReps.Polynomial.UnaryChebSparse.DCTMultiplication as D
import Numeric.AERN.RealArithmetic.Interval.Double (DI, Interval(..))
import Numeric.AERN.RealArithmetic.Basis.Double ()
import Control.DeepSeq (force)
import Control.Exception (SomeException, evaluate, try)
import Control.Monad (forM, forM_, unless)
import qualified Data.HashMap.Strict as H
import Data.List (foldl')
import Data.Ratio ((%))
import System.Environment (getArgs)
import System.IO (stdout, hSetBuffering, BufferMode(LineBuffering))
import System.Timeout (timeout)

-- Independent exact oracle: expand Chebyshev terms by their three-term
-- recurrence, convolve ordinary monomials, then triangularly eliminate back
-- into the Chebyshev basis. No donor floating operation enters this oracle.
add a b = zipWith (+) (pad n a) (pad n b) where n = max (length a) (length b)
pad n xs = take n (xs ++ repeat 0)
scale a = map (a*)
trim = reverse . dropWhile (==0) . reverse
chebs :: [[Rational]]
chebs = [1] : [0,1] : zipWith (\b a -> trim (add (0:scale 2 b) (scale (-1) a))) (tail chebs) chebs
toMono xs = trim $ foldl' add [] (zipWith scale xs chebs)
multiply a b = trim $ foldl' add [] [replicate i 0 ++ scale c b | (i,c) <- zip [0..] a]
fromMono [] = []
fromMono p = add (replicate degree 0 ++ [c]) (fromMono rest)
  where degree = length p - 1
        t = chebs !! degree
        c = last p / last t
        rest = trim $ add p (scale (-c) t)
oracle a b = fromMono (multiply (toMono a) (toMono b))

contains :: DI -> Rational -> Bool
contains (Interval l r) q = not (isNaN l || isNaN r) && l <= r
  && (isInfinite l && l < 0 || toRational l <= q)
  && (isInfinite r && r > 0 || q <= toRational r)
asInput xs = H.fromList [(i,fromRational c :: DI) | (i,c) <- zip [0..] xs]
coeffs seed n = take (n+1) [toInteger (fromInteger (x `mod` 15) - 7 :: Int) % 8
  | x <- tail $ iterate (\s -> (1103515245*s + 12345) `mod` 2147483648) seed]

checkCase (label,a,b) = do
  let expected = oracle a b
      left = asInput a
      right = asInput b
      results = [("direct", D.multiplyDirect_terms left right), ("DCT", D.multiplyDCT_terms left right)]
  forM results $ \(method,result) -> do
    let end = max (length expected - 1) (maximum (0:H.keys result))
        bad = [(i,q,show v) | (i,q) <- zip [0..end] (pad (end+1) expected),
                    let v = H.lookupDefault 0 i result, not (contains v q)]
        width (Interval l r) = r-l
        maxWidth = maximum (0:map width (H.elems result))
    _ <- evaluate (force bad)
    putStrLn $ show (label,method,"terms",H.size result,"maxWidth",maxWidth,"failures",bad)
    pure (length bad)

checkTransform n = do
  let a = map fromRational (coeffs (toInteger n) n) :: [DI]
      r = D.tDCT_I_reference a
      f = D.tDCT_I_nlogn a
      overlap (Interval l u) (Interval l' u') = max l l' <= min u u'
      misses = [i | (i,(x,y)) <- zip [0::Int ..] (zip r f), not (overlap x y)]
  putStrLn ("DCT-I reference overlap " ++ show (n,misses))
  pure (length misses)

boundary name value = do
  result <- timeout 2000000 (try (evaluate (force value)) :: IO (Either SomeException String))
  putStrLn (name ++ ": " ++ case result of
    Nothing -> "TIMEOUT"
    Just (Left e) -> "EXCEPTION " ++ show e
    Just (Right s) -> s)

main = do
  hSetBuffering stdout LineBuffering
  args <- getArgs
  case args of
    ["empty"] -> do
      boundary "direct empty polynomial" (show (D.multiplyDirect_terms H.empty (H.singleton 0 1)))
      boundary "DCT empty polynomial" (show (D.multiplyDCT_terms H.empty (H.singleton 0 1)))
    ["empty-sdct"] -> boundary "SDCT empty vector" (show (D.tSDCT_III_nlogn []))
    _ -> do
      let degrees = [0,1,2,3,4,7,8,15,16,31,32]
          cases = [(show (d,e,s), coeffs s d, coeffs (s+11) e)
                    | d <- degrees, e <- [0,1,d], s <- [1,71]]
            ++ [("zero",[0],[0]), ("identity",[1],[0,0,1]),
                ("T4 squared",[0,0,0,0,1],[0,0,0,0,1])]
      failures <- concat <$> mapM checkCase cases
      misses <- mapM checkTransform [2,4,8,16,32,64]
      putStrLn ("TOTAL " ++ show (length cases, sum failures, sum misses))
      unless (sum failures == 0 && sum misses == 0) $ error "DCT enclosure oracle failed"
