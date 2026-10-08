module Demo where

import qualified Data.Number.Real as R

import Control.Monad

import System.IO

harmonic   :: Int -> R.CReal
harmonic n = go n 1
             where go 1 acc = acc
                   go i acc = go (pred i) (acc + (recip . R.fromInt) i)

alternating   :: Int -> R.CReal
alternating n = go n 1
                where go 1 acc             = acc
                      go i acc | odd i     = go (pred i) (acc + (recip . R.fromInt) i)
                               | otherwise = go (pred i) (acc - (recip . R.fromInt) i)

-- pi using Chudnovsky algorithm
pichud :: R.CReal
pichud = recip (R.infSumRec 13591409 pi') * R.sqrt (640320 ^ (3 :: Int) :: R.CReal) / 12
       where pi' :: R.CReal -> R.Nat -> (R.CReal, R.CReal)
             pi' r k = (negate r * x1 * x2 / (x3 * x4 * x5 * x6), 10 ^^ ((- fromIntegral n + 1) * 14 :: Int))
                 where x1 = fromIntegral (product [6*n-5..6*n])
                       x2 = fromIntegral (13591409 + 545140134 * n)
                       x6 = fromIntegral (13591409 + 545140134 * pred n)
                       x3 = fromIntegral (product [3*n-2..3*n])
                       x4 = fromIntegral (n * n * n)
                       x5 = fromIntegral (640320 * 640320 * 640320 :: Integer)
                       n = toInteger k

-- pi using borwein's quartic convergence algorithm
-- this is actually slower than gauss
piborw :: R.Nat -> R.CReal
piborw p = recip (go (6 - 4 * R.sqrt 2) (R.sqrt 2 - 1) 0)
    where go :: R.CReal -> R.CReal -> R.Nat -> R.CReal
          go a y n | n == p'   = a
                   | otherwise = go ak yk (succ n)
                   where yk = (1 - y4) / (1 + y4)
                         ak = a * (1 + yk) ^ (4 :: Int) - 2 ^ (2 * n + 3) * yk * (1 + yk + yk ^ (2 :: Int))
                         y4 = R.sqrt (R.sqrt (1 - y ^ (4 :: Int)))
          p' = ceiling (logBase 4 (fromIntegral p) :: Double)

-- gauss-lagendre (quadratic)
pigl :: R.Nat -> R.CReal
pigl pr = go 0 1 (1 / R.sqrt 2) (1 / 4) 1 
    where go                       :: R.Nat -> R.CReal -> R.CReal -> R.CReal -> R.CReal -> R.CReal
          go n a b t p | n == k    = let m = a + b in m * m / (4 * t)
                       | otherwise = go (succ n) an bn tn pn
                       where an = (a + b) / 2
                             bn = R.sqrt (a * b)
                             tn = t - p * (a - an) ^ (2 :: Int)
                             pn = 2 * p
          k = ceiling (logBase 2 (fromIntegral pr) :: Double)

e :: R.CReal
e = R.infSumRec 1 er
    where er     :: R.CReal -> R.Nat -> (R.CReal, R.CReal)
          er r n = let r' = r / fromIntegral n in (r', 3 * r')

wrap :: R.Nat -> Double
wrap n = 6 * 2 ^ n * pidouble n

wrap1 :: R.Nat -> R.CReal
wrap1 n = 6 * 2 ^ n * pireal n

pidouble :: R.Nat -> Double
pidouble 0 = 1 / sqrt 3
pidouble n = (sqrt (n1 * n1 + 1) - 1) / n1
    where n1 = pidouble (pred n)

pireal :: R.Nat -> R.CReal
pireal 0 = 1 / R.sqrt 3
pireal n = (R.sqrt (n1 * n1 + 1) - 1) / n1
    where n1 = pireal (pred n)


limr :: R.CReal
limr = R.limRec 1 f
       where f r n = (R.sqrt (3 + r), 1 / 2 ^ n)

limp :: R.CReal
limp = R.lim an rn
       where an n = 1 + 1 / 2 ^ n
             rn n = 1 / 2 ^ n

-- erdos-borwein constant
eb :: R.CReal
eb = R.infSum a r
     where a n = 1 / (2 ^ (n + 1) - 1)
           r n = 1 / (2 ^ n)

-----------------------------------
usage :: String
usage = unlines
        ["Usage: ",
         "pigl n           (pi to n decimals using Gauss-Lagendre algorithm)",
         "piborw n         (pi to n decimals using Borwein's (quartic) algorithm)", 
         "pichud n         (pi to n decimals using Chudnovsky's algorithm)",
         "e n              (e to n decimals)", 
         "erdosBorw n      (Erdos-Borwein's constant to n decimals)", 
         "limr n           (limit of a recursive sequence a(n+1) = sqrt (3 + a(n)))", 
         "limp n           (limit of a sequence 1 + 2^(-n))", 
         "harmonic p n     (sum of first n elements of harmonic series to p decimals)",
         "alternating p n  (same as harmonic only for alternating harmonic series)", 
   --      "compareDoubleReal n p (compare accuracy of Double and Real",
         "",
         "usage            (print this information)",
         "quit"            ]

run2     :: String -> R.Nat -> String
run2 ex p = case ex of
            "pigl" -> R.toStringDec p (pigl p)
            "piborw" -> R.toStringDec p (piborw p)
            "pichud" -> R.toStringDec p pichud
            "e" -> R.toStringDec p e
            "erdosBorw" -> R.toStringDec p eb
            "limr" -> R.toStringDec p limr
            "limp" -> R.toStringDec p limp
            _ -> usage
run3       :: String -> R.Nat -> Int -> String
run3 ex p n = case ex of
            "harmonic" -> R.toStringDec p (harmonic n)
            "alternating" -> R.toStringDec p (alternating n)
            "compareDoubleReal" -> unlines (("Real " ++ replicate (fromIntegral p) ' ' ++ "|    Double") : 
                                            map (\k -> R.toStringDec p (wrap1 k)
                                                 ++ "    |    " ++ show (wrap k)) [1..fromIntegral n])
            _ -> usage

loop :: IO ()
loop = do putStr ">> "
          hFlush stdout
          args <- fmap words getLine
          if head args /= "quit" then do
              case args of 
                (ex : p : [])    -> do catch (readIO p >>= return . run2 ex  >>= putStrLn)
                                                 (\_ -> putStrLn $ "Not an integer: " ++ p)
                (ex : p : n : _) -> catch ( readIO p >>= \p' -> 
                                                (readIO n >>= \n' -> return (p',n'))
                                                >>= return . uncurry (run3 ex) >>= putStrLn) 
                                    (\_ -> putStrLn $ "Not integers: " ++ p ++ " " ++ n)
                _                -> putStrLn usage
              loop
            else print "Goodbye!"


main :: IO ()
main = do putStrLn usage
	  loop