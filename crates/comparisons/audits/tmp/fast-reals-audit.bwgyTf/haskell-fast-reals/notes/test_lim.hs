import Reals

------------
-- Functions
------------

errAltSeq s = zipWith (\x y -> abs(x-y)) s (drop 1 s)

lima seq = lim seq (errAltSeq seq)


-- SQRT - Babylonian method: sqrt(x) = lim a_n,  a_(n+1) = (a_n + x/a_n) / 2
sqrt_seq x
    | x < 0            = error "not enough Imagination"
    | x == 0 || x == 1 = repeat x
    | x > 1            = seq x
    | x < 1            = seq 1
    where seq y = y : seq ((y + x/y) / 2)

-- To get the errors we alternate between upper and lower approximations
sqrt_aseq x = zip2 (sqrt_seq x) $ (map recip . sqrt_seq . recip) x
    where zip2 [] _ = []
          zip2 _ [] = []
          zip2 (a:as) (b:bs) = a : b : (zip2 as bs)

sqrt_err x = errAltSeq $ sqrt_aseq x

limsqrt x = lima $ sqrt_aseq x


-- ARCTAN - Taylor series

atan_seq x = let x2 = x*x
                 nom = x : map (* x2) nom
                 denom = zipWith (*) (cycle [1,-1]) $ map ((+1).(*2)) [0..]
             in scanl1 (+) $ zipWith (/) nom $ map fromInteger denom

limatan = lima . atan_seq


------------
-- Constants
------------

-- e --

intfac = 1 : 1 : (zipWith (*) [2..] $ drop 1 intfac)

--realfac :: [RealNum Dyadic]
realfac = map fromInteger $ intfac

-- e_seq = ( Sum_{k=0}^{k=n} 1/k! )_n
e_seq = scanl1 (+) $ map recip realfac

-- errors are obtained by approximating the infinite series from n on
-- with a geometric series (and then by substituting (n+2)/(n+1) with 2)
e_err = map ((* 2).recip) $ drop 1 realfac

e = lim e_seq e_err

-- pi --

pi_denom = zipWith (*) (cycle [1, -1]) $ map (\n -> n * (n-1) * (2*n - 1)) [2..]

pi_seq = scanl1 (+) $ map (recip.fromInteger) pi_denom

pi_madhava = (exact 3) + lima pi_seq


pi_leibniz = 4 * limatan 1


pi_machin = 16 * limatan 0.2 - 4 * limatan (1/239)


---------------
-- Old versions
---------------

-- lim0 :: (Ord q, Num q) => [RealNum q] -> [RealNum q] -> RealNum q
-- lim0 seq err = limit (\s ->
--     let r = rounding s
--         k = precision s
--         approximations = map (\x -> approximate x (prec_down k)) $ take (k+1) seq
--         errors = map (upper. \x -> approximate x (prec_down k)) $ take (k+1) err
--         low = maximum $ zipWith (-) (map lower approximations) errors
--         up = minimum $ zipWith (+) (map upper approximations) errors
--     in if low > up then error "not convergent"
--                    else if r == RoundDown then Interval {lower=low, upper=up}
--                                           else Interval {lower=up, upper=low})

-- lim1 :: [RealNum Dyadic] -> [RealNum Dyadic] -> RealNum Dyadic
-- lim1 seq err = 
--     let approximations = map (\(x,k) -> iapprox_to x k) $ zip seq [0..]
--         errors = map (upper. \(x,k) -> iapprox_to x k) $ zip err [0..]
--         lows = scanl1 max $ zipWith (-) (map lower approximations) errors
--         ups = scanl1 min $ zipWith (+) (map upper approximations) errors
--     in limit (\s -> 
--            let k = precision s
--                l = lows !! k
--                u = ups !! k
--            in case rounding s of
--                RoundDown -> Interval {lower=l, upper=u}
--                RoundUp   -> Interval {lower=u, upper=l})
