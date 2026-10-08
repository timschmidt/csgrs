import Data.Number.IReal
import Test.QuickCheck 

infix 3 =??=

-- We test equalities up to 1e-200 in this test suite
x =??= y = x =?= y `atDecimals` 200

-- Tests that generator uniformNum generates proper real numbers.
propIsRealNum2 (l,u) = forAll (uniformNum (l,u)) $ \x ->
                   propIsRealNum x .&&. x > fromInteger l .&&. x < fromInteger u

-- Tests that generator uniformIval generates proper real intervals.
propIsRealIval2 (l,u) = forAll (uniformIval (l,u)) $ \x -> 
                   propIsRealIval x  .&&. mid x > fromInteger l .&&. mid x < fromInteger u

-- Tests that generator exprGen generates proper real numbers.
propExprNum ival = forAll (exprGen (uniformNum ival)) propIsRealNum

-- Tests that IReal and Double versions of functions give consistent
-- result (differ in at most two last bits of Double mantissa).
-- Arguments f and g should be "the same function" over the two types.
prop_IRealDouble f g ival = forAll (choose ival) p
   where p x = abs ((w - fromDouble (g (abs x)))/w) < 1/2^(floatDigits x - 2)
           where fromDouble :: Double -> IReal
                 fromDouble x = fromRational (toRational x)
                 w = f (fromDouble (abs x))
     
-- Tests that function argument is the identity function over given interval            
prop_id f ival = forAll (uniformNum ival) $ \x -> f x =??= x

-- Tests that function applications of random numbers in given
-- interval are proper real numbers.
propFunNum f ival = forAll (uniformNum ival) $ \x ->
                    propIsRealNum (f x)

-- Tests that function applications of random intervals in given
-- interval are proper real intervals.
propFunIval f ival = forAll (uniformIval ival) $ \x ->
                     propIsRealIval (f x)

-- Some basic arithmetic properties
propCommutative f x y = f x y =??= f y x
propAssociative f x y z = f x (f y z) =??= f (f x y) z

prop_pyth x = sin x^2 + cos x^2 =??= 1 

prop_double x = sin (2*x) =??= 2 * sin x * cos x

prop_log_add x y = x /= 0 && y /= 0 ==> log (abs (x * y)) =??= log (abs x) + log (abs y)

prop_recip x = x /= 0 ==> x * recip x =??= 1

qc (str,test) = do putStr str
                   quickCheck test

main = do 
  putStrLn "Test the generator of IReal numbers, i.e. that generated values are"
  putStrLn "proper numbers (satisfy the Cauchy criterion) in the intended range."
  mapM_ qc [("    range is (0,10)             ", propIsRealNum2 (0,10)),
            ("    range is (0,1000)           ", propIsRealNum2 (0,1000)),
            ("    range is (0,10^6)           ", propIsRealNum2 (0,10^3)),
            ("    range is (0,10^10)          ", propIsRealNum2 (0,10^6))]
  putStrLn "Test the generator of IReal intervals, i.e. that generated values are"
  putStrLn "proper intervals (the end points are proper numbers and left end point"
  putStrLn "smaller than right end point) and have midpoint in the intended range."
  mapM_ qc [("    range is (0,10)             ", propIsRealIval2 (0,10)),
            ("    range is (0,1000)           ", propIsRealIval2 (0,1000)),
            ("    range is (0,10^6)           ", propIsRealIval2 (0,10^3)),
            ("    range is (0,10^10)          ", propIsRealIval2 (0,10^6))]
  putStrLn "Test that function values are proper real numbers, i.e. function"
  putStrLn "values for random arguments satisfy Cauchy criterion."
  mapM_ qc [("   exp over (0,100)             ", propFunNum exp (0,100)),
            ("   sin over (0,100)             ", propFunNum sin (0,100)), 
            ("   log over (0,100)             ", propFunNum log (0,100)), 
            ("   atan over (0,100)            ", propFunNum atan (0,100))] 
  putStrLn "Test that function values for interval arguments are proper intervals,"
  putStrLn "i.e. the end points are proper numbers and left end point smaller"
  putStrLn "than right end point."
  mapM_ qc [("   exp over (0,10)              ", propFunIval exp (0,10)), 
            ("   sin over (0,10)              ", propFunIval sin (0,10)), 
            ("   log . (+1) . abs over (1,10) ", propFunIval (log . (+1) . abs) (1,10)), 
            ("   atan over (0,10)             ", propFunIval atan (0,10))]
  putStrLn "Test that randomly generated expression values are proper real numbers."
  mapM_ qc [("   range is (0,1)               ", propExprNum (0,1)),
            ("   range is (0,5)               ", propExprNum (0,5))]
  putStrLn "Test that functions in Floating give consistent results in"
  putStrLn "instances for IReal and Double, i.e. differ in at most two"
  putStrLn "last bits of Double mantissa."
  mapM_ qc [("   log over (1,10^6)            ", prop_IRealDouble log log (1,10^6)), 
            ("   sin over (0,pi)              ", prop_IRealDouble sin sin (0,pi)),
            ("   exp over (0,100)             ", prop_IRealDouble exp exp (0,100)),
            ("   atan over (0,10^6)           ", prop_IRealDouble atan atan (0,10^6)),
            ("   sqrt over (0,10^6)           ", prop_IRealDouble sqrt sqrt (0,10^6))]
  qc ("Test commutativity of addition  ", propCommutative (+))
  qc ("Test commutativity of multiplication", propCommutative (*))
  qc ("Test associativity of addition  ", propAssociative (+))
  qc ("Test associativity of multiplication", propAssociative (*))
  putStrLn "Test that the following functions are the identity function."
  putStrLn "Use random inputs, uniformly distributed over given intervals."
  putStrLn "True equality cannot be tested; test passes if |f(x) - x| < 1e-200."
  mapM_ qc [("   exp . log over (0,10)        ", prop_id (exp . log) (0,10)), 
            ("   exp . log over (0,1000)      ", prop_id (exp . log) (0,1000)),
            ("   exp . log over (0,10^10)     ", prop_id (exp . log) (0,10^10)), 
            ("   sq . sqrt over (0,10)        ", prop_id (sq . sqrt) (0,10)), 
            ("   sq . sqrt over (0,1000)      ", prop_id (sq . sqrt) (0,1000)),
            ("   sq . sqrt over (0,10^10)     ", prop_id (sq . sqrt) (0,10^10)),
            ("   tan . atan over (0,10)       ", prop_id (tan . atan) (0,10)), 
            ("   tan . atan over (0,1000)     ", prop_id (tan . atan) (0,1000)), 
            ("   tan . atan over (0,10^10)    ", prop_id (tan .atan) (0,10^10)), 
            ("   asin . sin over (-1,1)       ", prop_id (asin . sin) (-1,1)), 
            ("   asinh . sinh over (0,100)    ", prop_id (asinh . sinh) (0,100)), 
            ("   recip . recip over (0,1000)  ", prop_id (recip . recip) (1,1000))]
  putStrLn "Test pi, by testing the following identities up to 1e-200."
  mapM_ qc [("   sin pi = 0                   ", sin pi =??= 0),
            ("   cos pi = -1                  ", cos pi =??= -1),
            ("   atan 1 = pi/4                ", atan 1 =??= pi/4)]
  putStrLn "Test the following identities up to 1e-200, for random real numbers."
  mapM_ qc [("    cos x^2 + sin x^2 = 1       ", prop_pyth), 
            ("    sin (2*x) = 2*sin x*cos x   ", prop_double)]
  putStrLn "Test the following identities up to 1e-200, for random non-zero numbers."
  mapM_ qc [("    log |x*y| = log |x|+log |y| ", prop_log_add)]
  mapM_ qc [("    x * recip x = 1             ", prop_recip)] 
