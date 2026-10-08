{-# LANGUAGE BangPatterns #-}

-- |
module Main where

import Data.Approximate.ApproximateField
import Data.Time.Clock
import Numeric

-- Testing MPFR module
import Data.Approximate.Floating.MPFR
import Data.Approximate.MPFRLowLevel as MPLL hiding (max)

-- Testing HMPFR module
--import Data.Approximate.Floating.HMPFR
--import Data.Number.MPFR as MPLL hiding (max)

-- Testing Dyadic module
--import Data.Approximate.Floating.Dyadic

run :: (() -> a) -> Int -> Int
run f 0 = 0
run f n =
      let !z = f () in run f (n-1)

runAndMeasure :: UTCTime -> NominalDiffTime -> IO Int -> (() -> t) -> IO Int
runAndMeasure startTime timeLimit n f =
    do n2 <- n
       let !r = run f n2
       end <- getCurrentTime
       if diffUTCTime end startTime < timeLimit
           then do
                  n3 <- runAndMeasure startTime timeLimit (return $ 2 * n2) f
                  return $ n2 + n3
           else return n2

showTime start end n =
    showFFloat Nothing (fromRational . toRational $ diffUTCTime end start * 1000 / fromInteger ( toInteger n )) "ms"

measure f s =
  do
    start  <- getCurrentTime
    !ncalls <- runAndMeasure start 1 (return 1) f
    end <- getCurrentTime
    putStrLn $ s ++ " took    " ++ showTime start end ncalls ++
      " (" ++ show ncalls ++ " eval in " ++ showTime start end 1 ++ ")"
    return ()


--{-
appSet s = MPLL.set Down (fromInteger (max 2 (toInteger (s))))
appSqrt s = MPLL.sqrt Down (fromInteger (max 2 (toInteger (precision s))))
appExp s = MPLL.exp Down (fromInteger (max 2 (toInteger (precision s))))
appLog s = MPLL.log Down (fromInteger (max 2 (toInteger (precision s))))
appSin s = MPLL.sin Down (fromInteger (max 2 (toInteger (precision s))))
appCos s = MPLL.cos Down (fromInteger (max 2 (toInteger (precision s))))
appArcCos s = MPLL.acos Down (fromInteger (max 2 (toInteger (precision s))))
appArcTan s = MPLL.atan Down (fromInteger (max 2 (toInteger (precision s))))
---}

{-
C benchmarks give us:

x*y        took 0.002945 ms (524287 eval in 1544 ms)
x*x        took 0.001804 ms (1048575 eval in 1892 ms)
x/y        took 0.004776 ms (262143 eval in 1252 ms)
sqrt(x)    took 0.004776 ms (262143 eval in 1252 ms)
exp(x)     took 0.172873 ms (8191 eval in 1416 ms)
log(x)     took 0.164571 ms (8191 eval in 1348 ms)
sin(x)     took 0.175314 ms (8191 eval in 1436 ms)
cos(x)     took 0.164571 ms (8191 eval in 1348 ms)
arccos(x)  took 0.376068 ms (4095 eval in 1540 ms)
arctan(x)  took 0.357509 ms (4095 eval in 1464 ms)

Haskell benchmarks, results from Rounded and HMPFR are comparable:
[precision is 3322 bits]
x=x       took    0.002036874841ms (524287 eval in 1067.907ms)
x+y       took    0.002923126074ms (524287 eval in 1532.557ms)
x*y       took    0.005483861861ms (262143 eval in 1437.556ms)
x*x       took    0.005483705458ms (262143 eval in 1437.515ms)
x/y       took    0.006924937915ms (262143 eval in 1815.324ms)
sqrt(x)   took    0.006131920364ms (262143 eval in 1607.44ms)
exp(x)    took    0.170452569893ms (8191 eval in 1396.177ms)
log(x)    took    0.161739103894ms (8191 eval in 1324.805ms)
sin(x)    took    0.154188743743ms (8191 eval in 1262.96ms)
cos(x)    took    0.146092418508ms (8191 eval in 1196.643ms)
arccos(x) took    0.315081807081ms (4095 eval in 1290.26ms)
arctan(x) took    0.317491330891ms (4095 eval in 1300.127ms)

Haskell benchmarks of Dyadics implementation:
x+y       took    0.142887559516ms (8191 eval in 1170.392ms)
x*y       took    0.193928335978ms (8191 eval in 1588.467ms)
x*x       took    0.193587718227ms (8191 eval in 1585.677ms)
x/y       took    0.001369591111ms (1048575 eval in 1436.119ms)  <-- TODO a bug??
-}

main = do
    let n = 1000
    let precbits = truncate $ n * Prelude.log (10.0) / Prelude.log (2.0) + 1.0
    putStrLn $ "[precision is " ++ show precbits ++ " bits]"
    let s = precDown precbits
    let !x = appSub s (appSqrt s (appFromInteger 3)) (appFromInteger 1)
    let !y = appSqrt s (appFromInteger 5)
    --let !z = appLog s x
    {- Following ugly blocks of numbers let's us test dyadic module -}
    {-    let !x = Dyadic { mant = 7320508075688772935274463415058723669428052538103806280558069794519330169088000370811461867572485756756261414154067030299699450949989524788116555120943736485280932319023055820679748201010846749232650153123432669033228866506722546689218379712270471316603678615880190499865373798593894676503475065760507566183481296061009476021871903250831458295239598329977898245082887144638329173472241639845878553976679580638183536661108431737808943783161020883055249016700235207111442886959909563657970871684980728994932964842830207864086039887386975375823173178313959929830078387028770539133695633121037072640192491067682311992883756411414220167427521023729942708310598984594759876642888977961478379583902288548529035760338528080643819723446610596897228728652641538226646984200211954841552784411812865345070351916500166892944154808460712771439997629268346295774383618951101271486387469765459824517885509753790138806649619119622229571105552429237231921977382625616314688420328537166829386496119170497388363954959381, expo = -3000}
    let !y = Dyadic { mant = 2236067977499789696409173668731276235440618359611525724270897245410520925637804899414414408378782274969508176150773783504253267724447073863586360121533452708866778173191879165811276645322639856580535761350417533785003423392414064442086432539097252592627228876299517402440681611775908909498492371390729728898482088641542689894099131693577019748678884425089754132956183176921499977424801530434115035957668332512498815178139408000562420855243542235556106306342820234093331982933959746352271201341749614202635904737885504389687061135660045757139956595566956917564578221952500060539231234005009286764875529722056766253666074485853505262330678494633422242317637277026632407680104443315825733505893098136226343198686471946989970180818952426445962034522141192232912598196325811104170495807048120403455994943506855551855572512388641655010262436312571024449618789424682903404474716115455723201737676590460918529575603577984398054155380779064393639723028756062999482213852177348592453515121046345555040707227872, expo = -3000}
    -}
    --{-
    --measure (\() -> appSet s x)   "x=x      "
    ---
    measure (\() -> appAdd s x y) "x+y      "
    measure (\() -> appMul s x y) "x*y      "
    measure (\() -> appMul s x x) "x*x      "
    measure (\() -> appDiv s x y) "x/y      "
    --{-
    measure (\() -> appSqrt s x)  "sqrt(x)  "
    measure (\() -> appExp s x)   "exp(x)   "
    measure (\() -> appLog s x)   "log(x)   "
    measure (\() -> appSin s x)    "sin(x)   "
    measure (\() -> appCos s x)    "cos(x)   "
    measure (\() -> appArcCos s x) "arccos(x)"
    measure (\() -> appArcTan s x) "arctan(x)"
    ---}
