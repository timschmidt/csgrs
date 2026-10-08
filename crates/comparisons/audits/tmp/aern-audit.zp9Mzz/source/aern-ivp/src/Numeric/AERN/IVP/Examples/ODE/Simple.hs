{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE ScopedTypeVariables #-}
{-|
    Module      :  Numeric.AERN.IVP.Examples.ODE.Simple
    Description :  simple examples of ODE IVPs
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Simple examples of ODE IVPs.
-}

module Numeric.AERN.IVP.Examples.ODE.Simple where

import Numeric.AERN.IVP.Specification.ODE

import Numeric.AERN.RmToRn.Domain
import Numeric.AERN.RmToRn.New

import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut
import Numeric.AERN.RealArithmetic.RefinementOrderRounding (dblToReal, dbldblToReal)
import Numeric.AERN.RealArithmetic.RefinementOrderRounding.Operators

--import qualified Numeric.AERN.RealArithmetic.NumericOrderRounding as ArithUpDn

import Numeric.AERN.RealArithmetic.ExactOps

import qualified Numeric.AERN.RefinementOrder as RefOrd
--import Numeric.AERN.RefinementOrder.Operators

--import qualified Numeric.AERN.NumericOrder as NumOrd
--import Numeric.AERN.NumericOrder.Operators


import qualified Data.Map as Map

ivpByName :: 
    (Var f ~ String,
     HasConstFns f,
     HasProjections f,
     RefOrd.RoundedLattice f,
     Neg f,
     ArithInOut.RoundedRing f,
     ArithInOut.RoundedAbs f,
     ArithInOut.RoundedMixedField f Double,
     ArithInOut.RoundedReal (Domain f),
     RefOrd.IntervalLike (Domain f),
     ArithInOut.RoundedSquareRoot (Domain f),
     Show f, Show (Domain f)
    )
    =>
    String {-^ IVP name - see source code for the list -} -> 
    f {-^ sample function of the type to be used in simulation -} -> 
    Maybe (ODEIVP f)
ivpByName name sampleFn =
    Map.lookup name $ ivpByNameMap sampleFn
    
ivpByNameReportError ::
    (Var f ~ String,
     HasConstFns f,
     HasProjections f,
     RefOrd.RoundedLattice f,
     Neg f,
     ArithInOut.RoundedRing f,
     ArithInOut.RoundedAbs f,
     ArithInOut.RoundedMixedField f Double,
     ArithInOut.RoundedReal (Domain f),
     RefOrd.IntervalLike (Domain f),
     ArithInOut.RoundedSquareRoot (Domain f),
     Show f, Show (Domain f)
    )
    => 
    String -> 
    f -> 
    ODEIVP f
ivpByNameReportError ivpName samplePoly =
    case ivpByName ivpName samplePoly of
        Just ivp -> ivp
        _ -> 
            error $
                "unknown ivp: " ++ ivpName
                ++ "\n known ivps:\n"
                ++ unlines (map ("    " ++) (ivpNames samplePoly))
    
ivpNames :: 
    (Var f ~ String,
     HasConstFns f,
     HasProjections f,
     RefOrd.RoundedLattice f,
     Neg f,
     ArithInOut.RoundedRing f,
     ArithInOut.RoundedAbs f,
     ArithInOut.RoundedMixedField f Double,
     ArithInOut.RoundedReal (Domain f),
     RefOrd.IntervalLike (Domain f),
     ArithInOut.RoundedSquareRoot (Domain f),
     Show f, Show (Domain f)
    )
    =>
    f -> [String]
ivpNames sampleFn = Map.keys $ ivpByNameMap sampleFn
    
ivpByNameMap ::
    (Var f ~ String,
     HasConstFns f,
     HasProjections f,
     RefOrd.RoundedLattice f,
     Neg f,
     ArithInOut.RoundedRing f,
     ArithInOut.RoundedAbs f,
     ArithInOut.RoundedMixedField f Double,
     ArithInOut.RoundedReal (Domain f),
     RefOrd.IntervalLike (Domain f),
     ArithInOut.RoundedSquareRoot (Domain f),
     Show f, Show (Domain f)
    )
    =>
    f ->
    Map.Map String (ODEIVP f)    
ivpByNameMap sampleFn =
    Map.fromList
    [
        ("ivpExpDecay-ev", ivpExpDecay_ev sampleFn),
        ("ivpExpDecay-uv", ivpExpDecay_uv sampleFn),
        ("ivpSpringMass-ev", ivpSpringMass_ev sampleFn),
        ("ivpSpringMass-uv", ivpSpringMass_uv sampleFn),
        ("ivpSpringMassAir-ev", ivpSpringMassAir_ev sampleFn),
        ("ivpCubicSpringMass-triple", ivpCubicSpringMass_triple sampleFn),
        ("ivpCubicSpringMass-monoCheck", ivpCubicSpringMass_monoCheck sampleFn),
        ("ivpFallAir-ishii", ivpFallAir_ishii sampleFn),
        ("ivpLorenz-ev", ivpLorenz_ev sampleFn),
        ("ivpLorenz-uv", ivpLorenz_uv sampleFn),
        ("ivpRoessler", ivpRoessler sampleFn),
        ("ivpVanDerPol-ev", ivpVanDerPol_ev sampleFn),
        ("ivpVanDerPol-uv", ivpVanDerPol_uv sampleFn)
    ]


ivpExpDecay_ev ::
    (Var f ~ String,
     HasConstFns f,
     HasProjections f,
     Neg f,
     ArithInOut.RoundedRing f,
     ArithInOut.RoundedReal (Domain f),
     Show f, Show (Domain f)
    )
    => 
    f -> ODEIVP f
ivpExpDecay_ev sampleFn =
    ivp
    where
    ivp =
        ODEIVP
        {
            odeivp_description = "x' = -x; x(" ++ show tStart ++ ") = " ++ show initialValues,
            odeivp_componentNames = ["x"],
            odeivp_field = \ [x] -> [neg x],
            odeivp_intersectDomain = Just,
            odeivp_tVar = "t",
            odeivp_tStart = toDom 0,
            odeivp_t0End = toDom 0, 
            odeivp_tEnd = toDom 1,
            odeivp_makeInitialValueFnVec = makeIV,
            odeivp_valuePlotExtents = map toDomBoth [(0,1)],
            odeivp_enclosureRangeWidthLimit = toDom 2,
            odeivp_maybeExactValuesAtTEnd = Just [(toDom 1) <*>| expMOne]
        }
    initialValues = [toDom 1]
    expMOne = exp (-1) :: Double
    makeIV =
        makeFnVecFromInitialValues componentNames initialValues
    componentNames = odeivp_componentNames ivp
    tStart = odeivp_tStart ivp
    toDom = dblToReal sampleDom
    toDomBoth (a,b) = (toDom a, toDom b)
    sampleDom = getSampleDomValue sampleFn

ivpExpDecay_uv :: 
    (Var f ~ String,
     HasConstFns f,
     HasProjections f,
     Neg f,
     ArithInOut.RoundedRing f,
     ArithInOut.RoundedReal (Domain f),
     Show f, Show (Domain f)
    )
    => 
    f -> ODEIVP f
ivpExpDecay_uv sampleFn =
    ivp
    where
    ivp =
        ODEIVP
        {
            odeivp_description = "x' = -x; x(" ++ show tStart ++ ") ∊ " ++ show initialValues,
            odeivp_componentNames = ["x"],
            odeivp_field = \ [x] -> [neg x],
            odeivp_intersectDomain = Just,
            odeivp_tVar = "t",
            odeivp_tStart = toDom 0,
            odeivp_t0End = toDom 0, 
            odeivp_tEnd = toDom 1,
            odeivp_makeInitialValueFnVec = makeIV,
            odeivp_valuePlotExtents = map toDomBoth [(0,2)],
            odeivp_enclosureRangeWidthLimit = toDom 4,
            odeivp_maybeExactValuesAtTEnd = 
                Just [lrDom (0.875 * expMOne) (1.125 * expMOne)]
        }
    initialValues = [lrDom (1 - 0.125 ) (1 + 0.125)]
    expMOne = exp (-1) :: Double
    makeIV =
        makeFnVecFromInitialValues componentNames initialValues
    componentNames = odeivp_componentNames ivp
    tStart = odeivp_tStart ivp
    lrDom = dbldblToReal sampleDom
    toDom = dblToReal sampleDom
    toDomBoth (a,b) = (toDom a, toDom b)
    sampleDom = getSampleDomValue sampleFn

ivpSpringMass_ev :: 
    (Var f ~ String,
     HasConstFns f,
     HasProjections f,
     Neg f,
     ArithInOut.RoundedRing f,
     ArithInOut.RoundedReal (Domain f),
     Show f, Show (Domain f)
    )
    => 
    f -> ODEIVP f
ivpSpringMass_ev sampleFn =
    ivp
    where
    ivp =
        ODEIVP
        {
            odeivp_description = "x'' = -x; (x,x')(" ++ show tStart ++ ") = " ++ show initialValues,
            odeivp_componentNames = ["x", "x'"],
            odeivp_intersectDomain = Just,
            odeivp_field = \ [x,x'] -> [x',neg x],
            odeivp_tVar = "t",
            odeivp_tStart = toDom 0,
            odeivp_t0End = toDom 0,
            odeivp_tEnd = toDom 1,
            odeivp_makeInitialValueFnVec = makeIV,
            odeivp_valuePlotExtents = map toDomBoth [(-1,1), (-1,1)],
            odeivp_enclosureRangeWidthLimit = toDom 4,
            odeivp_maybeExactValuesAtTEnd = Just [cosOne, neg sinOne]
        }
    initialValues = [toDom 1,toDom 0]
    cosOne = toDom (cos 1)
    sinOne = toDom (sin 1)
    makeIV =
        makeFnVecFromInitialValues componentNames initialValues
    componentNames = odeivp_componentNames ivp
    tStart = odeivp_tStart ivp
--    lrDom = dbldblToReal sampleDom
    toDom = dblToReal sampleDom
    toDomBoth (a,b) = (toDom a, toDom b)
    sampleDom = getSampleDomValue sampleFn

ivpSpringMass_uv :: 
    (Var f ~ String,
     HasConstFns f,
     HasProjections f,
     Neg f,
     ArithInOut.RoundedRing f,
     ArithInOut.RoundedReal (Domain f),
     Show f, Show (Domain f)
    )
    => 
    f -> ODEIVP f
ivpSpringMass_uv sampleFn =
    ivp
    where
    ivp =
        ODEIVP
        {
            odeivp_description = "x'' = -x; (x,x')(" ++ show tStart ++ ") ∊ " ++ show initialValues,
            odeivp_componentNames = ["x", "x'"],
            odeivp_intersectDomain = Just,
            odeivp_field = \ [x,x'] -> [x',neg x],
            odeivp_tVar = "t",
            odeivp_tStart = toDom 0,
            odeivp_t0End = toDom 0,
            odeivp_tEnd = toDom 1,
            odeivp_makeInitialValueFnVec = makeIV,
            odeivp_valuePlotExtents = map toDomBoth [(-1.5,1.5), (-1.5,1.5)],
            odeivp_enclosureRangeWidthLimit = toDom 4,
            odeivp_maybeExactValuesAtTEnd = Just $ 
                [
                    lrDom (0.875 * cosOne - 0.125 * sinOne) (1.125 * cosOne + 0.125 * sinOne)  
                , 
                    lrDom (-1.125 * sinOne - 0.125 * cosOne) (-0.875 * sinOne + 0.125 * cosOne)
                ]
        }
    initialValues = 
        [
            lrDom (1 - 0.125 ) (1 + 0.125)
        ,
            lrDom (0 - 0.125 ) (0 + 0.125)
        ]
    cosOne = cos 1
    sinOne = sin 1
    makeIV =
        makeFnVecFromInitialValues componentNames initialValues
    componentNames = odeivp_componentNames ivp
    tStart = odeivp_tStart ivp
    lrDom = dbldblToReal sampleDom
    toDom = dblToReal sampleDom
    toDomBoth (a,b) = (toDom a, toDom b)
    sampleDom = getSampleDomValue sampleFn

ivpSpringMassAir_ev :: 
    (Var f ~ String,
     HasConstFns f,
     HasProjections f,
     Neg f,
     ArithInOut.RoundedRing f,
     ArithInOut.RoundedAbs f,
     ArithInOut.RoundedReal (Domain f),
     Show f, Show (Domain f)
    )
    => 
    f -> ODEIVP f
ivpSpringMassAir_ev sampleFn =
    ivp
    where
    ivp =
        ODEIVP
        {
            odeivp_description = "x'' = -x - x'*|x'|; (x,x')(" ++ show tStart ++ ") = " ++ show initialValues,
            odeivp_componentNames = ["x", "x'"],
            odeivp_intersectDomain = Just,
            odeivp_field = \ [x,x'] -> [x',neg (x <+> (x' <*> (myAbs x')))],
            odeivp_tVar = "t",
            odeivp_tStart = toDom 0,
            odeivp_t0End = toDom 0,
            odeivp_tEnd = toDom 1,
            odeivp_makeInitialValueFnVec = makeIV,
            odeivp_valuePlotExtents = map toDomBoth [(-1.5,1.5), (-1.5,1.5)],
            odeivp_enclosureRangeWidthLimit = toDom 4,
            odeivp_maybeExactValuesAtTEnd = Nothing -- Just [cosOne, -sinOne]
        }
    initialValues = [toDom 1,toDom 0]
    myAbs fn =
        ArithInOut.absOutEff effAbsFn fn
        where
        effAbsFn = ArithInOut.absDefaultEffort fn
        
    makeIV =
        makeFnVecFromInitialValues componentNames initialValues
    componentNames = odeivp_componentNames ivp
    tStart = odeivp_tStart ivp
--    lrDom = dbldblToReal sampleDom
    toDom = dblToReal sampleDom
    toDomBoth (a,b) = (toDom a, toDom b)
    sampleDom = getSampleDomValue sampleFn


ivpCubicSpringMass_triple :: 
    (Var f ~ String,
     HasConstFns f,
     HasProjections f,
     Neg f,
     ArithInOut.RoundedRing f,
     ArithInOut.RoundedReal (Domain f),
     Show f, Show (Domain f)
    )
    => 
    f -> ODEIVP f
ivpCubicSpringMass_triple sampleFn =
    ivp
    where
    ivp =
        ODEIVP
        {
            odeivp_description = "x1' = x2 + a*x1, x2' = -x1^3 + a*(x2-x1), a = 0.001",
            odeivp_componentNames = ["x", "x'", "y", "y'", "z", "z'"],
            odeivp_intersectDomain = Just,
            odeivp_field = fieldAll6,
            odeivp_tVar = "t",
            odeivp_tStart = toDom 0,
            odeivp_t0End = toDom 0,
            odeivp_tEnd = toDom 1,
            odeivp_makeInitialValueFnVec = makeIV,
            odeivp_valuePlotExtents = map toDomBoth $ replicate 6 (-1.5,1.5),
            odeivp_enclosureRangeWidthLimit = toDom 4,
            odeivp_maybeExactValuesAtTEnd = Nothing
        }
    fieldAll6 [x,x',y,y',z,z'] = concat $ map field [[x,x'], [y,y'], [z,z']]
    fieldAll6 _ = error "ivpCubicSpringMass_triple: internal error"
    field [x,x'] = 
        [x' <+> (a <*> x),
         (neg  $ x <*> x <*> x) <+> (a <*> (x' <-> x))]
        where
        aD = 0.001 :: Double
        a = newConstFnFromSample x $ toDom aD
    field _ = error "ivpCubicSpringMass_triple: internal error"
    initialValues = map toDom $ [1,1,1,0,1,-1]
    makeIV =
        makeFnVecFromInitialValues componentNames initialValues
    componentNames = odeivp_componentNames ivp
--    tStart = odeivp_tStart ivp
--    lrDom = dbldblToReal sampleDom
    toDom = dblToReal sampleDom
    toDomBoth (a,b) = (toDom a, toDom b)
    sampleDom = getSampleDomValue sampleFn

ivpCubicSpringMass_monoCheck :: 
    (Var f ~ String,
     HasConstFns f,
     HasProjections f,
     Neg f,
     ArithInOut.RoundedRing f,
     ArithInOut.RoundedReal (Domain f),
     Show f, Show (Domain f)
    )
    => 
    f -> ODEIVP f
ivpCubicSpringMass_monoCheck sampleFn =
    ivp
    where
    ivp =
        ODEIVP
        {
            odeivp_description = "x1' = x2 + a*x1, x2' = -x1^3 + a*(x2-x1), a = 0.001",
            odeivp_componentNames = ["x1", "x2", "x1d1", "x2d1", "x1d2", "x2d2"],
            odeivp_intersectDomain = Just,
            odeivp_field = field,
            odeivp_tVar = "t",
            odeivp_tStart = toDom 0,
            odeivp_t0End = toDom 0,
            odeivp_tEnd = toDom 1,
            odeivp_makeInitialValueFnVec = makeIV,
            odeivp_valuePlotExtents = map toDomBoth $ replicate 6 (-1.5,1.5),
            odeivp_enclosureRangeWidthLimit = toDom 4,
            odeivp_maybeExactValuesAtTEnd = Nothing
        }
    field [x1,x2,x1d1,x2d1,x1d2,x2d2] =
        [x2 <+> (a <*> x1),
         (neg  $ x1 <*> x1 <*> x1) <+> (a <*> (x2 <-> x1))]
        ++
        [x2d1 <+> (a <*> x1d1),
         (neg  $ three <*> x1 <*> x1 <*> x1d1) <+> (a <*> (x2d1 <-> x1d1))]
        ++
        [x2d2 <+> (a <*> x1d2),
         (neg  $ three <*> x1 <*> x1 <*> x1d2) <+> (a <*> (x2d2 <-> x1d2))]
        where
        aD = 0.001 :: Double
        a = newConstFnFromSample x1 $ toDom aD
        three = newConstFnFromSample x1 $ toDom 3
    field _ = error "ivpCubicSpringMass_monoCheck: internal error"
    initialValues = 
        [toDom 1, (-1) `lrDom` 1
         ,toDom 1, toDom 0
         ,toDom 0, toDom 1
        ]
    makeIV =
        makeFnVecFromInitialValues componentNames initialValues
    componentNames = odeivp_componentNames ivp
    lrDom = dbldblToReal sampleDom
    toDom = dblToReal sampleDom
    toDomBoth (a,b) = (toDom a, toDom b)
    sampleDom = getSampleDomValue sampleFn


ivpFallAir_ishii :: 
    (Var f ~ String,
     HasConstFns f,
     HasProjections f,
     Neg f,
     ArithInOut.RoundedRing f,
     ArithInOut.RoundedMixedField f Double,
     ArithInOut.RoundedReal (Domain f),
     Show f, Show (Domain f)
    )
    => 
    f -> ODEIVP f
ivpFallAir_ishii sampleFn =
    ivp
    where
    ivp =
        ODEIVP
        {
            odeivp_description = "x'' = -9.8+(x'*x')/1000; (x,x')(" ++ show tStart ++ ") ∊ " ++ show initialValues,
            odeivp_componentNames = ["x", "x'"],
            odeivp_intersectDomain = Just,
            odeivp_field = \ [_x,x'] -> [x',(-9.8 :: Double) |<+>  x' <*> x' </>| (1000 :: Double)],
            odeivp_tVar = "t",
            odeivp_tStart = toDom 0,
            odeivp_t0End = toDom 0,
            odeivp_tEnd = toDom 4, -- 10000
            odeivp_makeInitialValueFnVec = makeIV,
            odeivp_valuePlotExtents = map toDomBoth $ [(-10,1.5), (-10,5)],
            odeivp_enclosureRangeWidthLimit = toDom 10,
            odeivp_maybeExactValuesAtTEnd = Nothing
        }
    initialValues = 
        [
            lrDom 1 1 -- .1
        ,
            toDom (-4.1)
        ]
    makeIV =
        makeFnVecFromInitialValues componentNames initialValues
    componentNames = odeivp_componentNames ivp
    tStart = odeivp_tStart ivp
    lrDom = dbldblToReal sampleDom
    toDom = dblToReal sampleDom
    toDomBoth (a,b) = (toDom a, toDom b)
    sampleDom = getSampleDomValue sampleFn

ivpLorenz_ev :: 
    (Var f ~ String,
     HasConstFns f,
     HasProjections f,
     Neg f,
     ArithInOut.RoundedRing f,
     ArithInOut.RoundedMixedField f Double,
     ArithInOut.RoundedReal (Domain f),
     Show f, Show (Domain f)
    )
    => 
    f -> ODEIVP f
ivpLorenz_ev sampleFn =
    ivp
    where
    ivp =
        ODEIVP
        {
            odeivp_description = "x' = 10(y-x), y' = x(28-z)-y, z' = xy - 8z/3; (x,y,z)(" ++ show tStart ++ ") ∊ " ++ show initialValues,
            odeivp_componentNames = ["x", "y", "z"],
            odeivp_intersectDomain = Just,
            odeivp_field = \ [x,y,z] -> 
                [(10 :: Double) |<*> (y <-> x),
                  (x <*> ((28 :: Double) |<+> (neg z))) <-> y,
                  (x <*> y) <-> (((8 :: Double) |<*> z) </>| (3 :: Double))
                ],
            odeivp_tVar = "t",
            odeivp_tStart = toDom 0,
            odeivp_t0End = toDom 0,
            odeivp_tEnd = toDom 32,
            odeivp_makeInitialValueFnVec = makeIV,
            odeivp_valuePlotExtents = map toDomBoth $ [(-20,20), (-20, 20), (-40,40)],
            odeivp_enclosureRangeWidthLimit = toDom 1,
            odeivp_maybeExactValuesAtTEnd = Nothing
        }
    initialValues = 
        [
            (toDom 15)
        ,
            (toDom 15)
        ,
            (toDom 36)
        ]
    makeIV =
        makeFnVecFromInitialValues componentNames initialValues
    componentNames = odeivp_componentNames ivp
    tStart = odeivp_tStart ivp
--    lrDom = dbldblToReal sampleDom
    toDom = dblToReal sampleDom
    toDomBoth (a,b) = (toDom a, toDom b)
    sampleDom = getSampleDomValue sampleFn

ivpLorenz_uv :: 
    (Var f ~ String,
     HasConstFns f,
     HasProjections f,
     Neg f,
     ArithInOut.RoundedRing f,
     ArithInOut.RoundedMixedField f Double,
     ArithInOut.RoundedReal (Domain f),
     Show f, Show (Domain f)
    )
    => 
    f -> ODEIVP f
ivpLorenz_uv sampleFn =
    ivp
    where
    ivp =
        ODEIVP
        {
            odeivp_description = "x' = 10(y-x), y' = x(28-z)-y, z' = xy - 8z/3; (x,y,z)(" ++ show tStart ++ ") ∊ " ++ show initialValues,
            odeivp_componentNames = ["x", "y", "z"],
            odeivp_intersectDomain = Just,
            odeivp_field = \ [x,y,z] -> 
                [(10 :: Double) |<*> (y <-> x),
                  (x <*> ((28 :: Double) |<+> (neg z))) <-> y,
                  (x <*> y) <-> (((8 :: Double) |<*> z) </>| (3 :: Double))
                ],
            odeivp_tVar = "t",
            odeivp_tStart = toDom 0,
            odeivp_t0End = toDom 0,
            odeivp_tEnd = toDom 8,
            odeivp_makeInitialValueFnVec = makeIV,
            odeivp_valuePlotExtents = map toDomBoth $ [(-20,20), (-20, 20), (-40,40)],
            odeivp_enclosureRangeWidthLimit = toDom 25,
            odeivp_maybeExactValuesAtTEnd = Nothing
        }
    initialValues = 
        [
            (toDom 15) <+> pmDelta
        ,
            (toDom 15) <+> pmDelta
        ,
            (toDom 36) <+> pmDelta
        ]
        where
        pmDelta = lrDom (- delta) delta
        delta = 0.01
    makeIV =
        makeFnVecFromInitialValues componentNames initialValues
    componentNames = odeivp_componentNames ivp
    tStart = odeivp_tStart ivp
    lrDom = dbldblToReal sampleDom
    toDom = dblToReal sampleDom
    toDomBoth (a,b) = (toDom a, toDom b)
    sampleDom = getSampleDomValue sampleFn

ivpRoessler :: 
    (Var f ~ String,
     HasConstFns f,
     HasProjections f,
     Neg f,
     ArithInOut.RoundedRing f,
     ArithInOut.RoundedMixedField f Double,
     ArithInOut.RoundedReal (Domain f),
     Show f, Show (Domain f)
    )
    => 
    f -> ODEIVP f
ivpRoessler sampleFn =
    ivp
    where
    ivp =
        ODEIVP
        {
            odeivp_description = 
                "Rössler attractor: x' = -y - z; y' = x + 0.2y; z' = 0.2 + z(x-5.7); "
                ++ "x(0) = 0, y(0) ∊ -8.38095+-0.01, z(0) ∊ 0.0295902+-0.01",
            odeivp_componentNames = ["x", "y", "z"],
            odeivp_intersectDomain = Just,
            odeivp_field = \ [x,y,z] -> 
                [neg $ y <+> z, 
                 x <+> ((0.2 :: Double) |<*> y),
                 (0.2 :: Double) |<+> (z <*> ((-5.7 :: Double) |<+> x))
                ],
            odeivp_tVar = "t",
            odeivp_tStart = toDom 0,
            odeivp_t0End = toDom 0,
            odeivp_tEnd = toDom 48,
            odeivp_makeInitialValueFnVec = makeIV,
            odeivp_valuePlotExtents = map toDomBoth $ [(-20,20), (-20, 20), (-20,20)],
            odeivp_enclosureRangeWidthLimit = toDom 5,
            odeivp_maybeExactValuesAtTEnd = Nothing 
        }
    initialValues = 
        [
            toDom 0
        ,
            (toDom (-8.38095)) <+> pmDelta
        ,
            (toDom 0.0295902) <+> pmDelta
        ]
        where
        pmDelta = lrDom (- delta) delta
        delta = 0.01
    makeIV =
        makeFnVecFromInitialValues componentNames initialValues
    componentNames = odeivp_componentNames ivp

    lrDom = dbldblToReal sampleDom
    toDom = dblToReal sampleDom
    toDomBoth (a,b) = (toDom a, toDom b)
    sampleDom = getSampleDomValue sampleFn

ivpVanDerPol_ev :: 
    (Var f ~ String,
     HasConstFns f,
     HasProjections f,
     Neg f,
     ArithInOut.RoundedRing f,
     ArithInOut.RoundedMixedField f Double,
     ArithInOut.RoundedReal (Domain f),
     Show f, Show (Domain f)
    )
    => 
    f -> ODEIVP f
ivpVanDerPol_ev sampleFn =
    ivp
    where
    ivp =
        ODEIVP
        {
            odeivp_description = 
                "Van der Pol oscillator: x' = y; y' = \\mu*(1 - x^2)*y - x; \\mu = " ++ show mu ++ "; " 
                ++ "x(0) = " ++ show initialX ++ ", y(0) = " ++ show initialY,
            odeivp_componentNames = ["x", "y"],
            odeivp_intersectDomain = Just,
            odeivp_field = \ [x,y] -> 
                [y, 
                 ((mu |<*> y) <*> ((1::Double) |<+> (neg x <*> x))) <+> (neg x)
                ],
            odeivp_tVar = "t",
            odeivp_tStart = toDom 0,
            odeivp_t0End = toDom 0,
            odeivp_tEnd = toDom 1,
            odeivp_makeInitialValueFnVec = makeIV,
            odeivp_valuePlotExtents = map toDomBoth $ [(-2,2), (-3, 3)],
            odeivp_enclosureRangeWidthLimit = toDom 1,
            odeivp_maybeExactValuesAtTEnd = Nothing 
        }
    initialValues = 
        [
            toDom initialX
        ,
            toDom initialY
        ]
    
    mu = 1 :: Double
    initialX = 1 :: Double
    initialY = 1 :: Double
    makeIV =
        makeFnVecFromInitialValues componentNames initialValues
    componentNames = odeivp_componentNames ivp

--    lrDom = dbldblToReal sampleDom
    toDom = dblToReal sampleDom
    toDomBoth (a,b) = (toDom a, toDom b)
    sampleDom = getSampleDomValue sampleFn


ivpVanDerPol_uv :: 
    (Var f ~ String,
     HasConstFns f,
     HasProjections f,
     Neg f,
     ArithInOut.RoundedRing f,
     ArithInOut.RoundedMixedField f Double,
     ArithInOut.RoundedReal (Domain f),
     Show f, Show (Domain f)
    )
    => 
    f -> ODEIVP f
ivpVanDerPol_uv sampleFn =
    ivp
    where
    ivp =
        (ivpVanDerPol_ev sampleFn)
        {
            odeivp_description = 
                "Van der Pol oscillator: x' = y; y' = \\mu*(1 - x^2)*y - x; \\mu = 1 " 
                ++ "x(0) = " ++ show (initialValues !! 0) ++ ", y(0) = " ++ show (initialValues !! 1),
            odeivp_makeInitialValueFnVec = makeIV
        }
    initialValues = 
        [
            (toDom initialX) <+> pmDelta
        ,
            (toDom initialY) <+> pmDelta
        ]
        where
        pmDelta = lrDom (- delta) delta
        delta = 0.25
    
    initialX = 1 :: Double
    initialY = 1 :: Double
    makeIV =
        makeFnVecFromInitialValues componentNames initialValues
    componentNames = odeivp_componentNames ivp
    lrDom = dbldblToReal sampleDom
    toDom = dblToReal sampleDom
    sampleDom = getSampleDomValue sampleFn
    
    
    