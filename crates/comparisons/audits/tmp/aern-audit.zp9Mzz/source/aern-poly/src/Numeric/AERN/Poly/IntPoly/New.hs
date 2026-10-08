{-# OPTIONS_GHC -fno-warn-orphans #-}
{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE UndecidableInstances #-}
{-|
    Module      :  Numeric.AERN.Poly.IntPoly.New
    Description :  creating constant and single-variable polynomials  
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Creating constant and single-variable polynomials.
-}

module Numeric.AERN.Poly.IntPoly.New
--    (
--    )
where
    
import Numeric.AERN.Poly.IntPoly.Config
import Numeric.AERN.Poly.IntPoly.IntPoly
import Numeric.AERN.Poly.IntPoly.Show ()
    
import Numeric.AERN.RmToRn.New
import Numeric.AERN.RmToRn.Domain

import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut
import Numeric.AERN.RealArithmetic.RefinementOrderRounding.Operators
import Numeric.AERN.RealArithmetic.ExactOps

import qualified Numeric.AERN.RefinementOrder as RefOrd
--import Numeric.AERN.RefinementOrder.OpsDefaultEffort

import Numeric.AERN.Basics.SizeLimits
import Numeric.AERN.Basics.Consistency

import qualified Data.Map as Map
import qualified Data.IntMap as IntMap
--import Data.List (elemIndex)

{-- Basic function-approximation specific ops --}

instance
    (Ord var, ArithInOut.RoundedReal cf,
     RefOrd.IntervalLike cf) 
    => 
    (HasDomainBox (IntPoly var cf))
    where
    type (Domain (IntPoly var cf)) = cf
    type (Var (IntPoly var cf)) = var
    type (VarBox (IntPoly var cf)) = Map.Map var
    getSampleDomValue (IntPoly cfg _) = 
        ipolycfg_sample_cf cfg
    defaultDomSplit _ =
        defaultDomSplitUsingEndpointsDefaultEffort
    getDomainBox (IntPoly cfg _) = Map.fromList $ zip vars doms
        where
        vars = ipolycfg_vars cfg
        doms = zipWith (<+>) domsLE domsLZ
        domsLZ = ipolycfg_domsLZ cfg
        domsLE = ipolycfg_domsLE cfg
    getVarDoms (IntPoly cfg _) = 
        cfg2vardomains cfg
    getNSamplesFromInsideDomainBox sampleP@(IntPoly cfg _) dombox n =
        getNSamplesFromInsideDomainBoxUsingEndpointsDefaultEffort sampleDom sampleP dombox n
        where
        sampleDom = ipolycfg_sample_cf cfg
    getSampleFromInsideDomainBox sampleP@(IntPoly cfg _) dombox =
        getSampleFromInsideDomainBoxUsingEndpointsDefaultEffort sampleDom sampleP dombox
        where
        sampleDom = ipolycfg_sample_cf cfg

instance 
    (Ord var, Show var, 
     ArithInOut.RoundedReal cf,
     RefOrd.IntervalLike cf
    ) 
    => 
    CanAdjustDomains (IntPoly var cf)
    where
    adjustDomain poly@(IntPoly cfg terms) var newDom =
        IntPoly adjustedCfg terms
        where
        adjustedCfg =
--            adjustSizeLimitsToVarsAndDombox poly vars adjustedDombox cfg
            cfgAdjustDomains vars domains cfg
            where
            domains =
                map getDomain vars
            getDomain var2 =
                case lookupVar adjustedDombox var2 of
                    Just dom -> dom
                    Nothing ->
                        error $ 
                            "aern-poly: IntPoly adjustSizeLimitsToVarsAndDombox: variable "
                            ++ show var2 ++ " not in the given domain box"
        vars = ipolycfg_vars cfg
        adjustedDombox = insertVar var newDom $ getDomainBox poly

instance
    (Ord var, Show var,
     ArithInOut.RoundedReal cf,
     RefOrd.IntervalLike cf)
    => 
    (HasSizeLimits (IntPoly var cf))
    where
    type (SizeLimits (IntPoly var cf)) = IntPolySizeLimits cf
    defaultSizeLimits (IntPoly cfg _) =
        defaultIntPolySizeLimits sampleCf cf_limits arity
        where
        cf_limits = defaultSizeLimits sampleCf
        sampleCf = ipolycfg_sample_cf cfg
        arity = length $ ipolycfg_vars cfg 
    getSizeLimits (IntPoly cfg _) = ipolycfg_limits cfg

instance 
    (Ord var, Show var, Show cf, 
     ArithInOut.RoundedReal cf,
     HasConsistency cf,
     RefOrd.IntervalLike cf) 
    => 
    (HasConstFns (IntPoly var cf))
    where
    newConstFn limits varDoms value = 
        IntPoly cfg $ mkConstTerms value vars
        where
        cfg = 
            cfgAdjustDomains vars domains $
                defaultIntPolyCfg sampleCF limits
--        domains = 
--            case (sequence $ map (lookupVar dombox) vars) of
--                Just ds -> ds
--                Nothing ->
--                    error "aern-poly: IntPoly newProjection: incompatible domain box"
        (vars, domains) = unzip varDoms
        sampleCF = value

mkConstTerms ::
    (HasConsistency cf, Show cf)
    => 
    cf -> [var] -> IntPolyTerms var cf
mkConstTerms value vars 
--    | valueInConsistent = 
--        error $ "aern-poly: mkConstTerms: inconsistent coefficient" ++ show value
    | otherwise = 
        aux vars
    where
    aux [] = IntPolyC value
    aux (var:rest) = IntPolyV var $ IntMap.singleton 0 (aux rest)
--    valueInConsistent = 
--        (isConsistentEff (consistencyDefaultEffort value) value) == Just False

{-- Basic function-approximation specific ops --}

instance
    (HasSampleFromContext cf,
     HasSampleFromContext var,
     Ord var, Show var, Show cf, 
     RefOrd.IntervalLike cf, 
     ArithInOut.RoundedReal cf, 
     HasConsistency cf)
    =>
    HasSampleFromContext (IntPoly var cf)
    where
    sampleFromContext = 
        newConstFn sizeLimits [(sampleFromContext, sampleCf)] sampleCf
        where
        sizeLimits = defaultIntPolySizeLimits sampleCf (defaultSizeLimits sampleCf) arity 
        sampleCf = sampleFromContext
        arity = 1

instance
    (Ord var, Show var, Show cf,
     ArithInOut.RoundedReal cf,
     HasConsistency cf,
     RefOrd.IntervalLike cf)
    =>
    HasZero (IntPoly var cf)
    where
    zero sampleP = newConstFnFromSample sampleP $ zero sampleCf
        where
        sampleCf = getSampleDomValue sampleP

instance
    (Ord var, Show var, Show cf,
     HasConsistency cf,
     ArithInOut.RoundedReal cf,
     RefOrd.IntervalLike cf)
    =>
    HasOne (IntPoly var cf)
    where
    one sampleP = newConstFnFromSample sampleP $ one sampleCf
        where
        sampleCf = getSampleDomValue sampleP

instance
    (Ord var, Show var, Show cf,
     ArithInOut.RoundedReal cf,
     HasConsistency cf,
     RefOrd.IntervalLike cf)
    =>
    HasInfinities (IntPoly var cf)
    where
    plusInfinity sampleP = newConstFnFromSample sampleP $ plusInfinity sampleCf
        where
        sampleCf = getSampleDomValue sampleP
    minusInfinity sampleP = newConstFnFromSample sampleP $ minusInfinity sampleCf
        where
        sampleCf = getSampleDomValue sampleP
    excludesMinusInfinity (IntPoly _cfg terms) =
        and $ termsCollectCoeffsWith excludesInfty terms
        where
        excludesInfty _ coeff =
            excludesMinusInfinity coeff 
    excludesPlusInfinity (IntPoly _cfg terms) =
        and $ termsCollectCoeffsWith excludesInfty terms
        where
        excludesInfty _ coeff =
            excludesPlusInfinity coeff 

instance
    (Ord var, Show var, Show cf,
     HasConsistency cf,
     ArithInOut.RoundedReal cf,
     RefOrd.IntervalLike cf)
    =>        
    CanAddVariables (IntPoly var cf)
    where
    addVariablesFront varDoms (IntPoly cfg terms)
        =
        IntPoly cfgWithNewVars termsWithNewVars
        where
        (vars, doms) = unzip varDoms
        cfgWithNewVars =
            cfg
            {
                ipolycfg_vars = newVars,
                ipolycfg_domsLE = newDomsLE,
                ipolycfg_domsLZ = newDomsLZ
            }
            where
            newVars = vars ++ (ipolycfg_vars cfg)
            newDomsLE = domsLE ++ (ipolycfg_domsLE cfg)
            newDomsLZ = domsLZ ++ (ipolycfg_domsLZ cfg)
            domsLE = map (fst . RefOrd.getEndpointsOut) doms
            domsLZ = zipWith (<->) doms domsLE
        termsWithNewVars = addVars vars
            where
            addVars [] = terms
            addVars (var : rest) =
                IntPolyV var $ IntMap.singleton 0 $ addVars rest
    addVariablesBack varDoms (IntPoly cfg terms)
        =
        IntPoly cfgWithNewVars termsWithNewVars
        where
        (vars, doms) = unzip varDoms
        cfgWithNewVars =
            cfg
            {
                ipolycfg_vars = newVars,
                ipolycfg_domsLE = newDomsLE,
                ipolycfg_domsLZ = newDomsLZ
            }
            where
            newVars = (ipolycfg_vars cfg) ++ vars
            newDomsLE = (ipolycfg_domsLE cfg) ++ domsLE
            newDomsLZ = (ipolycfg_domsLZ cfg) ++ domsLZ
            domsLE = map (fst . RefOrd.getEndpointsOut) doms
            domsLZ = zipWith (<->) doms domsLE
        termsWithNewVars = addVarsToAllTerms terms
            where
            addVarsToAllTerms (IntPolyC value) =
                mkConstTerms value vars
            addVarsToAllTerms (IntPolyV var powers) =
                IntPolyV var $ IntMap.map addVarsToAllTerms powers
             
instance
    (Ord var, Show var, Show cf, Show (SizeLimits cf),
     ArithInOut.RoundedReal cf,
     RefOrd.IntervalLike cf)
    =>        
    CanRenameVariables (IntPoly var cf)
    where
    renameVar oldVar newVar p@(IntPoly cfg terms) 
        | oldVar == newVar = p
        | newVar `elem` vars =
            error $ "variable " ++ show newVar ++ " already in " ++ show p
        | otherwise = 
            IntPoly cfgRenamed (renameInTerms terms)
        where
        vars = ipolycfg_vars cfg
        cfgRenamed = cfg
            {
                ipolycfg_vars = renameOneOccurrenceInList vars 
            }
        renameInTerms t@(IntPolyC _) = t
        renameInTerms (IntPolyV var powers) 
            | var == oldVar = (IntPolyV newVar powers)
            | otherwise =
                IntPolyV var $ IntMap.map renameInTerms powers
              
        renameOneOccurrenceInList (var : rest)
            | var == oldVar = newVar : rest
            | otherwise = var : (renameOneOccurrenceInList rest)
        renameOneOccurrenceInList _ =
            error $
                "variable " ++ show oldVar ++ " not in " ++ show p

instance 
    (Ord var, Show var,  Show cf,
     HasConsistency cf,
     ArithInOut.RoundedReal cf,
     RefOrd.IntervalLike cf) => 
    (HasProjections (IntPoly var cf))
    where
    newProjection limits varDoms var 
        | isExact domOfVar == Just True =
            IntPoly cfg $ mkConstTerms domOfVar vars
        | otherwise = 
            IntPoly cfg $ mkProjTerms cfg var vars domsLE
        where
        domsLE = ipolycfg_domsLE cfg
        cfg = cfgAdjustDomains vars domains $ defaultIntPolyCfg sampleCF limits
        (sampleCF : _)  = domains
        (vars, domains) = unzip varDoms
        domOfVar = 
            case lookup var varDoms of
                Just v -> v
                _ -> error "newProjection called with a variable that is not present in the domain." 
        
mkProjTerms :: 
    (Eq var, Show var, Show cf, HasOne cf, 
     HasConsistency cf)
    =>
    IntPolyCfg _var cf -> 
    var -> 
    [var] -> 
    [cf] -> 
    IntPolyTerms var cf
mkProjTerms cfg var vars domsLE = aux vars domsLE
    where
    aux [] [] = 
        error $ 
            "IntPoly: newProjection: variable " ++ show var 
            ++ " not among specified variables " ++ show vars
    aux (cvar : restVars) (domLE : restDoms)
        | cvar == var = 
            IntPolyV var $ 
                IntMap.fromAscList $ 
                    [(1, mkConstTerms o restVars),
                     (0, mkConstTerms domLE restVars)]
        | otherwise = 
            IntPolyV cvar $ IntMap.singleton 0 (aux restVars restDoms)
        where
        o = one sampleCf
--        z = zero sampleCf
        sampleCf = ipolycfg_sample_cf cfg 
    aux _ _ = error "aern-intpoly internal error in New.mkProjTerms"
            
            

            
