{-# LANGUAGE DeriveDataTypeable #-}
{-|
    Module      :  Numeric.AERN.Basics.Exception
    Description :  exception type to signal AERN specific errors  
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    An exception type to be thrown on various arithmetic exceptions
    whose proper pure handling would be inefficient.
    'Control.Exception.ArithmeticException' is not
    flexible enough, eg because often we will not be checking
    overflows but DomViolations instead.
    
    The default exeception policy is:
    
    * no operation should ever return DomViolation; when a DomViolation represents "any value"/bottom
      (eg with 0/0 or &infin - &infin),
      it should be rounded up to +&infin; or down to -&infin;; when a DomViolation represent
      an illegal argument exception (eg with log(-1)), an AERN DomViolation exception should
      be thrown with an appropriate message
    
    * intervals support infinite endpoints
    
    * polynomial coefficients must be finite, overflows of coefficients detected
      and result in a special polynomial denoting the constant function +&infin; or -&infin;
-}
module Numeric.AERN.Basics.Exception where

import Prelude hiding (catch)

import Control.Exception
import Data.Typeable
import System.IO.Unsafe

data AERNException =
    AERNException String
    | AERNIllegalValue String
    | AERNDomViolationException String
    | AERNMaybeDomViolationException String
    deriving (Show, Typeable)

instance Exception AERNException

evalCatchAERNExceptions :: String -> t -> Either AERNException t
evalCatchAERNExceptions contextDescription a =
    unsafePerformIO $ catch (evaluateEmbed a) handler
    where
    handler e@(AERNException msg) =
        do
        putStrLn $ "caught AERN exception: " ++ msg
        return (Left e)
    handler e@(AERNIllegalValue msg) =
        do 
        putStrLn $ "caught AERN illegal value exception: " ++ msg
        return (Left e)
    handler e@(AERNDomViolationException msg) =
        do 
        putStrLn $ "caught AERN operation domain violation exception: " ++ msg
        return (Left e)
    handler e@(AERNMaybeDomViolationException msg) =
        do 
        putStrLn $ "caught AERN potential operation domain violation exception: " ++ msg
        return (Left e)
    evaluateEmbed a =
        do
        aa <- evaluate a
        return $ Right aa

evalCatchDomViolationExceptions :: String -> t -> Either AERNException t
evalCatchDomViolationExceptions contextDescription a =
    case evalCatchAERNExceptions contextDescription a of
        Left e@(AERNDomViolationException _) -> Left e
        Left e@(AERNMaybeDomViolationException _) -> Left e
        Left e ->
            unsafePerformIO $
            do
            putStrLn $ contextDescription ++ ": rethrowing" 
            throw e
        r -> r

raisesAERNException :: String -> t -> Bool
raisesAERNException contextDescription a =
    case (evalCatchAERNExceptions contextDescription a) of
        (Left _) -> True
        _ -> False

raisesDomViolationException :: String -> t -> Bool
raisesDomViolationException contextDescription a =
    case (evalCatchDomViolationExceptions contextDescription a) of
        (Left _) -> True
        _ -> False

class HasLegalValues t where
  maybeGetProblem :: t -> Maybe String
  
detectIllegalValues :: (HasLegalValues t) => String -> t -> t
detectIllegalValues contextDescription value =
    case maybeGetProblem value of
        Nothing -> value
        Just problemDescription ->   
            throw $ AERNIllegalValue $ 
                contextDescription ++ ": " ++ problemDescription
 