import Control.Exception
import Numeric.AERN.Basics.Exception

main :: IO ()
main = do
    let bad :: Int
        bad = throw (AERNDomViolationException "probe")
    case evalCatchAERNExceptions "scalar" bad of
        Left (AERNDomViolationException _) -> putStrLn "scalar exception captured"
        _ -> error "scalar exception was not captured"
    case evalCatchAERNExceptions "container" [bad] of
        Right [element] -> do
            result <- try (evaluate element) :: IO (Either AERNException Int)
            case result of
                Left (AERNDomViolationException _) -> putStrLn "nested exception escaped the pure wrapper; result was Right"
                _ -> error "expected deferred exception"
        _ -> error "unexpected nested wrapper result"
    if raisesAERNException "list" [bad]
        then error "unexpected deep forcing"
        else putStrLn "raisesAERNException reports False for the unevaluated nested exception"
