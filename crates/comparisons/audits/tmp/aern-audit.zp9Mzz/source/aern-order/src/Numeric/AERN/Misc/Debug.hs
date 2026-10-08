{-|
    Module      :  Numeric.AERN.Misc.Debug
    Description :  miscellaneous debugging functions  
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Miscellaneous debugging functions.
-}

module Numeric.AERN.Misc.Debug where

import System.IO.Unsafe

unsafePrint msg val =
    unsafePerformIO $
        do
        putStrLn $ "unsafe: " ++ msg
        return val

unsafePrintReturn msg a =
    unsafePrint (msg ++ show a) a

{-| Like 'and' except each boolean parameter has a message
   associated with it and if one of the parameters
   is false, its message is unsafely printed to the console. 
 -}
andUnsafeReportFirstFalse :: [(Bool, String)] -> Bool
andUnsafeReportFirstFalse [] = True
andUnsafeReportFirstFalse ((True, _) : rest) = andUnsafeReportFirstFalse rest
andUnsafeReportFirstFalse ((False, msg) : _) =
   unsafePrint msg False
   
{-| Like 'or' except each parameter has a message
   associated with it and if all of the parameters
   are false, all their messages are unsafely printed to the console. 
 -}
orUnsafeReportFalse :: [(Bool, String)] -> Bool
orUnsafeReportFalse boolMsgList
    | notAllFalse = True 
    | otherwise =
        unsafePrint allMessages False
    where
    notAllFalse = or boolList
    allMessages = unlines $ messageList
    (boolList, messageList) = unzip boolMsgList
