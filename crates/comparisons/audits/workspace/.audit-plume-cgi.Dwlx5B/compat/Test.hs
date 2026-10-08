module Test
 (helloScript
 ) 
where

import CGI
import qualified System
import Calc
import Database





{- helloScript :: Query -> IO [HTML]
helloScript = \req ->
 do{ let out =  check (request2query getRequest) in
     return $ welcomePage out
   }

-}

helloScript = envScript 


welcomePage :: String -> [HTML]
welcomePage str 
 =  [ prose $ "Welcome on "++str
    ]


check :: Query -> String
check query = case (tryRead (find query "expr")) of
      Nothing -> "Empty"
      Just s  -> s



{-
envScript = \query ->
  do{ env <- cgiEnv ;
    return $ envPage (query)
   }
-}

envScript = \query -> [envPage (query) | _ <- cgiEnv]


   
envPage (query)
 = [  h1 "Query"
    , prose (do_calc (find query "expr") emptyDB 10)
    ]
    

do_calc s db digits = ans
  where  (ndb, ans, ndigits) = calc db s digits
