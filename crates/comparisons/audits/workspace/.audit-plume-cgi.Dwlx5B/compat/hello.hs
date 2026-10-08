module hello 
 (helloScript
 ) 
where

import CGI
import qualified System

helloScript :: Query -> IO [HTML]
helloScript = \req ->
 do{ out <- "hello" -- check (request2query req)
     ; return $ (p out)
   }

welcomePage (server,host)
 =  [ prose $ "Welcome on "++server++", "
    , prose $ "visitor from "++host++"!"
    ]

{-
check :: Query -> String
check query = case (tryRead (find query "expr")) of
      Nothing -> "Empty";
      Just s  -> s
-}