module Hello 
 (helloScript
 ) 
where

import CGI
import qualified System

helloScript :: Query -> IO [HTML]
helloScript = \query ->
 do{ host <- getEnv "QUERY_STRING"
   ; server <- getEnv "SERVER_NAME"
   ; return $ welcomePage (server,host)
   }

welcomePage (server,host)
 =  [ prose $ "Welcome on "++server++", "
    , prose $ "visitor from "++host++"!"
    ]