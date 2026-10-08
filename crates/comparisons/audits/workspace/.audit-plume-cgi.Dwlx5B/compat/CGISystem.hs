module CGISystem
 ( getEnv
 , cgiEnv
 , errorLog
 )
where

import qualified System

errorLog msg
 = do{ appendFile "c:/hugs/lib/D.log" msg }
 
getEnv var
 = do{ r <- try (System.getEnv var)
     ; return (either (const "") id r)
     }
     
cgiEnv
 = do{ cgiValues <- mapM getEnv cgiVars
     ; return (cgiVars `zip` cgiValues)
     }

cgiVars 
 = [ "HTTP_CONNECTION"
   , "HTTP_HOST"
   , "HTTP_USER_AGENT"
   , "HTTP_UA_CPU"
   , "HTTP_UA_OS"
   , "HTTP_UA_COLOR"
   , "HTTP_UA_PIXELS"
   , "HTTP_ACCEPT_LANGUAGE"
   , "HTTP_REFERER"
   , "HTTP_ACCEPT"
   , "HTTP_CONTENT_LENGTH"
   , "SERVER_SOFTWARE"
   , "SERVER_NAME"
   , "SERVER_PORT"
   , "SERVER_PROTOCOL"
   , "GATEWAY_INTERFACE"
   , "REQUEST_METHOD"
   , "SCRIPT_NAME"
   , "CONTENT_TYPE"
   , "CONTENT_LENGTH"
   , "REMOTE_USER"
   , "REMOTE_HOST"
   , "REMOTE_ADDR"
   , "PATH_INFO"
   , "PATH_TRANSLATED"
   , "QUERY_STRING"
   ]

try :: IO a -> IO (Either IOError a)
try p
 = catch (p >>= (return . Right)) (return . Left)
