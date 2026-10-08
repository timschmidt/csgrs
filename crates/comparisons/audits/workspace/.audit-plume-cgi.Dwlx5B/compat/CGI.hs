module CGI
 ( wrapper
 , cgi
 , tryRead
 , module Request
 , module Response
 , module Mime
 , module CGISystem
 )
where

import Request
import Response
import Mime
import CGISystem

tryRead s
 = case readsPrec 0 s of
     [(a,"")]  -> Just a
     otherwise -> Nothing
     
wrapper :: (Request -> IO Response) -> IO ()
wrapper worker
 = do{ request <- getRequest
     ; response <- worker request
     ; putResponse response
     }

cgi :: (Query -> IO HTML) -> IO ()
cgi script = wrapper $ \request ->
   do{ document <- script (request2query request)
     ; return (html document)
     }