module Request
 ( Request(..)
 , getRequest
 , request2query
 )
where

import Mime
import UrlEncoded
import CGISystem

data Request
 = GET Query
 | POST Mime

request2query request
 = case request of
     GET query               -> query
     POST (UrlEncoded query) -> query
     POST _                  -> emptyQuery

getRequest :: IO Request
getRequest
 = do{ method <- getEnv "REQUEST_METHOD"
     ; case method of 
         "POST" -> getPOST
         "GET"  -> getGET
         _      -> getGET
     }

getGET
 = do{ query <- getEnv "QUERY_STRING"
     ; return $ GET (readQuery query)
     }

getPOST
 = do{ mime <- getMime
     ; return $ POST mime
     }