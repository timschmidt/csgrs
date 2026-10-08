module Mime
  ( Mime(..)
  , putMime
  , getMime
  , mimeType
  , module HTMLWizard
  , module UrlEncoded
  )
where

import HTMLWizard
import PrettyHTML
import UrlEncoded
import CGISystem

data Mime
 = TextPlain  String
 | TextHtml   HTML
 | UrlEncoded Query

putMime :: Mime -> IO ()
getMime :: IO Mime
mimeType :: Mime -> String

mimeType mime
 = case mime of
     { TextPlain _   -> "text/plain"
     ; TextHtml  _   -> "text/html"
     ; UrlEncoded _  -> "application/x-www-form-urlencoded"
     }

{-
Special hack for Xiwin server

getMime
 = do{ query <- getEnv "QUERY_STRING"
     ; return $ UrlEncoded (readQuery query)
     }
-}     

getMime
 = do{ contentLength <- getEnv "CONTENT_LENGTH"
     ; stdin <- getContents
     ; let mime = take (read contentLength) stdin
     ; contentType <- getEnv "CONTENT_TYPE"
     ; case contentType of
        "application/x-www-form-urlencoded" 
          -> return $ UrlEncoded (readQuery mime)
        "text/plain"                        
         -> return $ TextPlain mime
        _  
         -> return $ UrlEncoded [("QUERY_STRING",mime)]
     }

putMime mime
 = do{ case mime of
         TextHtml html    -> putStr (showHTML html)
         TextPlain s      -> putStr s
         UrlEncoded query -> putStr (show query)
     }
