module WebSite 
 ( Page
 , Title
 , WebSite (..)
 , website
 , mkSite
 )
where

import CGI
     
type Page = [HTML]
type Title = String
type Position = (Int,Int)


data WebSite
 = WebSite Title (Query -> IO Page) [WebSite]

data Navigator
 = Prev Position
 | Up   Position
 | Next Position
 
{-
Function site maps an abstract website into a script.
-}

website :: WebSite -> IO ()
website s = cgi $ \query ->
 do{ case tryRead (find query "page") of
       Nothing   -> getPage (0,0) query
       Just page -> getPage page  query
   } where
        getPage p query
         = case lookup p (mkSite (0,0) s []) of
             Just script -> do{ script query }
             Nothing     -> do{ return $ errorPage }
        
errorPage
 = page "Error" [h1 "The requested page is not available on this site" ]
 
mkSite :: Position -> WebSite -> [Navigator] -> [(Position, Query -> IO HTML)]
mkSite this (WebSite title script sites) ns
 = (this, \query -> do{ home <- script query
                      ; return $ page title (home ++ [navigationBar ns])
                      }
   ) 
   : (concat $ nextSites this sites)

nextSites :: Position -> [WebSite] -> [ [(Position,Query -> IO HTML)] ]
nextSites up@(d,i) sites
 = [ let this = (d+1,j) 
     in mkSite this site (prev this ++ [Up up] ++ next this)
   | (j,site) <- [0..] `zip` sites 
   ] where 
       n = length sites
       
       prev (d,0) = []
       prev (d,j) = [Prev (d,j-1)]

       next (d,j) | j >= (n-1) = []
                  | otherwise  = [Next (d,j+1)]

navigationBar :: [Navigator] -> HTML
navigationBar ns
 = p [ hr
     , table [ row [ navigationButton n | n <- ns ] ] 
     , hr
     ]

navigationButton :: Navigator -> HTML
navigationButton n
 =  cell 1 [post [ button ("",img), hidden ("page",page) ]]
    where
      (img,page)
       = case n of
           Up page   -> ("CgiLib/Buttons/Home.gif",show page)
           Prev page -> ("CgiLib/Buttons/Prev.gif",show page)
           Next page -> ("CgiLib/Buttons/Next.gif",show page)
            
 
