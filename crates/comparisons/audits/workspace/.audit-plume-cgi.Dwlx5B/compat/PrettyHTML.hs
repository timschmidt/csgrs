module PrettyHTML
 ( HTML
 , showHTML
 )
where

import Pretty
import HTML

showHTML html
 = show (ppHTML html)

ppHTML (Element tag  attributes  html)
 = ppElement tag attributes html

ppHTML (Text t)
 = text t

ppElement tag attributes []
 = ppOpenTag tag attributes <> ppCloseTag tag

ppElement tag attributes html
 =  sep [ ppOpenTag tag attributes
        , nest indent (sep (map ppHTML html))
        , ppCloseTag tag
        ]

ppOpenTag tag attributes
 = case (ppAttributes attributes) of
      [] -> text "<" <> text tag <> text ">"
      as -> sep [ text "<" <> text tag
                , nest indent (sep as)
                , text ">"
                ]

ppCloseTag tag
 = text "</" <> text tag <> text ">"

ppAttributes as
 = [ text name <> text "=" <> text (show value)
   | (name , value) <- as
   ] -- Can be empty :-)

indent = 2 :: Int

