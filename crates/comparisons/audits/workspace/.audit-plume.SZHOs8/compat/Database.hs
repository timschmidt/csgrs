module Database (

-- Types
Database, DBitem,

-- Constants
emptyDB, 

-- Functions
dbNext, dbFetch, 
dbDigits,
dbStore
	) where

import ParseTree



data DBitem   = DBI (String, Exp1)
type Database = [DBitem]

emptyDB :: Database
--emptyDB = [DBI ("Digits", (Assign "10" (Term (Factor (Int "10")))))]
emptyDB = []


-- Return next string and remaining db
dbNext []              = undefined
dbNext (DBI (s,p):db') = (s,p,db') 



-- Fetch parser associated with string
dbFetch :: Database -> String -> Exp1
dbFetch [] _                        = NoVar
dbFetch (DBI (s,p):x) v | s==v      = p
dbFetch (a:x) v         | otherwise = dbFetch x v




-- dbDigits : Fetch Digits var and convert to integer
dbDigits :: Database -> Int
dbDigits []                            = undefined
dbDigits _                             = undefined
{-
dbDigits (DBI (s,val):x) | s=="Digits" = read val   -- Haskell internals
dbDigits (a:x)           | otherwise   = dbDigits x
-}


-- Remove item from database by adding new one and filtering old ones.
-- Not strictly necessary ... could be more efficient if need be.
dbStore :: Database -> String -> Exp1 -> Database
dbStore db s p = (DBI (s,p):filter (\(DBI (v,_)) -> s/=v) db)


