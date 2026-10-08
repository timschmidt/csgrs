HASKELL MONADS VIA EXAMPLES

This file explains what Haskell monads are. It is written in the "literal Haskell" format in which everything is a comment, except those lines that start with >. You can run the code from ghci by typing ":l monads.lhs" without quotes.

There are any number of introductions on monads written by Haskell bloggers around the internet. You may want to look at some of them for further insight. We shall introduce monads by examples.

Haskell is a purely functional language. In particular, it does not allow direct invocation of any operations that would allow us to detect the order of evaluation. This rules out mutable variables, exceptions, I/O, and many other useful programming concepts. All these can be put back into Haskell with a clever way of programming known as "monadic style".

Our first example is a very simple kind of exception which is sometimes known as "abort". It is an operation which aborts whatever is being computed and cannot be intercepted. To have something like that in Haskell, we need to be explicit about values which may trigger abort. So we define a datatype Abortable t which means "either an ordinary value of type t, or a special value Aborted indicating that abort happened" (the deriving clause is of no concern right now, it means Haskell will automatically derive a way of showing abortable values on scren):

> data Abortable t = Value t | Aborted
>                    deriving Show

The constant Aborted signifies a value whose computation was aborted. The other possibility is a value of the form Value v, which signifies a successfully computed value v (abort did not happen). Haskell insists that we write Value v rather than just v. This way it can tell the difference between ordinary values and values that could have been aborted but were not.

An ordinary value v of type t may always be converted to an abortable value Value v of type Abortable t. The operation that does this is called "return" in Haskell and is the first half of a monad.

The second half of a monad is an operation >>= (called "bind") which combines an abortable value

    x :: Abortable t

and a function

    f :: t -> Abortable t

which expects an ordinary value and outputs an abortable one. This is written as

    x >>= f

What should x >>= f be? Well, if x is Aborted then x >>= f must also be Aborted (we want Aborted to act as an uncatchable exception). If x is of the form Value v then x >>= f should be f v. This brings us to the official monad definition:

> instance Monad Abortable where
>    return v        = Value v
>    Aborted >>= f   = Aborted
>    (Value v) >>= f = f v

In principle we can use return and >>= to compute with abortable values, but it is very cumbersome. For example,  suppose we have a division operation whose result is an abortable integer,

> divide :: Int -> Int -> Abortable Int
> divide x 0 = Aborted
> divide x y = Value (x `div` y)

In order to compute the function which maps x and y to (x/y + y/x) we have to write

> f :: Int -> Int -> Abortable Int
> f x y = (divide x y) >>= (\u -> (divide y x) >>= (\v -> return (u + v)))

With good indentation it is possible to improve the code a bit:

> g :: Int -> Int -> Abortable Int
> g x y = (divide x y) >>= (\u ->
>         (divide y x) >>= (\v ->
>         return (u + v)))

This we can read as: feed the abortable value divide x y into u, feed the abortable value divide y x into v, then return the abortable value u + v. The operator >>= makes sure that the whole result is Aborted if either u or v is. Haskell has special notation which significantly improves the code:

> h :: Int -> Int -> Abortable Int
> h x y = do u <- divide x y
>            v <- divide y x
>            return (u + v)

The functions g and h are the exact same thing written in different notations.

The Abortable monad is already built into Haskell, except it is called the Maybe monad. Instead of Value v and Aborted it has Just v and Nothing. To give an example of its use, suppose we have an association list

> lst = [("apple", 3), ("orange", 10), ("banana", 2), ("stone", 6)]

and would like to find the value corresponding to "orange". We can do this by writing

> y = lookup "orange" lst             -- y equals Just 10

The answer is Just 10 because lookup returns "Maybe" values (it may find something, or not). So if we lookup something that is not in the list we get Nothing:

> z = lookup "cow" lst                -- z equals Nothing

The monad and do notation come in handy if we have a piece of code that does several lookups and we want it to fail as soon as one of the lookups fails, e.g.

> sum = do u <- lookup "banana" lst
>          v <- lookup "apple" lst
>          w <- lookup "cherry" lst
>          return (u + v + w)

The value of sum is Nothing because the third lookup fails. Had it succeeded, we would get Just i for some integer i. Here is one way of writing the same code without the do notation:

> sum' = case (lookup "banana" lst, lookup "apple" lst, lookup "cherry" lst) of
>           (Just u, Just v, Just w) -> Just (u + v + w)
>           (_, _, _) -> Nothing

You may judge for yourself which one is more readable.

The next example of a monad is non-deterministic choice operator. Suppose in a computation we have  choice points, at which we can choose any value from a given list. Such a situation occurs when we perform a search over a tree, and at each node we can choose one of the branches. We would like a convenient way of programming choice points so that when we evaluate an expression with choice points all possible combinations of choices are taken, and the possible results are stored in a list.

First we define a dataype Chooose t which holds possible results of a computation that has choice points:

> data Choose t = Choices [t]
>                 deriving Show

For example, the value Choices [1,2,3] means that the possible outcomes are 1, 2, and 3. To get a monad,

> instance Monad Choose where

we must define return, which converts an ordinary value v of type t to one of type Choose t. This part is more or less obvious:

>     return v = Choices [v]

To define >>= we have to think about how to combine a value

    Choices [v1, ..., vn]

with a function f which accepts an ordinary value v and returns some choices. It is not hard to see that we should loop over v1, ..., vn, get all the different choices produced by f and combine them into a single choice, like this:

>     (Choices lst) >>= f = Choices (combine f lst [])
>         where combine f [] ws     = ws
>               combine f (v:vs) ws = let Choices us = f v
>                                     in combine f vs (ws ++ us)

Now we can use the do notation to write programs like this:

> -- all sums of the form x+y where x is 1,...,10 and y is 1,...,x.
> c = do x <- Choices [1..10]
>        y <- Choices [1..x]
>        return (x + y)
> -- c is Choices [2,3,4,4,5, ..., 18,19,20] (55 elements)

Without the do notation we would have to have a double loop of some kind. Here is a more complicated example:

> -- binary k computes all binary lists of length k
> binary :: Int -> Choose [Int]
> binary 0 = return []
> binary k = do b <- Choices [0,1]
>               bs <- (binary (k-1))
>               return (b:bs)

The monad we just considered is also built into Haskell, and is known as the "list monad". Possible choices are represented as a list, so we can write [x1,...,xn] rather than Choices [x1,...,xn]:

> -- all sums of the form x+y where x is 1,...,10 and y is 1,...,x.
> d = do x <- [1..10]
>        y <- [1..x]
>        return (x + y)
> -- d is [2,3,4,4,5, ..., 18,19,20] (55 elements)

Our last example is the state monad. In procedural programming languages we have variables that can be updated, i.e., their values change as computation progresses. We say that the computation is "stateful" because it depends on the state of the variables, and it may change the state of the variable.

In general, stateful computation is a function which accepts the current state and returns a result together with the new state. That is, in Haskell a stateful computation computing a result of type t and having access to state of type s is a function

    v :: s -> (t, s)

which accepts the current state m, and returns a pair (x, m') representing the computed value x and the new state m'. (If you've never heard of this before, you should pause to digest what has just been said.)

We define a Haskell type of such stateful computations:

> data State s t = Stateful (s -> (t, s))

Here is an example of a stateful computation:

> incr = Stateful $ \m -> (m, m+1)

By the way, the mysterious $ does nothing, except it allows us to write fewer parenthesis. The meaning of f $ x is the same as f x, except that $ is an infix operator with low precedence. If we wrote incr without $, we would have to write Stateful (\m -> (m, m+1)).

The definition of incr is read as follows: incr is a stateful computation which takes a memory location m and returns the current value of m. It also increases the memory location by 1. In other words, this would be written as m++ in C.

But how do we actually execute incr? We give it the initial value of the memory location and out comes the result together with updated memory:

> (r1,m1) = let Stateful v = incr in v 42       -- r1 is 42, m1 is 43

This is quite ugly, so we define an auxiliary function

> run m (Stateful v) = v m

Now we can write

> (r2, m2) = run 42 incr                        -- r2 is 42, m2 is 43

and read it as "run incr with initial state 42". Let us also define the state monad:

> instance Monad (State s) where

Return converts a pure (non-stateful) value to a stateful one that doesn't change the state:

>     return x = Stateful $ \m -> (x, m)

The operation >>= chains together stateful computations:

>     v >>= f  = Stateful $ \m -> let (x, m') = run m v in run m' (f x)

To compute v >>= f in state m we first run v in state m to obtain a result x and the new state m'. We then compute f x in state m'.

This is all good and well, but the do notation does not allow us to manipulate the state, so we also need basic operations that do:

> get = Stateful $ \m -> (m, m)        -- get the current value of state
> update n = Stateful $ \m -> ((), n)  -- update state to n, give () as result

The Haskell value () is the empty tuple, the equivalent of void in C++ and Java.

With this we may write procedural program:

> a = run 42 (do x <- get        -- let x be current value of memory (42)
>                update (x+1)    -- update memory to x+1 (43)
>                y <- get        -- let y be current value of memory (43)
>                update 7        -- update memory to 7
>                return (x + y)) -- return x+y (42+43 = 85)
> -- a is (85, 7)

Let us conclude this short introduction by giving the official equations that return and >>= must satisfy in order to deserve the name "monad":

    return x >>= f            =   f x
    m >>= return              =   m
    m >>= (\x -> f x >>= g)   =   (m >>= f) >>= g

Further reading:

1. http://www.haskell.org/tutorial/monads.html
   "A Gentle Introduction to Haskell"
   Chapter 9: About Moands
 
2. http://www.haskell.org/all_about_monads/html/index.html
   "All about Monads"
   A comprehensive guide to the theory and practice of monadic programming in Haskell

3. http://blog.sigfpe.com/
   "A neighborhood of infinity"
   Dan Piponi's blog has many clever and mind boggling examples of monads


