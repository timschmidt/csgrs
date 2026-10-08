# Selected-root Sturm certificate

Let f(t,u) be the input fiber polynomial after multiplying all coefficients by one positive rational number to obtain integer coefficients. Its derivative g = ∂f/∂u uses the same scale. At a regular PRS step, consecutive fiber degrees differ by one. Therefore pseudo-division gives

    prem(A, B) = lc(B)^2 * rem(A, B).

The initial remainder is prem(f,g). Each subsequent Brown remainder divides a pseudo-remainder by the square of the preceding leading coefficient polynomial. The implementation accepts each division only after exact division in Z[t]; a failed division or a nonconsecutive degree returns to the existing field algorithm.

For the selected t = alpha, a separate STRICT field replays the retained root evidence and every nonzero row's leading coefficient. Consequently every multiplier and divisor used before termination is a nonzero square at alpha, hence positive. Applying row signs +,+,-,-,+,+,-,-,... (including f as the first row) produces the ordinary Sturm sign sequence up to positive row scales. The original input polynomial is retained as the first row; the positive initial rational scale does not affect its sign or roots.

An entirely zero specialized row certifies an exact zero remainder after the already-proved nonzero multipliers. It ends the chain. A specialized row with a nonzero lower term but a vanishing leading coefficient cannot use this regular certificate and falls back to the original field construction. No irreducibility assumption is made about the retained defining polynomial. No assertion about rational payloads is made for arbitrary exact Real coefficients: those remain on the general field path.

The exact fixture independently checked in contact-fiber-20260923-independent-count.json selects one root of a degree-34 factor inside its degree-41 defining presentation. All eight retained Sturm rows have certified nonzero leading coefficients; the final constant row vanishes on that selected factor. The signed boundary variations are 4 at u=0 and 3 at u=1, proving one distinct contact and no endpoint contact. The certificate was computed with independent Sympy polynomial arithmetic and Fraction interval evaluation.
