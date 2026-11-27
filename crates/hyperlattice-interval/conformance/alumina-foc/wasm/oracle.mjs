// All interval endpoints are signed integers in units of one Q2.30 grid step.
// Host-only exact BigInt formulas are independent of both runtime providers.
// Transform helpers operate on ordered nonempty endpoint pairs and positive
// divisors; they do not enforce i32 range or Clarke balance policy. transforms
// supplies the bounded success corpus, while parkRange separately predicts Range.
// Returned arrays are treated as immutable; some helpers share existing arrays.
const scale = 1n << 30n;
const one = [scale, scale];
const zero = [0n, 0n];
const negativeOne = [-scale, -scale];

// Floor n/d for a positive BigInt denominator; JS division alone truncates toward
// zero. Internal callers satisfy positivity; zero/type errors are not normalized.
function floorRatio(n, d) {
  return n / d - (n % d < 0n ? 1n : 0n);
}

// Outward integer hull of a nonempty BigInt candidate list divided by a positive
// divisor. Empty lists fail in reduce; no machine-width restriction is applied.
function hull(values, divisor = 1n) {
  const low = values.reduce((x, y) => x < y ? x : y);
  const high = values.reduce((x, y) => x > y ? x : y);
  return [floorRatio(low, divisor), -floorRatio(-high, divisor)];
}

// Independent Cartesian sum and difference; no extra scaling or rounding.
const add = ([a, b], [c, d]) => [a + c, b + d];
const sub = ([a, b], [c, d]) => [a - d, b - c];
// Four raw product corners followed by one outward division by the Q30 scale.
const product = (a, b) => hull(a.flatMap(x => b.map(y => x * y)), scale);
// One raw unit of fixture radius, without a range check.
const around = x => [x - 1n, x + 1n];

// Cardinal points or outward 3/5,4/5 component brackets. The harness passes a
// nonnegative integer Number index; this helper does not perform Wasm i32 coercion.
function rotation(index) {
  return [
    [zero, one], [one, zero], [negativeOne, zero], [zero, negativeOne],
    [hull([3n * scale], 5n), hull([4n * scale], 5n)],
  ][index % 5];
}

// Both Clarke coefficients are irrational and lie strictly between 0 and 1.
// Bracket each root on the Q30 grid using integer square comparisons.
// The internal ratios satisfy 0<n<d. Bisection returns adjacent raw integers;
// for the two irrational ratios used here, the lower/upper inequalities are strict.
function rootBracket(n, d) {
  let low = 0n;
  let high = scale;
  while (high - low > 1n) {
    const mid = (low + high) / 2n;
    if (mid * mid * d <= n * scale * scale) low = mid;
    else high = mid;
  }
  return [low, high];
}
const inverseRootThree = rootBracket(1n, 3n);
const halfRootThree = rootBracket(3n, 4n);

// Corner hull of (2a-b-c)/3 and the bracketed (b-c)/sqrt(3), preserving wide
// numerators until their prescribed division. Balance is a fixture precondition.
function clarke([a, b, c]) {
  const numerators = a.flatMap(x => b.flatMap(y => c.map(z => 2n * x - y - z)));
  return [hull(numerators, 3n), product(sub(b, c), inverseRootThree)];
}

// Reuse alpha as phase a, then combine the outward negative half with the radical
// beta product. Reflecting the half hull gives the same bounds as rounding HALF*alpha
// then negating on these inputs; this helper does not model intermediate failures.
function inverseClarke([alpha, beta]) {
  const half = hull(alpha.map(x => -x), 2n);
  const betaTerm = product(beta, halfRootThree);
  return [alpha, add(half, betaTerm), sub(half, betaTerm)];
}

// Round each raw product before the d sum and q difference, using independent
// component intervals rather than imposing an exact joint unit-circle constraint.
function park([a, b], [sin, cos]) {
  return [add(product(a, cos), product(b, sin)), sub(product(b, cos), product(a, sin))];
}

// Stage d*cos-q*sin and d*sin+q*cos with the same per-product outward rounding.
function inversePark([d, q], [sin, cos]) {
  return [sub(product(d, cos), product(q, sin)), add(product(d, sin), product(q, cos))];
}

// Reconstruct the sample and all five tagged outputs, including the complete
// five-transform chain. Expects a nonnegative integer Number in the u32 domain;
// indices reduce modulo 33/five, without a general input validator. BigInt n/2
// truncates toward zero like the Rust fixture. Two-axis outputs have two lanes;
// consumers supply the zero padding used by the ABI and digest protocol.
export function transforms(index) {
  const n = BigInt(index % 33) - 16n;
  const unit = 1n << 24n;
  const a = n * unit;
  const b = -(n / 2n) * unit;
  const phases = [around(a), around(b), around(-a - b)];
  const stationary = [around(a), around((16n - (n < 0n ? -n : n)) * unit)];
  const rotating = [around(a), around((n + 8n) * unit / 2n)];
  const r = rotation(index);
  const forward = clarke(phases);
  const chain = clarke(inverseClarke(inversePark(park(forward, r), r)));
  return [
    [1, forward], [2, park(stationary, r)], [1, inversePark(rotating, r)],
    [3, inverseClarke(stationary)], [1, chain],
  ];
}

// Predict 0/1 range status for signed-i32 BigInt point inputs and a fixture rotation.
// Inspect all four rounded products and both combined bounds against the signed
// i32 range. This classifies whether any stage fails, not the native first-failure
// execution order; it does not return endpoints, typed causes or balance decisions.
export function parkRange(a, b, index) {
  const [sin, cos] = rotation(index);
  const ac = product([a, a], cos);
  const as = product([a, a], sin);
  const bc = product([b, b], cos);
  const bs = product([b, b], sin);
  return Number([ac, as, bc, bs, add(ac, bs), sub(bc, as)].some(
    ([lo, hi]) => lo < -(1n << 31n) || hi >= (1n << 31n)));
}
