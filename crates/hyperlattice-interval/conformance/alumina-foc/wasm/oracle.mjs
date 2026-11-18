// All interval endpoints are signed integers in units of one Q2.30 grid step.
const scale = 1n << 30n;
const one = [scale, scale];
const zero = [0n, 0n];
const negativeOne = [-scale, -scale];

function floorRatio(n, d) {
  return n / d - (n % d < 0n ? 1n : 0n);
}

function hull(values, divisor = 1n) {
  const low = values.reduce((x, y) => x < y ? x : y);
  const high = values.reduce((x, y) => x > y ? x : y);
  return [floorRatio(low, divisor), -floorRatio(-high, divisor)];
}

const add = ([a, b], [c, d]) => [a + c, b + d];
const sub = ([a, b], [c, d]) => [a - d, b - c];
const product = (a, b) => hull(a.flatMap(x => b.map(y => x * y)), scale);
const around = x => [x - 1n, x + 1n];

function rotation(index) {
  return [
    [zero, one], [one, zero], [negativeOne, zero], [zero, negativeOne],
    [hull([3n * scale], 5n), hull([4n * scale], 5n)],
  ][index % 5];
}

// Both Clarke coefficients are irrational and lie strictly between 0 and 1.
// Bracket each root on the Q30 grid using integer square comparisons.
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

function clarke([a, b, c]) {
  const numerators = a.flatMap(x => b.flatMap(y => c.map(z => 2n * x - y - z)));
  return [hull(numerators, 3n), product(sub(b, c), inverseRootThree)];
}

function inverseClarke([alpha, beta]) {
  const half = hull(alpha.map(x => -x), 2n);
  const betaTerm = product(beta, halfRootThree);
  return [alpha, add(half, betaTerm), sub(half, betaTerm)];
}

function park([a, b], [sin, cos]) {
  return [add(product(a, cos), product(b, sin)), sub(product(b, cos), product(a, sin))];
}

function inversePark([d, q], [sin, cos]) {
  return [sub(product(d, cos), product(q, sin)), add(product(d, sin), product(q, cos))];
}

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

export function parkRange(a, b, index) {
  const [sin, cos] = rotation(index);
  const ac = product([a, a], cos);
  const as = product([a, a], sin);
  const bc = product([b, b], cos);
  const bs = product([b, b], sin);
  return Number([ac, as, bc, bs, add(ac, bs), sub(bc, as)].some(
    ([lo, hi]) => lo < -(1n << 31n) || hi >= (1n << 31n)));
}
