import json
import random
import statistics
import subprocess


def sample(version, exponent):
    binary = f"/tmp/hyperreal-power-{version}/target/release/hyperreal-power-audit"
    result = subprocess.run(
        ["taskset", "-c", "6", binary, "65537", "65536", str(exponent), "10000", "true", "1"],
        check=True, capture_output=True, text=True,
    )
    return float(result.stdout.strip())


results = {}
for exponent in [-4001, 4001]:
    pairs = []
    for _ in range(41):
        a = sample("baseline", exponent)
        b = sample("fixed", exponent)
        b2 = sample("fixed", exponent)
        a2 = sample("baseline", exponent)
        pairs.append([(a + a2) / 2, (b + b2) / 2])
    ratios = [b / a for a, b in pairs]
    rng = random.Random(20260904)
    estimates = sorted(statistics.median(rng.choices(ratios, k=41)) for _ in range(5000))
    results[str(exponent)] = {
        "baseline_median_ns": statistics.median(a for a, _ in pairs),
        "fixed_median_ns": statistics.median(b for _, b in pairs),
        "ratio_median": statistics.median(ratios),
        "ratio_bootstrap_95pct": [estimates[125], estimates[4874]],
        "pairs": pairs,
    }
    print(exponent, {k: v for k, v in results[str(exponent)].items() if k != "pairs"}, flush=True)
with open("/tmp/hyperreal-power-recheck-results.json", "w") as output:
    json.dump(results, output, indent=2)
