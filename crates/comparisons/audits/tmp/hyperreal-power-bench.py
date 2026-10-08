import json
import random
import statistics
import subprocess

BINARIES = {
    name: f"/tmp/hyperreal-power-{name}/target/release/hyperreal-power-audit"
    for name in ["baseline", "fixed"]
}


def sample(name, arguments):
    result = subprocess.run(
        ["taskset", "-c", "6", BINARIES[name], *map(str, arguments), "1"],
        check=True, text=True, capture_output=True,
    )
    return float(result.stdout.strip())


def ratio_interval(ratios):
    rng = random.Random(20260904)
    bootstrap = sorted(
        statistics.median(rng.choices(ratios, k=len(ratios)))
        for _ in range(5000)
    )
    return [bootstrap[125], bootstrap[4874]]


results = {}
for case, arguments in {
    "small_exact_build": [7, 5, 17, 20000, "false"],
    "large_positive_build": [65537, 65536, 4001, 4000, "false"],
    "large_positive_evaluate": [65537, 65536, 4001, 1000, "true"],
    "large_positive_reciprocal_evaluate": [65537, 65536, -4001, 1000, "true"],
}.items():
    sample("baseline", arguments)
    sample("fixed", arguments)
    paired = []
    for _ in range(21):
        a1 = sample("baseline", arguments)
        b1 = sample("fixed", arguments)
        b2 = sample("fixed", arguments)
        a2 = sample("baseline", arguments)
        paired.append({"baseline": (a1 + a2) / 2, "fixed": (b1 + b2) / 2})
    ratios = [row["fixed"] / row["baseline"] for row in paired]
    results[case] = {
        "baseline_median_ns": statistics.median(row["baseline"] for row in paired),
        "fixed_median_ns": statistics.median(row["fixed"] for row in paired),
        "paired_ratio_median": statistics.median(ratios),
        "paired_ratio_bootstrap_95pct": ratio_interval(ratios),
        "pairs": paired,
    }
    print(case, json.dumps({k: v for k, v in results[case].items() if k != "pairs"}), flush=True)

for exponent in [-4001, -4000, 4000, 4001]:
    arguments = [-65537, 65536, exponent, 1000, "true"]
    sample("fixed", arguments)
    samples = [sample("fixed", arguments) for _ in range(21)]
    results[f"fixed_negative_{exponent}"] = {
        "median_ns": statistics.median(samples), "samples_ns": samples,
    }
    print(f"fixed_negative_{exponent}", statistics.median(samples), flush=True)

with open("/tmp/hyperreal-power-bench-results.json", "w") as output:
    json.dump(results, output, indent=2)
