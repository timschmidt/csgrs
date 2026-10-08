#!/usr/bin/env node

import fs from "node:fs";
import path from "node:path";

const criterionRoot = path.resolve(process.argv[2] ?? "target/criterion");
const format = process.argv[3] ?? "summary";
const tier = process.argv[4] ?? "adapter";

if (!new Set(["adapter", "retained"]).has(tier)) {
  throw new Error(`unknown benchmark tier ${tier}; expected adapter or retained`);
}

const exactImplementation =
  tier === "retained" ? "exactCorelib-retained" : "exactCorelib";

function walk(directory, results = []) {
  for (const entry of fs.readdirSync(directory, { withFileTypes: true })) {
    const candidate = path.join(directory, entry.name);
    if (entry.isDirectory()) {
      walk(candidate, results);
    } else if (entry.name === "benchmark.json" && path.basename(directory) === "new") {
      results.push(candidate);
    }
  }
  return results;
}

function readJson(file) {
  return JSON.parse(fs.readFileSync(file, "utf8"));
}

function family(operation) {
  if (operation.startsWith("rational.")) return "rational";
  if (operation.startsWith("real.")) return "real";
  if (operation.startsWith("complex.")) return "complex";
  if (/^vector[234]\./.test(operation)) return "vectors";
  if (/^matrix[34]\./.test(operation)) return "matrices";
  if (operation.startsWith("geometry2.")) return "geometry-2d";
  if (operation.startsWith("geometry3.")) return "geometry-3d";
  if (operation.startsWith("polynomial.") || operation.startsWith("bivariate.")) {
    return "polynomials";
  }
  if (operation.startsWith("triangulation.")) return "triangulation";
  if (operation.startsWith("curve.")) return "curves";
  if (operation.startsWith("mesh.")) return "meshes";
  if (operation.startsWith("path.")) return "paths";
  return "other";
}

function semanticStatus(operation) {
  if (/^matrix[34]\.determinant$/.test(operation)) return "divergent-family";
  if (operation.startsWith("geometry2.line_relation_")) return "divergent-family";
  if (
    operation === "geometry2.circle_line" ||
    operation === "geometry2.circle_segment"
  ) {
    return "divergent-family";
  }
  if (operation === "geometry2.circle_circle_distance") return "adapted";
  if (operation.startsWith("geometry3.plane_relation_")) return "divergent-family";
  if (operation.startsWith("geometry3.triangle_relation_")) return "divergent-family";
  if (operation === "polynomial.resultant") return "adapted";
  if (operation === "polynomial.discriminant") return "divergent-family";
  if (operation === "polynomial.root_count") return "divergent-family";
  if (operation.startsWith("triangulation.")) return "adapted";
  if (
    operation === "curve.supporting_line_circle" ||
    operation === "curve.circle_circle_relation"
  ) {
    return "adapted";
  }
  if (operation.startsWith("mesh.triangle_")) return "divergent-family";
  return "matched";
}

function quantile(sorted, fraction) {
  if (sorted.length === 0) return Number.NaN;
  const position = (sorted.length - 1) * fraction;
  const lower = Math.floor(position);
  const upper = Math.ceil(position);
  if (lower === upper) return sorted[lower];
  return sorted[lower] + (sorted[upper] - sorted[lower]) * (position - lower);
}

function geometricMean(values) {
  return Math.exp(values.reduce((sum, value) => sum + Math.log(value), 0) / values.length);
}

function normalizedSamples(directory) {
  const sample = readJson(path.join(directory, "sample.json"));
  return sample.times.map((time, index) => time / sample.iters[index]);
}

function outlierCount(directory) {
  const [lowSevere, lowMild, highMild, highSevere] = readJson(
    path.join(directory, "tukey.json"),
  );
  return normalizedSamples(directory).filter(
    (value) =>
      value < lowSevere ||
      (value >= lowSevere && value < lowMild) ||
      (value > highMild && value <= highSevere) ||
      value > highSevere,
  ).length;
}

function implementation(fullId) {
  const separator = fullId.lastIndexOf("/");
  return {
    operation: fullId.slice(0, separator),
    implementation: fullId.slice(separator + 1),
  };
}

const measurements = new Map();
for (const benchmarkFile of walk(criterionRoot)) {
  const directory = path.dirname(benchmarkFile);
  const benchmark = readJson(benchmarkFile);
  const estimates = readJson(path.join(directory, "estimates.json"));
  const parsed = implementation(benchmark.full_id);
  const median = estimates.median;
  measurements.set(benchmark.full_id, {
    ...parsed,
    medianNs: median.point_estimate,
    lowerNs: median.confidence_interval.lower_bound,
    upperNs: median.confidence_interval.upper_bound,
    relativeCiWidth:
      (median.confidence_interval.upper_bound - median.confidence_interval.lower_bound) /
      median.point_estimate,
    outliers: outlierCount(directory),
    sampleCount: normalizedSamples(directory).length,
  });
}

const grouped = new Map();
for (const measurement of measurements.values()) {
  const entry = grouped.get(measurement.operation) ?? {};
  if (measurement.implementation === exactImplementation) {
    entry.exact = measurement;
  } else if (measurement.implementation.startsWith("hyper")) {
    entry.hyper = measurement;
  }
  grouped.set(measurement.operation, entry);
}

const rows = [...grouped.entries()]
  .map(([operation, pair]) => {
    if (!pair.exact || !pair.hyper) {
      throw new Error(`incomplete benchmark pair for ${operation}`);
    }
    const ratio = pair.exact.medianNs / pair.hyper.medianNs;
    let intervalWinner = "overlap";
    if (pair.exact.lowerNs > pair.hyper.upperNs) intervalWinner = "hyper";
    if (pair.exact.upperNs < pair.hyper.lowerNs) intervalWinner = "exactCore";
    return {
      operation,
      family: family(operation),
      semanticStatus: semanticStatus(operation),
      exactNs: pair.exact.medianNs,
      exactLowerNs: pair.exact.lowerNs,
      exactUpperNs: pair.exact.upperNs,
      hyperNs: pair.hyper.medianNs,
      hyperLowerNs: pair.hyper.lowerNs,
      hyperUpperNs: pair.hyper.upperNs,
      ratio,
      intervalWinner,
      exactRelativeCiWidth: pair.exact.relativeCiWidth,
      hyperRelativeCiWidth: pair.hyper.relativeCiWidth,
      exactOutliers: pair.exact.outliers,
      hyperOutliers: pair.hyper.outliers,
      exactSampleCount: pair.exact.sampleCount,
      hyperSampleCount: pair.hyper.sampleCount,
    };
  })
  .sort((left, right) => left.operation.localeCompare(right.operation));

function summarize(selected) {
  const ratios = selected.map((row) => row.ratio).sort((a, b) => a - b);
  const samples = selected.reduce(
    (sum, row) => sum + row.exactSampleCount + row.hyperSampleCount,
    0,
  );
  const outliers = selected.reduce(
    (sum, row) => sum + row.exactOutliers + row.hyperOutliers,
    0,
  );
  return {
    count: selected.length,
    exactWins: selected.filter((row) => row.intervalWinner === "exactCore").length,
    hyperWins: selected.filter((row) => row.intervalWinner === "hyper").length,
    overlaps: selected.filter((row) => row.intervalWinner === "overlap").length,
    geometricMeanRatio: geometricMean(ratios),
    medianRatio: quantile(ratios, 0.5),
    p10Ratio: quantile(ratios, 0.1),
    p90Ratio: quantile(ratios, 0.9),
    medianRelativeCiWidth: quantile(
      selected
        .flatMap((row) => [row.exactRelativeCiWidth, row.hyperRelativeCiWidth])
        .sort((a, b) => a - b),
      0.5,
    ),
    pairsWithAnyCiAbove10Pct: selected.filter(
      (row) => row.exactRelativeCiWidth > 0.1 || row.hyperRelativeCiWidth > 0.1,
    ).length,
    pairsWithAnyCiAbove25Pct: selected.filter(
      (row) => row.exactRelativeCiWidth > 0.25 || row.hyperRelativeCiWidth > 0.25,
    ).length,
    ratioBuckets: {
      exactAtLeast1_25x: selected.filter((row) => row.ratio < 0.8).length,
      within25Pct: selected.filter((row) => row.ratio >= 0.8 && row.ratio <= 1.25)
        .length,
      hyper1_25To2x: selected.filter((row) => row.ratio > 1.25 && row.ratio <= 2)
        .length,
      hyper2To10x: selected.filter((row) => row.ratio > 2 && row.ratio <= 10).length,
      hyper10To100x: selected.filter((row) => row.ratio > 10 && row.ratio <= 100)
        .length,
      hyperAbove100x: selected.filter((row) => row.ratio > 100).length,
    },
    samples,
    outliers,
    outlierRate: samples === 0 ? Number.NaN : outliers / samples,
  };
}

const familyNames = [...new Set(rows.map((row) => row.family))];
const families = Object.fromEntries(
  familyNames.map((name) => [name, summarize(rows.filter((row) => row.family === name))]),
);
const semanticStatuses = Object.fromEntries(
  [...new Set(rows.map((row) => row.semanticStatus))].map((status) => [
    status,
    summarize(rows.filter((row) => row.semanticStatus === status)),
  ]),
);
const result = {
  criterionRoot,
  generatedAt: new Date().toISOString(),
  tier,
  exactImplementation,
  overall: summarize(rows),
  families,
  semanticStatuses,
  fastestExactCore: [...rows].sort((a, b) => a.ratio - b.ratio).slice(0, 15),
  fastestHyper: [...rows].sort((a, b) => b.ratio - a.ratio).slice(0, 15),
  rows,
};

if (format === "json") {
  process.stdout.write(`${JSON.stringify(result, null, 2)}\n`);
} else if (format === "tsv") {
  process.stdout.write(
      "operation\tfamily\texact_median_ns\texact_ci_low_ns\texact_ci_high_ns\t" +
      "hyper_median_ns\thyper_ci_low_ns\thyper_ci_high_ns\texact_over_hyper_ratio\t" +
      "interval_winner\tsemantic_status\texact_outliers\thyper_outliers\n",
  );
  for (const row of rows) {
    process.stdout.write(
      [
        row.operation,
        row.family,
        row.exactNs,
        row.exactLowerNs,
        row.exactUpperNs,
        row.hyperNs,
        row.hyperLowerNs,
        row.hyperUpperNs,
        row.ratio,
        row.intervalWinner,
        row.semanticStatus,
        row.exactOutliers,
        row.hyperOutliers,
      ].join("\t") + "\n",
    );
  }
} else if (format === "markdown") {
  const duration = (nanoseconds) => {
    if (nanoseconds < 1_000) return `${nanoseconds.toFixed(2)} ns`;
    if (nanoseconds < 1_000_000) return `${(nanoseconds / 1_000).toFixed(2)} µs`;
    return `${(nanoseconds / 1_000_000).toFixed(2)} ms`;
  };
  const interval = (row, prefix) =>
    `${duration(row[`${prefix}Ns`])} ` +
    `[${duration(row[`${prefix}LowerNs`])}, ${duration(row[`${prefix}UpperNs`])}]`;
  const advantage = (row) =>
    row.ratio >= 1
      ? `Hyper ${row.ratio.toFixed(2)}×`
      : `exactCore ${(1 / row.ratio).toFixed(2)}×`;

  process.stdout.write(
    "| Operation | Semantic status | exactCore median [95% CI] | " +
      "Hyper median [95% CI] | Median advantage | CI result |\n",
  );
  process.stdout.write("|---|---|---:|---:|---:|---|\n");
  for (const row of rows) {
    process.stdout.write(
      `| \`${row.operation}\` | ${row.semanticStatus} | ${interval(row, "exact")} | ` +
        `${interval(row, "hyper")} | ${advantage(row)} | ${row.intervalWinner} |\n`,
    );
  }
} else {
  process.stdout.write(
    `${JSON.stringify({ overall: result.overall, families, semanticStatuses }, null, 2)}\n`,
  );
}
