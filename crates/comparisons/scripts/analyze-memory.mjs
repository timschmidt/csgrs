#!/usr/bin/env node

import fs from "node:fs";
import path from "node:path";

const input = path.resolve(process.argv[2] ?? "target/memory-sweep/results.tsv");
const format = process.argv[3] ?? "summary";

const lines = fs
  .readFileSync(input, "utf8")
  .trim()
  .split(/\r?\n/)
  .filter(Boolean);
if (lines.length < 2) throw new Error(`no measurements in ${input}`);

const columns = lines[0].split("\t");
const textColumns = new Set(["library", "workload", "mode"]);
const rows = lines.slice(1).map((line, rowIndex) => {
  const values = line.split("\t");
  if (values.length !== columns.length) {
    throw new Error(
      `row ${rowIndex + 2} has ${values.length} fields; expected ${columns.length}`,
    );
  }
  return Object.fromEntries(
    columns.map((column, index) => [
      column,
      textColumns.has(column) ? values[index] : Number(values[index]),
    ]),
  );
});

function key(row) {
  return `${row.workload}\0${row.mode}\0${row.cases}`;
}

const pairs = new Map();
for (const row of rows) {
  const entry = pairs.get(key(row)) ?? {};
  if (row.library === "exactCorelib") entry.exact = row;
  else if (row.library === "Hyper") entry.hyper = row;
  else throw new Error(`unknown library ${row.library}`);
  pairs.set(key(row), entry);
}
for (const [pairKey, pair] of pairs) {
  if (!pair.exact || !pair.hyper) throw new Error(`incomplete pair ${pairKey}`);
}

function operationTraffic(row) {
  return row.operation_allocated_bytes + row.operation_reallocated_bytes;
}

function hwmDelta(row) {
  return Math.max(0, row.final_vm_hwm_bytes - row.baseline_vm_hwm_bytes);
}

function rssDelta(row) {
  return row.operation_rss_bytes - row.baseline_rss_bytes;
}

function pssDelta(row) {
  return row.operation_pss_bytes - row.baseline_pss_bytes;
}

function ratio(exact, hyper) {
  if (hyper === 0) return exact === 0 ? 1 : Number.POSITIVE_INFINITY;
  return exact / hyper;
}

function leastSquaresSlope(selected, metric) {
  if (selected.length < 2) return Number.NaN;
  const meanX = selected.reduce((sum, row) => sum + row.cases, 0) / selected.length;
  const meanY = selected.reduce((sum, row) => sum + metric(row), 0) / selected.length;
  const numerator = selected.reduce(
    (sum, row) => sum + (row.cases - meanX) * (metric(row) - meanY),
    0,
  );
  const denominator = selected.reduce(
    (sum, row) => sum + (row.cases - meanX) ** 2,
    0,
  );
  return numerator / denominator;
}

function bytes(value) {
  const sign = value < 0 ? "−" : "";
  const absolute = Math.abs(value);
  if (absolute >= 1024 ** 2) return `${sign}${(absolute / 1024 ** 2).toFixed(2)} MiB`;
  if (absolute >= 1024) return `${sign}${(absolute / 1024).toFixed(2)} KiB`;
  return `${sign}${absolute.toFixed(0)} B`;
}

function bytesPer(value) {
  if (!Number.isFinite(value)) return "n/a";
  return `${value.toFixed(1)} B/case`;
}

function bytesPerResult(value) {
  if (!Number.isFinite(value)) return "n/a";
  return `${value.toFixed(1)} B/result`;
}

function factor(value) {
  if (!Number.isFinite(value)) return "∞";
  return `${value.toFixed(2)}×`;
}

function largestRows(workload, mode, library) {
  return rows
    .filter(
      (row) => row.workload === workload && row.mode === mode && row.library === library,
    )
    .sort((left, right) => left.cases - right.cases);
}

const scalingWorkloadNames = ["line-point", "triangle-point", "triangle-pair"];
const workloads = scalingWorkloadNames.filter((workload) =>
  rows.some((row) => row.workload === workload),
);
const modes = ["streaming", "materialized"].filter((mode) =>
  rows.some((row) => row.mode === mode),
);

const slopes = [];
for (const workload of workloads) {
  for (const library of ["exactCorelib", "Hyper"]) {
    const selected = largestRows(workload, "streaming", library);
    if (selected.length === 0) continue;
    slopes.push({
      workload,
      library,
      fixtureBytesPerCase: leastSquaresSlope(selected, (row) => row.fixture_live_bytes),
      residualBytesPerCase: leastSquaresSlope(selected, (row) => row.residual_live_bytes),
      residualBytesPerWorkItem:
        leastSquaresSlope(selected, (row) => row.residual_live_bytes) /
        selected.at(-1).repetitions,
      largest: selected.at(-1),
    });
  }
}

const comparableOperations = [...pairs.values()]
  .filter(({ exact }) => !scalingWorkloadNames.includes(exact.workload))
  .sort((left, right) => left.exact.workload.localeCompare(right.exact.workload))
  .map(({ exact, hyper }) => ({
    operation: exact.workload,
    repetitions: exact.repetitions,
    fixture: {
      exactBytes: exact.fixture_live_bytes,
      hyperBytes: hyper.fixture_live_bytes,
      exactOverHyper: ratio(exact.fixture_live_bytes, hyper.fixture_live_bytes),
    },
    operationTraffic: {
      exactBytesPerIteration: operationTraffic(exact) / exact.work_items,
      hyperBytesPerIteration: operationTraffic(hyper) / hyper.work_items,
      exactOverHyper: ratio(operationTraffic(exact), operationTraffic(hyper)),
    },
    operationGrowth: {
      exactBytes: exact.operation_live_delta_bytes,
      hyperBytes: hyper.operation_live_delta_bytes,
    },
    ephemeralPeak: {
      exactBytes: exact.operation_ephemeral_peak_bytes,
      hyperBytes: hyper.operation_ephemeral_peak_bytes,
    },
    residual: {
      exactBytes: exact.residual_live_bytes,
      hyperBytes: hyper.residual_live_bytes,
    },
  }));

function summaryObject() {
  const largestStreaming = workloads.map((workload) => {
    const exact = largestRows(workload, "streaming", "exactCorelib").at(-1);
    const hyper = largestRows(workload, "streaming", "Hyper").at(-1);
    return {
      workload,
      cases: exact.cases,
      fixture: {
        exactBytes: exact.fixture_live_bytes,
        hyperBytes: hyper.fixture_live_bytes,
        exactOverHyper: ratio(exact.fixture_live_bytes, hyper.fixture_live_bytes),
      },
      operationTraffic: {
        exactBytesPerItem: operationTraffic(exact) / exact.work_items,
        hyperBytesPerItem: operationTraffic(hyper) / hyper.work_items,
        exactOverHyper: ratio(operationTraffic(exact), operationTraffic(hyper)),
      },
      operationGrowth: {
        exactBytes: exact.operation_live_delta_bytes,
        hyperBytes: hyper.operation_live_delta_bytes,
        exactOverHyper: ratio(
          exact.operation_live_delta_bytes,
          hyper.operation_live_delta_bytes,
        ),
      },
      ephemeralPeak: {
        exactBytes: exact.operation_ephemeral_peak_bytes,
        hyperBytes: hyper.operation_ephemeral_peak_bytes,
      },
      residual: {
        exactBytes: exact.residual_live_bytes,
        hyperBytes: hyper.residual_live_bytes,
        exactOverHyper: ratio(exact.residual_live_bytes, hyper.residual_live_bytes),
      },
    };
  });
  const materializedOutputs = workloads.flatMap((workload) => {
    const exactStreaming = largestRows(workload, "streaming", "exactCorelib").at(-1);
    const hyperStreaming = largestRows(workload, "streaming", "Hyper").at(-1);
    const exactMaterialized = largestRows(workload, "materialized", "exactCorelib").at(-1);
    const hyperMaterialized = largestRows(workload, "materialized", "Hyper").at(-1);
    if (!exactStreaming || !hyperStreaming || !exactMaterialized || !hyperMaterialized) {
      return [];
    }
    return [
      {
        workload,
        cases: exactStreaming.cases,
        exactBytesPerResult:
          (exactMaterialized.operation_ephemeral_peak_bytes -
            exactStreaming.operation_ephemeral_peak_bytes) /
          exactStreaming.work_items,
        hyperBytesPerResult:
          (hyperMaterialized.operation_ephemeral_peak_bytes -
            hyperStreaming.operation_ephemeral_peak_bytes) /
          hyperStreaming.work_items,
      },
    ];
  });
  return {
    input,
    generatedAt: new Date().toISOString(),
    measurementRows: rows.length,
    pairedConfigurations: pairs.size,
    repetitions: [...new Set(rows.map((row) => row.repetitions))],
    largestStreaming,
    materializedOutputs,
    comparableOperationCount: comparableOperations.length,
    comparableOperations,
    slopes,
    rows,
  };
}

function markdown() {
  let output = "## Generated memory-sweep analysis\n\n";
  output += `Input: \`${input}\`  \n`;
  output += `${rows.length} worker measurements / ${pairs.size} paired configurations.\n\n`;
  if (workloads.length > 0) {
    output += "### Largest streaming fixtures\n\n";
    output +=
      "| Workload | Cases | Retained exact / Hyper | Execution allocation traffic per item, exact / Hyper | Post-operation growth exact / Hyper | Ephemeral peak exact / Hyper | Residual exact / Hyper |\n";
    output += "|---|---:|---:|---:|---:|---:|---:|\n";
    for (const item of summaryObject().largestStreaming) {
      output += `| ${item.workload} | ${item.cases} | ${bytes(item.fixture.exactBytes)} / ${bytes(item.fixture.hyperBytes)} (${factor(item.fixture.exactOverHyper)}) | ${bytes(item.operationTraffic.exactBytesPerItem)} / ${bytes(item.operationTraffic.hyperBytesPerItem)} (${factor(item.operationTraffic.exactOverHyper)}) | ${bytes(item.operationGrowth.exactBytes)} / ${bytes(item.operationGrowth.hyperBytes)} | ${bytes(item.ephemeralPeak.exactBytes)} / ${bytes(item.ephemeralPeak.hyperBytes)} | ${bytes(item.residual.exactBytes)} / ${bytes(item.residual.hyperBytes)} |\n`;
    }
    output += "\n### Fitted retained/residual scaling\n\n";
    output +=
      "| Workload | Library | Retained slope | Residual slope per fixture case | Residual per processed item |\n";
    output += "|---|---|---:|---:|---:|\n";
    for (const item of slopes) {
      output += `| ${item.workload} | ${item.library} | ${bytesPer(item.fixtureBytesPerCase)} | ${bytesPer(item.residualBytesPerCase)} | ${bytesPerResult(item.residualBytesPerWorkItem)} |\n`;
    }
    output += "\n### Materialized native-output footprint\n\n";
    output += "| Workload | Cases | exactCore | Hyper |\n";
    output += "|---|---:|---:|---:|\n";
    for (const item of summaryObject().materializedOutputs) {
      output += `| ${item.workload} | ${item.cases} | ${bytesPerResult(item.exactBytesPerResult)} | ${bytesPerResult(item.hyperBytesPerResult)} |\n`;
    }
  }
  if (comparableOperations.length > 0) {
    output += `${workloads.length > 0 ? "\n" : ""}### Comparable-operation coverage\n\n`;
    output += `${comparableOperations.length} concrete retained operation${comparableOperations.length === 1 ? " was" : "s were"} measured.\n\n`;
    output +=
      "| Operation | Retained exact / Hyper | Traffic per iteration exact / Hyper | Post-op growth exact / Hyper | Ephemeral peak exact / Hyper | Residual exact / Hyper |\n";
    output += "|---|---:|---:|---:|---:|---:|\n";
    for (const item of comparableOperations) {
      output += `| ${item.operation} | ${bytes(item.fixture.exactBytes)} / ${bytes(item.fixture.hyperBytes)} | ${bytes(item.operationTraffic.exactBytesPerIteration)} / ${bytes(item.operationTraffic.hyperBytesPerIteration)} | ${bytes(item.operationGrowth.exactBytes)} / ${bytes(item.operationGrowth.hyperBytes)} | ${bytes(item.ephemeralPeak.exactBytes)} / ${bytes(item.ephemeralPeak.hyperBytes)} | ${bytes(item.residual.exactBytes)} / ${bytes(item.residual.hyperBytes)} |\n`;
    }
  }
  output += "\n### Complete paired table\n\n";
  output +=
    "| Workload / operation | Mode | Cases | Retained exact / Hyper | Operation traffic exact / Hyper | Post-op growth exact / Hyper | Ephemeral peak exact / Hyper | Residual exact / Hyper | RSS Δ exact / Hyper | HWM Δ exact / Hyper |\n";
  output += "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|\n";
  const orderedPairs = [...pairs.values()].sort((left, right) => {
    const workload = workloads.indexOf(left.exact.workload) - workloads.indexOf(right.exact.workload);
    if (workload !== 0) return workload;
    const mode = modes.indexOf(left.exact.mode) - modes.indexOf(right.exact.mode);
    if (mode !== 0) return mode;
    return left.exact.cases - right.exact.cases;
  });
  for (const { exact, hyper } of orderedPairs) {
    output += `| ${exact.workload} | ${exact.mode} | ${exact.cases} | ${bytes(exact.fixture_live_bytes)} / ${bytes(hyper.fixture_live_bytes)} | ${bytes(operationTraffic(exact))} / ${bytes(operationTraffic(hyper))} | ${bytes(exact.operation_live_delta_bytes)} / ${bytes(hyper.operation_live_delta_bytes)} | ${bytes(exact.operation_ephemeral_peak_bytes)} / ${bytes(hyper.operation_ephemeral_peak_bytes)} | ${bytes(exact.residual_live_bytes)} / ${bytes(hyper.residual_live_bytes)} | ${bytes(rssDelta(exact))} / ${bytes(rssDelta(hyper))} | ${bytes(hwmDelta(exact))} / ${bytes(hwmDelta(hyper))} |\n`;
  }
  output += "\nRSS Δ is current rollup RSS after execution minus the pre-fixture baseline; HWM Δ is the process high-water delta.\n";
  return output;
}

if (format === "json") {
  process.stdout.write(`${JSON.stringify(summaryObject(), null, 2)}\n`);
} else if (format === "markdown") {
  process.stdout.write(markdown());
} else if (format === "summary") {
  if (comparableOperations.length > 0) {
    process.stdout.write(
      `Comparable operation coverage: ${comparableOperations.length} retained pairs.\n`,
    );
  }
  if (workloads.length > 0) {
    process.stdout.write("Largest streaming fixture results:\n");
    for (const item of summaryObject().largestStreaming) {
      process.stdout.write(
        `  ${item.workload} n=${item.cases}: retained E/H ${factor(item.fixture.exactOverHyper)}, ` +
          `traffic E/H ${factor(item.operationTraffic.exactOverHyper)}, ` +
          `residual ${bytes(item.residual.exactBytes)} / ${bytes(item.residual.hyperBytes)}\n`,
      );
    }
    process.stdout.write("Fitted retained slopes:\n");
    for (const item of slopes) {
      process.stdout.write(
        `  ${item.workload} ${item.library}: ${bytesPer(item.fixtureBytesPerCase)} retained, ` +
          `${bytesPer(item.residualBytesPerCase)} residual\n`,
      );
    }
  }
} else {
  throw new Error(`unknown format ${format}; expected summary, json, or markdown`);
}
