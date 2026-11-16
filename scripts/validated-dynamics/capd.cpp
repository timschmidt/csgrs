// Separate-process CAPD adapter. Exact dyadic IVPs; no sampled trajectory is a tube.
#include "capd/capdlib.h"
#include <cmath>
#include <chrono>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <string>

static void bounds(const capd::interval& value) {
  if (!std::isfinite(value.leftBound()) || !std::isfinite(value.rightBound()))
    throw std::runtime_error("nonfinite enclosure");
  std::cout << "[\"" << std::hexfloat << value.leftBound() << "\",\""
            << value.rightBound() << "\"]";
}
static void vector(const capd::IVector& value) {
  std::cout << '[';
  for (unsigned i = 0; i < value.dimension(); ++i) {
    if (i) std::cout << ',';
    bounds(value[i]);
  }
  std::cout << ']';
}
struct Prepared {
  capd::IMap field;
  capd::IOdeSolver solver;
  capd::ITimeMap integrate;
  explicit Prepared(const std::string& name)
      : field(name == "constant" ? "var:x;fun:2;"
            : name == "exponential" ? "var:x;fun:x;"
            : name == "riccati" ? "var:x;fun:x*x;" : "var:x,y;fun:y,-x;"),
        solver(field, 12), integrate(solver) {
    solver.setMaxStep(1.0 / 16.0);
  }
};
struct AuditedCurve : capd::ITimeMap::SolutionCurve {
  AuditedCurve() : capd::ITimeMap::SolutionCurve(0) {}
  const Domain& timeDomains() const { return domain; }
  void checkStepCap() const {
    if (domain.size() != getNumberOfPieces() + 1)
      throw std::runtime_error("missing piece domain");
    for (unsigned i = 1; i < domain.size(); ++i) {
      const auto duration = domain[i] - domain[i - 1];
      if (duration.leftBound() <= 0 || duration.rightBound() > 1.0 / 16.0)
        throw std::runtime_error("invalid piece duration or exceeded step cap");
    }
  }
};
struct Outcome {
  capd::IVector initial, end, tube;
  AuditedCurve solution;
  Outcome(Prepared& prepared, bool oscillator, bool uncertain)
      : initial(oscillator ? 2 : 1), end(oscillator ? 2 : 1),
        tube(oscillator ? 2 : 1) {
    const double radius = uncertain ? 1.0 / 1024.0 : 0;
    initial[0] = capd::interval(1 - radius, 1 + radius);
    if (oscillator) initial[1] = capd::interval(-radius, radius);
    capd::C0Rect2Set set(initial, 0);
    // TimeMap replaces the solver's maxStep with the remaining horizon.
    // Exact checkpoints enforce the common cap without disabling adaptation.
    for (unsigned checkpoint = 1; checkpoint <= 4; ++checkpoint)
      end = prepared.integrate(capd::interval(checkpoint / 16.0), set, solution);
    if (solution.getLeftDomain() != 0 || solution.getRightDomain() != 0.25)
      throw std::runtime_error("incomplete horizon");
    tube = solution(capd::interval(0, 0.25));
    solution.checkStepCap();
  }
};
static void record(const std::string& name, bool uncertain, const Outcome& result) {
  std::cout << "{\"kind\":\"certificate\",\"solver\":\"capd\",\"case\":\"" << name
            << (uncertain ? "_box" : "_point")
            << "\",\"status\":\"certified\",\"encoding\":\"ieee_hex\",\"order\":12,\"initial\":";
  vector(result.initial);
  std::cout << ",\"start\":"; bounds(capd::interval(0));
  std::cout << ",\"end_time\":"; bounds(capd::interval(0.25));
  std::cout << ",\"end\":"; vector(result.end);
  std::cout << ",\"tube\":"; vector(result.tube);
  std::cout << ",\"time_domains\":[";
  bool first = true;
  for (const auto& time : result.solution.timeDomains()) {
    if (!first) std::cout << ',';
    bounds(time);
    first = false;
  }
  std::cout << "],\"pieces\":" << result.solution.getNumberOfPieces() << "}\n";
}
static bool same(const capd::IVector& a, const capd::IVector& b) {
  if (a.dimension() != b.dimension()) return false;
  for (unsigned i = 0; i < a.dimension(); ++i)
    if (a[i].leftBound() != b[i].leftBound() || a[i].rightBound() != b[i].rightBound())
      return false;
  return true;
}
static void benchmark(const std::string& name, bool uncertain, unsigned iterations, unsigned samples) {
  Prepared prepared(name);
  const bool oscillator = name == "oscillator";
  capd::IVector referenceEnd(oscillator ? 2 : 1), referenceTube(oscillator ? 2 : 1);
  {
    Outcome expected(prepared, oscillator, uncertain);
    record(name, uncertain, expected);
    referenceEnd = expected.end;
    referenceTube = expected.tube;
  }
  for (const std::string mode : {"reuse", "fresh"}) {
    auto run = [&] {
      if (mode == "fresh") {
        Prepared fresh(name);
        Outcome result(fresh, oscillator, uncertain);
        asm volatile("" : : "g"(&result) : "memory");
      } else {
        Outcome result(prepared, oscillator, uncertain);
        asm volatile("" : : "g"(&result) : "memory");
      }
    };
    for (unsigned i = 0; i < 8; ++i) run();
    for (unsigned sample = 0; sample < samples; ++sample) {
      auto start = std::chrono::steady_clock::now();
      for (unsigned i = 0; i < iterations; ++i) run();
      auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now() - start).count();
      std::cout << "{\"kind\":\"timing\",\"solver\":\"capd\",\"case\":\"" << name
                << (uncertain ? "_box" : "_point") << "\",\"mode\":\"" << mode
                << "\",\"sample\":" << sample << ",\"iterations\":" << iterations
                << ",\"total_ns\":" << elapsed << "}\n";
    }
    Outcome repeated(prepared, oscillator, uncertain);
    Prepared fresh(name);
    Outcome recreated(fresh, oscillator, uncertain);
    if (!same(repeated.end, referenceEnd) || !same(repeated.tube, referenceTube)
        || !same(recreated.end, referenceEnd) || !same(recreated.tube, referenceTube))
      throw std::runtime_error("history-dependent output: " + name);
  }
}
int main(int argc, char** argv) {
  try {
    if (argc != 1 && argc != 3) throw std::runtime_error("usage: [iterations samples]");
    const unsigned iterations = argc == 3 ? std::stoul(argv[1]) : 1;
    const unsigned samples = argc == 3 ? std::stoul(argv[2]) : 0;
    if (iterations < 1 || iterations > 100000 || samples > 100)
      throw std::runtime_error("invalid measurement bounds");
    for (const std::string name : {"constant", "exponential", "riccati", "oscillator"})
      for (bool uncertain : {false, true}) benchmark(name, uncertain, iterations, samples);
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
