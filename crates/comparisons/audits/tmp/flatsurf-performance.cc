#include <benchmark/benchmark.h>
#include <exact-real/arb.hpp>
#include <exact-real/element.hpp>
#include <exact-real/module.hpp>
#include <exact-real/rational_field.hpp>
#include <exact-real/real_number.hpp>
#include <exact-real/seed.hpp>

#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using Element = exactreal::Element<exactreal::RationalField>;
using Module = exactreal::Module<exactreal::RationalField>;
using exactreal::RealNumber;

Element sparse_one(long rank) {
  Module::Basis basis{RealNumber::rational(1)};
  for (long i = 1; i < rank; ++i)
    basis.push_back(RealNumber::random(exactreal::Seed(10'000 + i)));
  const auto module = Module::make(basis);
  return module->one();
}

void sparse_square(benchmark::State& state) {
  const auto input = sparse_one(state.range(0));
  for (auto _ : state) {
    auto result = input * input;
    benchmark::DoNotOptimize(result);
    benchmark::ClobberMemory();
  }
}

void trimmed_square(benchmark::State& state) {
  auto input = sparse_one(state.range(0));
  input.simplify();
  for (auto _ : state) {
    auto result = input * input;
    benchmark::DoNotOptimize(result);
    benchmark::ClobberMemory();
  }
}

void sparse_arb(benchmark::State& state) {
  const auto input = sparse_one(state.range(0));
  for (auto _ : state) {
    auto result = input.arb(128);
    benchmark::DoNotOptimize(result);
  }
}

void trimmed_arb(benchmark::State& state) {
  auto input = sparse_one(state.range(0));
  input.simplify();
  for (auto _ : state) {
    auto result = input.arb(128);
    benchmark::DoNotOptimize(result);
  }
}

void cached_random_arf(benchmark::State& state) {
  const auto input = RealNumber::random(exactreal::Seed(200'000));
  benchmark::DoNotOptimize(input->arf(state.range(0)));
  for (auto _ : state) {
    auto result = input->arf(state.range(0));
    benchmark::DoNotOptimize(result);
  }
}

void create_and_refine_random(benchmark::State& state) {
  unsigned seed = 300'000;
  for (auto _ : state) {
    const auto input = RealNumber::random(exactreal::Seed(seed++));
    auto result = input->arf(state.range(0));
    benchmark::DoNotOptimize(result);
    benchmark::ClobberMemory();
  }
}

BENCHMARK(sparse_square)->Arg(1)->Arg(4)->Arg(16)->Arg(32)->Arg(64);
BENCHMARK(trimmed_square)->Arg(1)->Arg(4)->Arg(16)->Arg(32)->Arg(64);
BENCHMARK(sparse_arb)->Arg(1)->Arg(4)->Arg(16)->Arg(32)->Arg(64);
BENCHMARK(trimmed_arb)->Arg(1)->Arg(4)->Arg(16)->Arg(32)->Arg(64);
BENCHMARK(cached_random_arf)->Arg(64)->Arg(256)->Arg(1024);
BENCHMARK(create_and_refine_random)->Arg(64)->Arg(256)->Arg(1024);

void report_shapes() {
  std::cout << "sizeof_element=" << sizeof(Element)
            << " sizeof_module=" << sizeof(Module)
            << " sizeof_coefficient=" << sizeof(mpq_class)
            << " sizeof_arb=" << sizeof(exactreal::Arb) << '\n';
  for (const long rank : {1, 4, 16, 32, 64}) {
    const auto input = sparse_one(rank);
    auto square = input * input;
    if (!(input == 1) || !(square == 1))
      throw std::runtime_error("rational value changed");
    const auto copy = square;
    const auto hash_before = std::hash<Element>{}(square);
    const auto output_rank = square.module()->rank();
    square.simplify();
    const auto hash_after = std::hash<Element>{}(square);
    if (!(square == copy) || hash_before != hash_after || copy.module()->rank() != output_rank)
      throw std::runtime_error("value, hash, or copied module changed");
    std::cout << "input_rank=" << rank << " product_rank=" << output_rank
              << " coefficient_storage_bytes=" << output_rank * sizeof(mpq_class)
              << " simplified_rank=" << square.module()->rank()
              << " hash_stable=1 copy_preserved=1\n";
  }
}

int main(int argc, char** argv) {
  if (argc == 2 && std::string(argv[1]) == "--audit-shapes") {
    report_shapes();
    return 0;
  }
  benchmark::Initialize(&argc, argv);
  if (benchmark::ReportUnrecognizedArguments(argc, argv))
    return 1;
  benchmark::RunSpecifiedBenchmarks();
  benchmark::Shutdown();
}
