#include <benchmark/benchmark.h>
#include <optional>
#include <vector>

#include "percepto/core/ray.h"
#include "percepto/core/vec3.h"
#include "percepto/math/intersection/moller_trumbore.h"

static const percepto::core::Vec3 v0(0.0, 0.0, 1000.0);
static const percepto::core::Vec3 v1(10.0, 0.0, 1000.0);
static const percepto::core::Vec3 v2(0.0, 10.0, 1000.0);

static const percepto::core::Ray hit_ray(percepto::core::Vec3(0.0, 0.0, 0.0),
                                         percepto::core::Vec3(0.0, 0.0, 1.0));

static const percepto::core::Ray miss_ray(percepto::core::Vec3(0.0, 0.0, 0.0),
                                          percepto::core::Vec3(0.0, 0.0, -1.0));

static void BM_MollerTrumbore_Hit(benchmark::State& state)
{
  for (auto _ : state)
  {
    benchmark::DoNotOptimize(percepto::math::intersection::moller_trumbore(v0, v1, v2, hit_ray));
  }
}

static void BM_MollerTrumbore_Miss(benchmark::State& state)
{
  for (auto _ : state)
  {
    benchmark::DoNotOptimize(percepto::math::intersection::moller_trumbore(v0, v1, v2, miss_ray));
  }
}

BENCHMARK(BM_MollerTrumbore_Hit);
BENCHMARK(BM_MollerTrumbore_Miss);

BENCHMARK_MAIN();
