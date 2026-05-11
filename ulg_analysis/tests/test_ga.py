import numpy as np
import pytest
from pid_optimizer.ga import GeneticAlgorithm
from pid_optimizer.gains import GAIN_BOUNDS

# Trivial fitness: minimize sum of all gains (lower = better)
def mock_fitness(gains_array: np.ndarray) -> float:
    return float(np.sum(gains_array))

def test_tournament_selects_better():
    ga = GeneticAlgorithm(GAIN_BOUNDS, fitness_fn=mock_fitness, pop_size=10, seed=42)
    ga.initialize()
    pop = ga.population
    scores = np.array([mock_fitness(ind) for ind in pop])
    winner_idx = ga._tournament(scores, tournament_size=3)
    winner_score = scores[winner_idx]
    # Winner should be at or below median score
    assert winner_score <= np.median(scores)

def test_crossover_genes_from_parents():
    bounds = [(0.0, 1.0)] * 4
    ga = GeneticAlgorithm(bounds, fitness_fn=lambda _: 0.0, pop_size=4, tournament_size=2, seed=0)
    p1 = np.array([0.1, 0.2, 0.3, 0.4])
    p2 = np.array([0.9, 0.8, 0.7, 0.6])
    child = ga._crossover(p1, p2, prob=1.0)
    for i, v in enumerate(child):
        assert v == p1[i] or v == p2[i], f"Gene {i}={v} not from either parent"

def test_mutation_stays_in_bounds():
    bounds = [(0.0, 1.0)] * 6
    ga = GeneticAlgorithm(bounds, fitness_fn=lambda _: 0.0, pop_size=4, tournament_size=2, seed=1)
    ind = np.array([0.5] * 6)
    for _ in range(100):
        mutated = ga._mutate(ind, prob=1.0)
        lo = np.array([b[0] for b in bounds])
        hi = np.array([b[1] for b in bounds])
        assert np.all(mutated >= lo) and np.all(mutated <= hi)

def test_ga_improves_fitness():
    ga = GeneticAlgorithm(GAIN_BOUNDS, fitness_fn=mock_fitness,
                           pop_size=30, seed=7)
    ga.initialize()
    initial_best = ga.best_fitness
    for _ in range(10):
        ga.step()
    assert ga.best_fitness <= initial_best, "Fitness should decrease (minimize sum)"


def test_pre_init_raises_runtime_error():
    ga = GeneticAlgorithm(GAIN_BOUNDS, fitness_fn=mock_fitness, pop_size=10, seed=0)
    with pytest.raises(RuntimeError):
        _ = ga.best_fitness
    with pytest.raises(RuntimeError):
        _ = ga.best_individual
    with pytest.raises(RuntimeError):
        ga.step()


def test_tournament_size_exceeds_pop_raises():
    with pytest.raises(ValueError):
        GeneticAlgorithm(GAIN_BOUNDS, fitness_fn=mock_fitness, pop_size=5, tournament_size=10)


def test_load_checkpoint_shape_mismatch_raises(tmp_path):
    ga = GeneticAlgorithm(GAIN_BOUNDS, fitness_fn=mock_fitness, pop_size=10, seed=0)
    ga.initialize()
    ckpt = str(tmp_path / "ckpt.json")
    ga.save_checkpoint(ckpt)
    # Load into a GA with different pop_size
    ga2 = GeneticAlgorithm(GAIN_BOUNDS, fitness_fn=mock_fitness, pop_size=20, seed=0)
    with pytest.raises(ValueError):
        ga2.load_checkpoint(ckpt)
