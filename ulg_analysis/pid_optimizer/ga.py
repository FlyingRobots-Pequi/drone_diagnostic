import json
import numpy as np
from typing import Callable, List, Optional, Tuple


class GeneticAlgorithm:
    """Minimizing genetic algorithm for PID gain optimization."""

    def __init__(
        self,
        bounds: List[Tuple[float, float]],
        fitness_fn: Callable[[np.ndarray], float],
        pop_size: int = 100,
        tournament_size: int = 5,
        crossover_prob: float = 0.7,
        mutation_prob: float = 0.2,
        mutation_sigma_frac: float = 0.05,
        seed: Optional[int] = None,
    ):
        self.bounds = bounds
        self.fitness_fn = fitness_fn
        self.pop_size = pop_size
        self.tournament_size = tournament_size
        self.crossover_prob = crossover_prob
        self.mutation_prob = mutation_prob
        self.mutation_sigma_frac = mutation_sigma_frac
        self.rng = np.random.default_rng(seed)
        self.n_genes = len(bounds)

        self.population: Optional[np.ndarray] = None
        self.scores: Optional[np.ndarray] = None
        self.generation: int = 0
        self.history: List[dict] = []  # per-generation stats

    @property
    def best_fitness(self) -> float:
        return float(self.scores.min()) if self.scores is not None else float("inf")

    @property
    def best_individual(self) -> np.ndarray:
        assert self.population is not None and self.scores is not None, "Call initialize() first"
        return self.population[self.scores.argmin()].copy()

    def initialize(self) -> None:
        lo = np.array([b[0] for b in self.bounds])
        hi = np.array([b[1] for b in self.bounds])
        self.population = self.rng.uniform(lo, hi, size=(self.pop_size, self.n_genes))
        self.scores = np.array([self.fitness_fn(ind) for ind in self.population])
        self.generation = 0

    def _tournament(self, scores: np.ndarray, tournament_size: int) -> int:
        idx = self.rng.choice(len(scores), size=tournament_size, replace=False)
        return int(idx[scores[idx].argmin()])

    def _crossover(self, p1: np.ndarray, p2: np.ndarray, prob: float) -> np.ndarray:
        mask = self.rng.random(self.n_genes) < prob
        child = np.where(mask, p1, p2)
        return child.copy()

    def _mutate(self, ind: np.ndarray, prob: float) -> np.ndarray:
        lo = np.array([b[0] for b in self.bounds])
        hi = np.array([b[1] for b in self.bounds])
        sigma = self.mutation_sigma_frac * (hi - lo)
        mask = self.rng.random(self.n_genes) < prob
        noise = self.rng.normal(0.0, sigma)
        mutated = ind + mask * noise
        return np.clip(mutated, lo, hi)

    def step(self) -> None:
        """Advance one generation."""
        assert self.population is not None and self.scores is not None, "Call initialize() first"
        new_pop = np.empty_like(self.population)
        for i in range(self.pop_size):
            p1_idx = self._tournament(self.scores, self.tournament_size)
            p2_idx = self._tournament(self.scores, self.tournament_size)
            child = self._crossover(self.population[p1_idx], self.population[p2_idx],
                                     self.crossover_prob)
            child = self._mutate(child, self.mutation_prob)
            new_pop[i] = child
        new_scores = np.array([self.fitness_fn(ind) for ind in new_pop])

        # Elitism: keep best from previous generation
        best_prev_idx = int(self.scores.argmin())
        worst_new_idx = int(new_scores.argmax())
        new_pop[worst_new_idx] = self.population[best_prev_idx]
        new_scores[worst_new_idx] = self.scores[best_prev_idx]

        self.population = new_pop
        self.scores = new_scores
        self.generation += 1
        self.history.append({
            "generation": self.generation,
            "best": float(new_scores.min()),
            "mean": float(new_scores.mean()),
            "std": float(new_scores.std()),
        })

    def save_checkpoint(self, path: str) -> None:
        assert self.population is not None and self.scores is not None, "Call initialize() first"
        payload = {
            "generation": self.generation,
            "history": self.history,
            "best_individual": self.best_individual.tolist(),
            "best_fitness": self.best_fitness,
            "population": self.population.tolist(),
            "scores": self.scores.tolist(),
        }
        with open(path, "w") as f:
            json.dump(payload, f, indent=2)

    def load_checkpoint(self, path: str) -> None:
        with open(path) as f:
            payload = json.load(f)
        self.generation = payload["generation"]
        self.history = payload["history"]
        self.population = np.array(payload["population"])
        self.scores = np.array(payload["scores"])
