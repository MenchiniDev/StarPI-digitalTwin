import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from starPi import simulate
import random as pyrandom

def save_montecarlo_results(costs, apogees, efforts, out_dir="./data"):
    os.makedirs(out_dir, exist_ok=True)

    # ---- 1) Excel ----
    df = pd.DataFrame({
        "run": list(range(len(costs))),
        "cost": costs,
        "apogee": apogees,
        "effort": efforts,
    })
    excel_path = os.path.join(out_dir, "montecarlo_results.xlsx")
    df.to_excel(excel_path, index=False)
    print(f"\nSaved Monte Carlo table to {excel_path}")

    # ---- 2) Gaussian plots ----
    def plot_gaussian(data, title, xlabel, filename):
        data = np.array(data)
        mu = data.mean()
        sigma = data.std()

        plt.figure()
        # histogram normalized
        counts, bins, _ = plt.hist(data, bins=15, density=True, alpha=0.6)
        x = np.linspace(bins[0], bins[-1], 200)
        pdf = 1.0 / (sigma * np.sqrt(2 * np.pi)) * np.exp(-(x - mu) ** 2 / (2 * sigma ** 2))
        plt.plot(x, pdf)
        plt.title(f"{title}\nμ={mu:.2f}, σ={sigma:.2f}")
        plt.xlabel(xlabel)
        plt.ylabel("Density")
        plt.tight_layout()
        out_path = os.path.join(out_dir, filename)
        plt.savefig(out_path)
        plt.close()
        print(f"Saved {title} plot to {out_path}")

    plot_gaussian(costs,   "Cost distribution",   "Cost",        "cost_distribution.png")
    plot_gaussian(apogees, "Apogee distribution", "Apogee [m]",  "apogee_distribution.png")
    plot_gaussian(efforts, "Effort distribution", "Effort",      "effort_distribution.png")

def evolutionary_search(
    generations=25,
    population_size=15,
    Kp_range=(1e-5, 5e-3),
    Ki_range=(0.0, 1e-3),
    Kd_range=(0.0, 5e-3),
):

    def random_individual():
        return (
            pyrandom.uniform(*Kp_range),
            pyrandom.uniform(*Ki_range),
            pyrandom.uniform(*Kd_range),
        )

    population = [random_individual() for _ in range(population_size)]
    best_overall = None
    best_cost_overall = float("inf")

    for gen in range(generations):
        scored = []
        print(f"\n=== Generation {gen} ===")
        for ind in population:
            Kp, Ki, Kd = ind
            cost, apogee, effort = simulate(Kp, Ki, Kd, make_plots=False)
            scored.append((cost, ind, apogee, effort))

        scored.sort(key=lambda x: x[0])
        best_cost, best_ind, best_apogee, best_effort = scored[0]
        print(
            f"Best in gen {gen}: cost={best_cost:.1f}, apogee={best_apogee:.1f}, "
            f"effort={best_effort:.3f}, "
            f"Kp={best_ind[0]:.6g}, Ki={best_ind[1]:.6g}, Kd={best_ind[2]:.6g}"
        )

        if best_cost < best_cost_overall:
            best_cost_overall = best_cost
            best_overall = best_ind

        parents = [ind for _, ind, _, _ in scored[:3]]

        new_population = []
        while len(new_population) < population_size:
            parent = pyrandom.choice(parents)
            Kp, Ki, Kd = parent

            def mutate(val, span):
                return max(val + pyrandom.gauss(0, span * 0.1), 0.0)

            new_population.append((
                mutate(Kp, Kp_range[1] - Kp_range[0]),
                mutate(Ki, Ki_range[1] - Ki_range[0]),
                mutate(Kd, Kd_range[1] - Kd_range[0]),
            ))

        population = new_population

    print("\n=== Best overall individual ===")
    Kp, Ki, Kd = best_overall
    print(f"Kp={Kp:.6g}, Ki={Ki:.6g}, Kd={Kd:.6g}, best_cost={best_cost_overall:.1f}")
    return Kp, Ki, Kd

if __name__ == "__main__":
    kp, ki, kd = evolutionary_search(
        generations=25,
        population_size=15,
        Kp_range=(1e-5, 5e-3),
        Ki_range=(0.0, 1e-3),
        Kd_range=(0.0, 5e-3),
    )

    costs = []
    apogees = []
    efforts = []

    print("\n=== Testing best individual over 100 runs ===")
    for i in range(100):
        cost, apogee, effort = simulate(kp, ki, kd, make_plots=False)
        costs.append(cost)
        apogees.append(apogee)
        efforts.append(effort)
        print(f"Run {i}: cost={cost:.1f}, apogee={apogee:.1f}, effort={effort:.3f}")

    # stats
    def mean_stddev(data):
        mean = sum(data) / len(data)
        variance = sum((x - mean) ** 2 for x in data) / len(data)
        stddev = variance ** 0.5
        return mean, stddev

    cost_mean, cost_stddev = mean_stddev(costs)
    apogee_mean, apogee_stddev = mean_stddev(apogees)
    effort_mean, effort_stddev = mean_stddev(efforts)

    print(f"\n=== Statistics over 100 runs ===")
    print(f"Cost: mean={cost_mean:.1f}, stddev={cost_stddev:.1f}")
    print(f"Apogee: mean={apogee_mean:.1f}, stddev={apogee_stddev:.1f}")
    print(f"Effort: mean={effort_mean:.3f}, stddev={effort_stddev:.3f}")

    save_montecarlo_results(costs, apogees, efforts, out_dir="./data/results")

    print("Done.")
