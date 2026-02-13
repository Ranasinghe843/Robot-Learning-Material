import subprocess
import re
import statistics
import time

GEOMETRIES = ['point', 'circle', 'rectangle']
ALGORITHMS = ['rrt', 'rrtstar']
NUM_SUCCESS_REQUIRED = 40

def run_test(geom, alg):
    costs = []
    runtimes = []
    attempts = 0

    print(f"\n>>> Benchmarking: {geom} + {alg}")

    while len(costs) < NUM_SUCCESS_REQUIRED:
        attempts += 1
        # Call your script via command line
        cmd = [
            "python", "assignment1_part2_2d.py",
            "-g", geom,
            "--alg", alg,
            "--iter", "100",  # Higher iterations increase success chance
            "--blind",         # Disables graph window
            "--fast"           # Disables animations
        ]

        try:
            # Capture terminal output
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
            output = result.stdout

            # Use Regex to find: "SUCCESS - found path of cost 45.12 in 2.15sec"
            match = re.search(r"SUCCESS - found path of cost ([\d.]+) in ([\d.]+)sec", output)

            if match:
                costs.append(float(match.group(1)))
                runtimes.append(float(match.group(2)))
                if len(costs) % 5 == 0:
                    print(f"  Captured {len(costs)}/{NUM_SUCCESS_REQUIRED} successes...")
        
        except subprocess.TimeoutExpired:
            continue
        except Exception as e:
            print(f"Error: {e}")
            break

    return {
        "avg_cost": statistics.mean(costs),
        "std_cost": statistics.stdev(costs),
        "avg_time": statistics.mean(runtimes),
        "std_time": statistics.stdev(runtimes),
        "success_rate": (NUM_SUCCESS_REQUIRED / attempts) * 100
    }

if __name__ == "__main__":
    results_table = {}

    for geom in GEOMETRIES:
        results_table[geom] = {}
        for alg in ALGORITHMS:
            results_table[geom][alg] = run_test(geom, alg)

    # Print Final Summary Table
    print("\n" + "="*50)
    print(f"{'Geom':<10} | {'Alg':<8} | {'Avg Cost':<10} | {'Avg Time':<10}")
    print("-" * 50)
    for geom in GEOMETRIES:
        for alg in ALGORITHMS:
            res = results_table[geom][alg]
            print(f"{geom:<10} | {alg:<8} | {res['avg_cost']:<10.2f} | {res['avg_time']:<10.2f}s")
    print("="*50)