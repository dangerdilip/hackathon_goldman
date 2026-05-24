import os
import subprocess
import json
import validate

def run_all():
    print("Compiling Solution.java...")
    res = subprocess.run(["javac", "-cp", ".;gson-2.10.1.jar", "Solution.java"], capture_output=True, text=True)
    if res.returncode != 0:
        print("Compilation FAILED:")
        print(res.stderr)
        return
    print("Compilation SUCCEEDED.\n")

    test_cases = [
        "sample_0.json",
        "sample_1.json",
        "custom_test_1.json",
        "custom_test_2.json",
        "custom_test_3.json",
        "custom_test_4.json"
    ]

    all_passed = True
    for test in test_cases:
        if not os.path.exists(test):
            print(f"Test file {test} not found, skipping.")
            continue
            
        print(f"Running {test}...")
        with open(test, "r") as inf:
            run_res = subprocess.run(
                ["java", "-cp", ".;gson-2.10.1.jar", "Solution"],
                stdin=inf,
                capture_output=True,
                text=True
            )
            
        if run_res.returncode != 0:
            print(f"Execution failed on {test}:")
            print(run_res.stderr)
            all_passed = False
            continue
            
        out_file = f"my_output_{test}"
        with open(out_file, "w") as outf:
            outf.write(run_res.stdout)
            
        # Validate
        val_res = validate.validate_solution(test, out_file)
        if val_res["valid"]:
            print(f"  Result: VALID")
            print(f"  Score: {val_res['raw_score']:.2f}")
            print(f"  Deliveries: {val_res['successful_deliveries']}/{val_res['total_deliveries']}")
            print(f"  Energy: {val_res.get('total_energy', 0):.2f}")
            print(f"  Makespan: {val_res.get('makespan', 0):.2f}")
        else:
            print(f"  Result: INVALID")
            print(f"  Errors: {val_res['errors']}")
            all_passed = False
        print("-" * 50)
        
    if all_passed:
        print("\nALL TEST CASES PASSED SUCCESSFULLY!")
    else:
        print("\nSOME TEST CASES FAILED OR RETURNED INVALID SOLUTIONS.")

if __name__ == "__main__":
    run_all()
