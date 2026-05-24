import subprocess
import json
import sys
import time

# Add current dir to path to import validate
sys.path.append(".")
import validate

test_file = sys.argv[1] if len(sys.argv) > 1 else "large_test_5000.json"
out_file = f"my_output_{test_file}"

print(f"Running on {test_file}...")
start_time = time.time()
with open(test_file, "r") as inf:
    res = subprocess.run(
        ["java", "-cp", ".;gson-2.10.1.jar", "Solution"],
        stdin=inf,
        capture_output=True,
        text=True
    )
elapsed = time.time() - start_time
print(f"Time taken: {elapsed:.2f}s")
if res.returncode != 0:
    print("Execution failed:")
    print(res.stderr)
else:
    if res.stderr:
        print("Java Stderr output:")
        print(res.stderr)
    with open(out_file, "w", encoding="utf-8") as outf:
        outf.write(res.stdout)
    val_res = validate.validate_solution(test_file, out_file)
    print(f"Validation Result: {json.dumps(val_res, indent=2)}")
