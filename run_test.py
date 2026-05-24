import subprocess
import json
import validate

samples = ["sample_0.json", "sample_1.json"]

for sample in samples:
    print(f"\nRunning on {sample}...")
    with open(sample, "r") as inf:
        res = subprocess.run(
            ["java", "-cp", ".;gson-2.10.1.jar", "Solution"],
            stdin=inf,
            capture_output=True,
            text=True
        )
    if res.returncode != 0:
        print(f"Execution failed on {sample}:")
        print(res.stderr)
        continue
        
    out_file = f"my_output_{sample}"
    with open(out_file, "w") as outf:
        outf.write(res.stdout)
        
    # Validate
    val_res = validate.validate_solution(sample, out_file)
    print(f"Validation Result: {json.dumps(val_res, indent=2)}")
