import os
import subprocess
import shutil
import json

# Define directories
dirs = ["chatgpt", "gemini", "claude"]
for d in dirs:
    os.makedirs(d, exist_ok=True)

# 1. Prepare ChatGPT solution
shutil.copy("chatgpt_solution.txt", "chatgpt/Solution.java")

# 2. Prepare Claude solution
shutil.copy("claudes_solution.txt", "claude/Solution.java")

# 3. Prepare Gemini solution (needs assembly with HEAD and TAIL)
# Let's extract HEAD and TAIL from 3rd_question, or we can just define them in python
head = """import java.io.*;
import java.util.*;
import com.google.gson.*;

public class Solution {
    public static void main(String[] args) throws Exception {
        BufferedReader reader = new BufferedReader(new InputStreamReader(System.in));
        StringBuilder sb = new StringBuilder();
        String line;
        while ((line = reader.readLine()) != null) {
            sb.append(line);
        }

        JsonObject input = JsonParser.parseString(sb.toString()).getAsJsonObject();

        JsonArray mapSize = input.getAsJsonArray("map_size");
        double warehouseX = mapSize.get(0).getAsDouble() / 2.0;
        double warehouseY = mapSize.get(1).getAsDouble() / 2.0;

        JsonArray drones = input.getAsJsonArray("drones");
        JsonArray deliveries = input.getAsJsonArray("deliveries");
        JsonArray noFlyZones = input.has("no_fly_zones") ? input.getAsJsonArray("no_fly_zones") : new JsonArray();
        JsonArray chargingStations = input.has("charging_stations") ? input.getAsJsonArray("charging_stations") : new JsonArray();
"""

with open("gemini_solution.txt", "r") as f:
    gemini_body = f.read()

tail = """
        JsonObject output = new JsonObject();
        output.add("flight_manifest", flightManifest);
        System.out.println(new Gson().toJson(output));
    }
}
"""

with open("gemini/Solution.java", "w") as f:
    f.write(head + "\n" + gemini_body + "\n" + tail)

# 4. Compile all solutions
classpath = "../gson-2.10.1.jar"
for d in dirs:
    print(f"Compiling {d}...")
    res = subprocess.run(["javac", "-cp", classpath, "Solution.java"], cwd=d, capture_output=True, text=True)
    if res.returncode != 0:
        print(f"Compilation failed for {d}:")
        print(res.stderr)
    else:
        print(f"Compilation succeeded for {d}")

# 5. Run and validate on sample_0 and sample_1
samples = ["sample_0.json", "sample_1.json"]

for d in dirs:
    print("\n" + "="*40)
    print(f"Testing {d.upper()} Solution")
    print("="*40)
    
    if not os.path.exists(f"{d}/Solution.class"):
        print(f"No compiled class for {d}, skipping.")
        continue
        
    for sample in samples:
        print(f"Running on {sample}...")
        # Run the java solution
        try:
            with open(sample, "r") as inf:
                res = subprocess.run(
                    ["java", "-cp", f".;{classpath}", "Solution"],
                    cwd=d,
                    stdin=inf,
                    capture_output=True,
                    text=True,
                    timeout=10
                )
            if res.returncode != 0:
                print(f"Execution failed for {d} on {sample}:")
                print(res.stderr)
                continue
                
            # Parse output
            output_json = res.stdout.strip()
            # Save output to file for validator
            out_file = f"{d}_output_{sample}"
            with open(out_file, "w") as outf:
                outf.write(output_json)
                
            # Run validator
            # We can import validate_solution directly
            import validate
            val_res = validate.validate_solution(sample, out_file)
            print(f"Validation Result: {json.dumps(val_res, indent=2)}")
            
        except subprocess.TimeoutExpired:
            print(f"Timeout expired for {d} on {sample}")
        except Exception as e:
            print(f"Error running {d} on {sample}: {e}")
