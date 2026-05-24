import json
data = json.load(open('my_output_custom_test_1.json'))
for drone in data['flight_manifest']:
    print("Drone:", drone["drone_id"])
    for i, p in enumerate(drone['path']):
        line = "  Step %d: (%s, %s) t=%s action=%s" % (i, p["x"], p["y"], p["t"], p["action"])
        if "delivery_id" in p:
            line += " delivery_id=" + p["delivery_id"]
        if "delivery_ids" in p:
            line += " delivery_ids=" + str(p["delivery_ids"])
        print(line)
