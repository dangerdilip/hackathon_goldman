import json
import math
import sys

def dist(x1, y1, x2, y2):
    return math.hypot(x1 - x2, y1 - y2)

def check_circle_intersection(x1, y1, t1, x2, y2, t2, cx, cy, r, ts, te):
    # Check overlap in time
    o_start = max(t1, ts)
    o_end = min(t2, te)
    if o_start > o_end + 1e-9:
        return False
    
    # Parametrize segment: P(f) = P1 + f * (P2 - P1) for f in [f_start, f_end]
    total_t = t2 - t1
    if total_t < 1e-9:
        # Stationary drone
        d = dist(x1, y1, cx, cy)
        return d <= r + 1e-9
    
    f_start = (o_start - t1) / total_t
    f_end = (o_end - t1) / total_t
    
    dx = x2 - x1
    dy = y2 - y1
    
    # Quadratic equation for dist to center:
    # (x1 + f*dx - cx)^2 + (y1 + f*dy - cy)^2 <= r^2
    # f^2 * (dx^2 + dy^2) + 2*f*( (x1-cx)*dx + (y1-cy)*dy ) + (x1-cx)^2 + (y1-cy)^2 - r^2 <= 0
    a = dx*dx + dy*dy
    b = 2 * ((x1 - cx)*dx + (y1 - cy)*dy)
    c = (x1 - cx)**2 + (y1 - cy)**2 - r*r
    
    if a < 1e-9:
        return c <= 1e-9
        
    disc = b*b - 4*a*c
    if disc < 0:
        return False
        
    f1 = (-b - math.sqrt(disc)) / (2*a)
    f2 = (-b + math.sqrt(disc)) / (2*a)
    
    # Check if [f1, f2] intersects [f_start, f_end]
    i_start = max(f1, f_start)
    i_end = min(f2, f_end)
    return i_start <= i_end + 1e-9

def check_rect_intersection(x1, y1, t1, x2, y2, t2, xmin, ymin, xmax, ymax, ts, te):
    o_start = max(t1, ts)
    o_end = min(t2, te)
    if o_start > o_end + 1e-9:
        return False
        
    total_t = t2 - t1
    if total_t < 1e-9:
        return (xmin - 1e-9 <= x1 <= xmax + 1e-9) and (ymin - 1e-9 <= y1 <= ymax + 1e-9)
        
    f_start = (o_start - t1) / total_t
    f_end = (o_end - t1) / total_t
    
    # Liang-Barsky for interval [f_start, f_end]
    dx = x2 - x1
    dy = y2 - y1
    
    t_enter = 0.0
    t_leave = 1.0
    
    for p, q in [(-dx, x1 - xmin), (dx, xmax - x1), (-dy, y1 - ymin), (dy, ymax - y1)]:
        if abs(p) < 1e-9:
            if q < -1e-9:
                return False
        else:
            t = q / p
            if p < 0:
                t_enter = max(t_enter, t)
            else:
                t_leave = min(t_leave, t)
                
    if t_enter > t_leave + 1e-9:
        return False
        
    # Check intersection of [t_enter, t_leave] and [f_start, f_end]
    return max(t_enter, f_start) <= min(t_leave, f_end) + 1e-9

def validate_solution(input_file, manifest_file):
    with open(input_file, 'r') as f:
        inp = json.load(f)
    with open(manifest_file, 'r') as f:
        out = json.load(f)
        
    width, height = inp["map_size"]
    wx, wy = width / 2.0, height / 2.0
    
    drones = {d["id"]: d for d in inp["drones"]}
    deliveries = {d["id"]: d for d in inp["deliveries"]}
    
    charging_stations = inp.get("charging_stations", [])
    nfzs = inp.get("no_fly_zones", [])
    
    manifest = out.get("flight_manifest", [])
    
    # Keep track of deliveries made
    delivered_ids = set()
    total_energy = 0.0
    makespan = 0.0
    errors = []
    
    # For charging slot tracking
    # We will track charging events and check overlapping slots at each station.
    # Structure: station_coord -> list of (t_start, t_end)
    charging_events = {}
    for st in charging_stations:
        charging_events[(st["x"], st["y"])] = []
        
    # Verify drone paths
    for drone_entry in manifest:
        drone_id = drone_entry.get("drone_id")
        if drone_id not in drones:
            errors.append(f"Unknown drone_id: {drone_id}")
            continue
            
        drone_cfg = drones[drone_id]
        max_pay = drone_cfg["max_payload"]
        
        path = drone_entry.get("path", [])
        if not path:
            continue
            
        # Check start action
        if path[0]["action"] != "PICKUP":
            errors.append(f"Drone {drone_id} path must start with PICKUP")
            
        # Check end action
        if path[-1]["action"] != "RETURN":
            errors.append(f"Drone {drone_id} path must end with RETURN")
            
        cur_t = 0.0
        cur_x, cur_y = wx, wy
        battery = 500.0
        payload_weight = 0.0
        carried_deliveries = set()
        
        # We need to trace each point to check transition validity
        for idx, pt in enumerate(path):
            action = pt.get("action")
            px, py = pt.get("x"), pt.get("y")
            pt_t = pt.get("t")
            
            # 1. Monotonic time
            if pt_t < cur_t - 1e-9:
                errors.append(f"Drone {drone_id} time decreased at step {idx}: {pt_t} < {cur_t}")
            
            # 2. Speed and distance constraint (if not waiting or charging)
            d_leg = dist(cur_x, cur_y, px, py)
            expected_dt = d_leg / 1.0  # speed = 1
            actual_dt = pt_t - cur_t
            
            # Check if drone moved during a wait/charge
            if action in ["WAIT", "CHARGE", "CHARGE_COMPLETE"] and d_leg > 1e-9:
                errors.append(f"Drone {drone_id} moved during {action} action at step {idx}")
                
            # If moving, check time matches distance
            if d_leg > 1e-9:
                if abs(actual_dt - expected_dt) > 1e-2:
                    errors.append(f"Drone {drone_id} travel time mismatch at step {idx}: expected {expected_dt}, got {actual_dt}")
                # Update battery
                battery -= d_leg * (1.0 + payload_weight)
                total_energy += d_leg * (1.0 + payload_weight)
                
                if battery < -1e-9:
                    errors.append(f"Drone {drone_id} ran out of battery (battery={battery}) at step {idx}")
                    
                # Check NFZ collisions during transit
                for nfz in nfzs:
                    shape = nfz["shape"]
                    ts = nfz["T_start"]
                    te = nfz["T_end"]
                    if shape == "circle":
                        cx, cy = nfz["center"]
                        r = nfz["radius"]
                        if check_circle_intersection(cur_x, cur_y, cur_t, px, py, pt_t, cx, cy, r, ts, te):
                            errors.append(f"Drone {drone_id} hit circular NFZ at center {cx},{cy} between t={cur_t} and {pt_t}")
                    elif shape == "rectangle":
                        corners = nfz["corners"]
                        xmin = min(corners[0][0], corners[1][0])
                        xmax = max(corners[0][0], corners[1][0])
                        ymin = min(corners[0][1], corners[1][1])
                        ymax = max(corners[0][1], corners[1][1])
                        if check_rect_intersection(cur_x, cur_y, cur_t, px, py, pt_t, xmin, ymin, xmax, ymax, ts, te):
                            errors.append(f"Drone {drone_id} hit rectangular NFZ between t={cur_t} and {pt_t}")
                            
            # Process action specific logic
            if action == "PICKUP":
                # Must be at warehouse
                if dist(px, py, wx, wy) > 1e-9:
                    errors.append(f"Drone {drone_id} tried to PICKUP away from warehouse at {px},{py}")
                # Fully recharged at warehouse
                battery = 500.0
                # Load packages
                del_ids = pt.get("delivery_ids", [])
                for d_id in del_ids:
                    if d_id not in deliveries:
                        errors.append(f"Drone {drone_id} tried to pickup unknown delivery {d_id}")
                        continue
                    if d_id in delivered_ids:
                        errors.append(f"Drone {drone_id} tried to pickup already delivered package {d_id}")
                    carried_deliveries.add(d_id)
                    payload_weight += deliveries[d_id]["weight"]
                if payload_weight > max_pay + 1e-9:
                    errors.append(f"Drone {drone_id} payload weight {payload_weight} exceeds max {max_pay}")
                    
            elif action == "DELIVER":
                d_id = pt.get("delivery_id")
                if d_id not in carried_deliveries:
                    errors.append(f"Drone {drone_id} tried to DELIVER package {d_id} it is not carrying")
                else:
                    carried_deliveries.remove(d_id)
                    payload_weight -= deliveries[d_id]["weight"]
                    # Check location matches delivery destination
                    del_cfg = deliveries[d_id]
                    if dist(px, py, del_cfg["x"], del_cfg["y"]) > 1e-9:
                        errors.append(f"Drone {drone_id} delivered {d_id} at wrong location {px},{py} instead of {del_cfg['x']},{del_cfg['y']}")
                    # Check deadline
                    if pt_t > del_cfg["deadline"] + 1e-9:
                        errors.append(f"Drone {drone_id} delivered {d_id} late at t={pt_t} (deadline={del_cfg['deadline']})")
                    else:
                        delivered_ids.add(d_id)
                        
            elif action == "CHARGE":
                # Find if we are at a charging station
                station_coord = (px, py)
                if station_coord not in charging_events:
                    errors.append(f"Drone {drone_id} tried to CHARGE at non-charging station {px},{py}")
                # Log charge start
                # We will pair this with the subsequent CHARGE_COMPLETE
                
            elif action == "CHARGE_COMPLETE":
                # Check that previous action was CHARGE or WAIT at same location
                # Find the corresponding CHARGE start
                found_charge_start = False
                prev_charge_idx = idx - 1
                while prev_charge_idx >= 0:
                    prev_pt = path[prev_charge_idx]
                    if prev_pt["action"] == "CHARGE":
                        if dist(prev_pt["x"], prev_pt["y"], px, py) < 1e-9:
                            charge_start_t = prev_pt["t"]
                            found_charge_start = True
                            break
                    prev_charge_idx -= 1
                if not found_charge_start:
                    errors.append(f"Drone {drone_id} had CHARGE_COMPLETE without preceding CHARGE at {px},{py}")
                else:
                    charge_duration = pt_t - charge_start_t
                    battery = min(500.0, battery + charge_duration * 2.0)
                    charging_events[(px, py)].append((charge_start_t, pt_t))
                    
            elif action == "RETURN":
                # Must end carrying no packages
                if carried_deliveries:
                    errors.append(f"Drone {drone_id} returned carrying packages: {carried_deliveries}")
                if dist(px, py, wx, wy) < 1e-9:
                    battery = 500.0  # recharged
                elif (px, py) in charging_events:
                    pass  # returned to charging station
                else:
                    errors.append(f"Drone {drone_id} returned to invalid location {px},{py}")
                    
            cur_x, cur_y = px, py
            cur_t = pt_t
            
        makespan = max(makespan, cur_t)
        
    # Verify charging station slot constraints
    for coord, events in charging_events.items():
        # Count overlapping intervals at any point in time
        # events is a list of (start, end)
        # Find the max overlap using sweep-line
        points = []
        for start, end in events:
            points.append((start, 1))
            points.append((end, -1))
        points.sort(key=lambda x: (x[0], x[1]))
        
        current_overlap = 0
        max_overlap = 0
        for t, val in points:
            current_overlap += val
            max_overlap = max(max_overlap, current_overlap)
            
        # Find station slots
        station_slots = 1
        for st in charging_stations:
            if st["x"] == coord[0] and st["y"] == coord[1]:
                station_slots = st.get("slots", 1)
                break
        if max_overlap > station_slots:
            errors.append(f"Charging station at {coord} exceeded slots capacity: {max_overlap} > {station_slots}")

    successful_deliveries = len(delivered_ids)
    raw_score = (successful_deliveries * 100.0) - (total_energy * 0.1) - (makespan * 0.05)
    
    if errors:
        return {
            "valid": False,
            "errors": errors[:10],
            "raw_score": 0.0,
            "successful_deliveries": successful_deliveries,
            "total_deliveries": len(deliveries)
        }
    else:
        return {
            "valid": True,
            "errors": [],
            "raw_score": raw_score,
            "successful_deliveries": successful_deliveries,
            "total_deliveries": len(deliveries),
            "total_energy": total_energy,
            "makespan": makespan
        }

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python validate.py <input.json> <manifest.json>")
        sys.exit(1)
    res = validate_solution(sys.argv[1], sys.argv[2])
    print(json.dumps(res, indent=2))
