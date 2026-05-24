import java.io.*;
import java.util.*;
import com.google.gson.*;
public class Solution {
static final double BATTERY_MAX = 500.0;
static final double CHARGE_RATE = 2.0;
static final double EPS = 1e-9;
static double mapWidth, mapHeight;
static double warehouseX, warehouseY;
static List<NFZ> nfzList = new ArrayList<>();
static List<Station> stations = new ArrayList<>();
static List<Node> fixedNodes = new ArrayList<>();
static Map<String, List<NFZ>> intersectingNFZs = new HashMap<>();
static long startTime;
static final long TIME_LIMIT_MS = 1950;
static int pathfindCalls = 0;
static int directClearCalls = 0;
static int dijkstraCalls = 0;
static long dijkstraTimeMs = 0;
static int outerLoopIters = 0;
static int seedsChecked = 0;
static int candidateBatchesCalls = 0;
static int findBestPermutationCalls = 0;
static class Delivery {
String id;
double x, y, weight, deadline;
double distToWarehouse;
Delivery(String id, double x, double y, double weight, double deadline) {
this.id = id; this.x = x; this.y = y; this.weight = weight; this.deadline = deadline;
this.distToWarehouse = dist(x, y, warehouseX, warehouseY);
}
}
static class Drone {
String id;
double maxPayload;
double time = 0.0;
List<PathPoint> path = new ArrayList<>();
Set<String> skipped = new HashSet<>();
Drone(String id, double maxPayload) { this.id = id; this.maxPayload = maxPayload; }
}
static class NFZ {
String shape;
double cx, cy, radius;
double xmin, ymin, xmax, ymax;
double tStart, tEnd;
}
static class Station {
double x, y;
int slots;
List<double[]> reservations = new ArrayList<>();
Station(double x, double y, int slots) { this.x = x; this.y = y; this.slots = slots; }
}
static class PathPoint {
double x, y, t;
String action;
String deliveryId;
List<String> deliveryIds;
PathPoint(double x, double y, double t, String action) {
this.x = x; this.y = y; this.t = t; this.action = action;
}
}
static class Node {
double x, y;
int id = -1;
NFZ parentNFZ = null;
Node(double x, double y) { this.x = x; this.y = y; }
Node(double x, double y, int id, NFZ parentNFZ) {
this.x = x; this.y = y; this.id = id; this.parentNFZ = parentNFZ;
}
}
public static void main(String[] args) throws Exception {
startTime = System.currentTimeMillis();
BufferedReader br = new BufferedReader(new InputStreamReader(System.in));
StringBuilder sb = new StringBuilder();
String line;
while ((line = br.readLine()) != null) sb.append(line);
JsonObject input = JsonParser.parseString(sb.toString()).getAsJsonObject();
JsonArray mapSize = input.getAsJsonArray("map_size");
mapWidth = mapSize.get(0).getAsDouble();
mapHeight = mapSize.get(1).getAsDouble();
warehouseX = mapWidth / 2.0;
warehouseY = mapHeight / 2.0;
JsonArray dronesArr = input.getAsJsonArray("drones");
JsonArray deliveriesArr = input.getAsJsonArray("deliveries");
JsonArray nfzArr = input.has("no_fly_zones") ? input.getAsJsonArray("no_fly_zones") : new JsonArray();
JsonArray stArr = input.has("charging_stations") ? input.getAsJsonArray("charging_stations") : new JsonArray();
for (JsonElement e : nfzArr) {
JsonObject o = e.getAsJsonObject();
NFZ n = new NFZ();
n.shape = o.get("shape").getAsString();
n.tStart = o.get("T_start").getAsDouble();
n.tEnd = o.get("T_end").getAsDouble();
if (n.shape.equals("circle")) {
JsonArray c = o.getAsJsonArray("center");
n.cx = c.get(0).getAsDouble();
n.cy = c.get(1).getAsDouble();
n.radius = o.get("radius").getAsDouble();
n.xmin = n.cx - n.radius;
n.xmax = n.cx + n.radius;
n.ymin = n.cy - n.radius;
n.ymax = n.cy + n.radius;
} else {
if (o.has("corners")) {
JsonArray c = o.getAsJsonArray("corners");
JsonArray a = c.get(0).getAsJsonArray();
JsonArray b = c.get(1).getAsJsonArray();
n.xmin = Math.min(a.get(0).getAsDouble(), b.get(0).getAsDouble());
n.xmax = Math.max(a.get(0).getAsDouble(), b.get(0).getAsDouble());
n.ymin = Math.min(a.get(1).getAsDouble(), b.get(1).getAsDouble());
n.ymax = Math.max(a.get(1).getAsDouble(), b.get(1).getAsDouble());
} else if (o.has("bottom_left") && o.has("top_right")) {
JsonArray bl = o.getAsJsonArray("bottom_left");
JsonArray tr = o.getAsJsonArray("top_right");
n.xmin = bl.get(0).getAsDouble();
n.ymin = bl.get(1).getAsDouble();
n.xmax = tr.get(0).getAsDouble();
n.ymax = tr.get(1).getAsDouble();
}
}
nfzList.add(n);
}
for (JsonElement e : stArr) {
JsonObject o = e.getAsJsonObject();
int slots = o.has("slots") ? o.get("slots").getAsInt() : 1;
stations.add(new Station(o.get("x").getAsDouble(), o.get("y").getAsDouble(), slots));
}
List<Delivery> allDeliveries = new ArrayList<>();
for (JsonElement e : deliveriesArr) {
JsonObject o = e.getAsJsonObject();
allDeliveries.add(new Delivery(
o.get("id").getAsString(),
o.get("x").getAsDouble(), o.get("y").getAsDouble(),
o.get("weight").getAsDouble(), o.get("deadline").getAsDouble()
));
}
List<Drone> drones = new ArrayList<>();
for (JsonElement e : dronesArr) {
JsonObject o = e.getAsJsonObject();
drones.add(new Drone(o.get("id").getAsString(), o.get("max_payload").getAsDouble()));
}
fixedNodes = new ArrayList<>();
double delta = 0.1;
int nodeCounter = 0;
for (NFZ nfz : nfzList) {
if (nfz.shape.equals("circle")) {
for (int angle = 0; angle < 360; angle += 90) {
double rad = Math.toRadians(angle);
double r = nfz.radius + delta;
fixedNodes.add(new Node(nfz.cx + Math.cos(rad) * r, nfz.cy + Math.sin(rad) * r, nodeCounter++, nfz));
}
} else {
fixedNodes.add(new Node(nfz.xmin - delta, nfz.ymin - delta, nodeCounter++, nfz));
fixedNodes.add(new Node(nfz.xmin - delta, nfz.ymax + delta, nodeCounter++, nfz));
fixedNodes.add(new Node(nfz.xmax + delta, nfz.ymin - delta, nodeCounter++, nfz));
fixedNodes.add(new Node(nfz.xmax + delta, nfz.ymax + delta, nodeCounter++, nfz));
}
}
for (Station s : stations) {
fixedNodes.add(new Node(s.x, s.y, nodeCounter++, null));
}
fixedNodes.add(new Node(warehouseX, warehouseY, nodeCounter++, null));
for (Node n : fixedNodes) {
n.x = Math.max(0.1, Math.min(mapWidth - 0.1, n.x));
n.y = Math.max(0.1, Math.min(mapHeight - 0.1, n.y));
}
scheduleAll(allDeliveries, drones);
JsonArray manifest = new JsonArray();
for (Drone d : drones) {
if (d.path.isEmpty()) continue;
JsonObject droneObj = new JsonObject();
droneObj.addProperty("drone_id", d.id);
JsonArray pathArray = new JsonArray();
for (PathPoint pp : d.path) {
JsonObject pt = new JsonObject();
pt.addProperty("x", pp.x);
pt.addProperty("y", pp.y);
pt.addProperty("t", pp.t);
pt.addProperty("action", pp.action);
if (pp.deliveryIds != null) {
JsonArray ids = new JsonArray();
for (String s : pp.deliveryIds) ids.add(s);
pt.add("delivery_ids", ids);
}
if (pp.deliveryId != null) {
pt.addProperty("delivery_id", pp.deliveryId);
}
pathArray.add(pt);
}
droneObj.add("path", pathArray);
manifest.add(droneObj);
}
JsonObject out = new JsonObject();
out.add("flight_manifest", manifest);
System.out.println(new Gson().toJson(out));
}
static double round2(double v) {
return Math.round(v * 100.0) / 100.0;
}
static double estimateBatchScore(List<Delivery> batch, double tStart) {
List<Delivery> ordered = new ArrayList<>();
List<Delivery> pool = new ArrayList<>(batch);
double cx = warehouseX, cy = warehouseY;
while (!pool.isEmpty()) {
int bestIdx = 0;
double bestDist = Double.MAX_VALUE;
for (int i = 0; i < pool.size(); i++) {
double d = dist(cx, cy, pool.get(i).x, pool.get(i).y);
if (d < bestDist) { bestDist = d; bestIdx = i; }
}
Delivery d = pool.remove(bestIdx);
ordered.add(d);
cx = d.x; cy = d.y;
}
double curX = warehouseX, curY = warehouseY, t = tStart;
double payload = 0;
for (Delivery d : ordered) payload += d.weight;
double estEnergy = 0;
for (Delivery d : ordered) {
double dDist = dist(curX, curY, d.x, d.y);
t += dDist;
if (t > d.deadline) return -Double.MAX_VALUE;
estEnergy += dDist * (1.0 + payload);
curX = d.x; curY = d.y;
payload -= d.weight;
}
double retDist = dist(curX, curY, warehouseX, warehouseY);
t += retDist;
estEnergy += retDist * 1.0;
if (estEnergy > BATTERY_MAX) {
double minDetour = Double.MAX_VALUE;
for (Station s : stations) {
double detour = dist(curX, curY, s.x, s.y) + dist(s.x, s.y, warehouseX, warehouseY) - retDist;
if (detour < minDetour) minDetour = detour;
}
if (minDetour != Double.MAX_VALUE) {
estEnergy += minDetour * 1.0;
} else {
return -Double.MAX_VALUE;
}
}
double urgencyBonus = ordered.stream().mapToDouble(d -> 1.0 / (Math.max(1.0, d.deadline - tStart))).sum() * 1000;
double score = (ordered.size() * 100.0) + urgencyBonus - (estEnergy * 0.1);
return score;
}
static void scheduleAll(List<Delivery> deliveries, List<Drone> drones) {
deliveries.sort(Comparator.comparingDouble(d -> d.deadline - d.distToWarehouse));
Set<String> done = new HashSet<>();
int prevDoneSize = -1;
int maxIter = deliveries.size() * 3;
class BatchWithScore {
List<Delivery> batch;
double estScore;
BatchWithScore(List<Delivery> b, double s) { this.batch = b; this.estScore = s; }
}
while (done.size() < deliveries.size() && done.size() != prevDoneSize && maxIter-- > 0) {
outerLoopIters++;
if (System.currentTimeMillis() - startTime > TIME_LIMIT_MS - 100) {
break;
}
prevDoneSize = done.size();
for (Drone drone : drones) {
if (System.currentTimeMillis() - startTime > TIME_LIMIT_MS - 100) break;
TripPlan bestTrip = null;
double bestTripScore = -Double.MAX_VALUE;
while (true) {
if (System.currentTimeMillis() - startTime > TIME_LIMIT_MS - 100) break;
Delivery seed = null;
for (Delivery d : deliveries) {
if (!done.contains(d.id) && !drone.skipped.contains(d.id) && d.weight <= drone.maxPayload + EPS) {
if (drone.time + d.distToWarehouse <= d.deadline) {
seed = d;
break;
}
}
}
if (seed == null) {
break;
}
seedsChecked++;
TripPlan seedPlan = attemptTrip(Collections.singletonList(seed), drone.time);
if (seedPlan == null) {
drone.skipped.add(seed.id);
continue;
}
List<Delivery> remaining = new ArrayList<>();
remaining.add(seed);
List<Delivery> candidates = new ArrayList<>();
for (Delivery d : deliveries) {
if (d != seed && !done.contains(d.id) && !drone.skipped.contains(d.id) && d.weight <= drone.maxPayload + EPS) {
if (drone.time + d.distToWarehouse <= d.deadline) {
if (Math.abs(d.x - seed.x) <= 180.0 && Math.abs(d.y - seed.y) <= 180.0) {
candidates.add(d);
}
}
}
}
final Delivery finalSeed = seed;
candidates.sort(Comparator.comparingDouble(d -> dist(finalSeed.x, finalSeed.y, d.x, d.y)));
for (int i = 0; i < Math.min(60, candidates.size()); i++) {
remaining.add(candidates.get(i));
}
candidateBatchesCalls++;
List<List<Delivery>> candidateBatches = getCandidateBatches(remaining, drone.maxPayload);
List<BatchWithScore> sortedBatches = new ArrayList<>();
for (List<Delivery> batch : candidateBatches) {
double est = estimateBatchScore(batch, drone.time);
if (est > -Double.MAX_VALUE + EPS) {
sortedBatches.add(new BatchWithScore(batch, est));
}
}
sortedBatches.sort((a, b) -> Double.compare(b.estScore, a.estScore));
for (int idx = 0; idx < Math.min(3, sortedBatches.size()); idx++) {
if (System.currentTimeMillis() - startTime > TIME_LIMIT_MS - 100) break;
List<Delivery> batch = sortedBatches.get(idx).batch;
findBestPermutationCalls++;
TripPlan plan = findBestPermutation(batch, drone.time);
if (plan != null && plan.netValue > 0) {
double urgencyBonus = batch.stream().mapToDouble(d -> 1.0 / (Math.max(1.0, d.deadline - drone.time))).sum() * 1000;
double score = (plan.batch.size() * 100.0) + urgencyBonus - (plan.totalEnergy * 0.1);
if (bestTrip == null || score > bestTripScore) {
bestTrip = plan;
bestTripScore = score;
}
}
}
if (bestTrip != null) {
break;
} else {
drone.skipped.add(seed.id);
}
}
if (bestTrip != null) {
for (Delivery d : bestTrip.batch) done.add(d.id);
drone.path.addAll(bestTrip.points);
drone.time = bestTrip.endTime;
for (StationReservation res : bestTrip.reservations) {
res.station.reservations.add(new double[]{res.tStart, res.tEnd});
res.station.reservations.sort(Comparator.comparingDouble(a -> a[0]));
}
}
}         }
optimizeFinalReturns(drones, deliveries);
rebuildReservations(drones);
}
static void optimizeFinalReturns(List<Drone> drones, List<Delivery> allDeliveries) {
Map<String, Double> deliveryWeights = new HashMap<>();
for (Delivery d : allDeliveries) {
deliveryWeights.put(d.id, d.weight);
}
for (Drone d : drones) {
if (d.path.isEmpty()) continue;
int lastDeliverIdx = -1;
for (int i = 0; i < d.path.size(); i++) {
if (d.path.get(i).action.equals("DELIVER")) {
lastDeliverIdx = i;
}
}
if (lastDeliverIdx == -1) {
d.path.clear();
d.time = 0.0;
continue;
}
PathPoint ppDel = d.path.get(lastDeliverIdx);
double battery = getBatteryAt(d.path, lastDeliverIdx, deliveryWeights);
double bestScore = Double.MAX_VALUE;
Transition bestTransition = null;
List<StationReservation> tempRes = new ArrayList<>();
Transition transWH = makeTransition(ppDel.x, ppDel.y, warehouseX, warehouseY, ppDel.t, battery, 0.0, 0.0, tempRes);
if (transWH != null) {
double endT = transWH.points.get(transWH.points.size() - 1).t;
bestScore = transWH.energyConsumed * 0.1 + endT * 0.05;
bestTransition = transWH;
}
for (Station st : stations) {
tempRes = new ArrayList<>();
Transition transSt = makeTransition(ppDel.x, ppDel.y, st.x, st.y, ppDel.t, battery, 0.0, 0.0, tempRes);
if (transSt != null) {
double endT = transSt.points.get(transSt.points.size() - 1).t;
double score = transSt.energyConsumed * 0.1 + endT * 0.05;
if (score < bestScore) {
bestScore = score;
bestTransition = transSt;
}
}
}
if (bestTransition != null) {
List<PathPoint> newPath = new ArrayList<>();
for (int i = 0; i <= lastDeliverIdx; i++) {
newPath.add(d.path.get(i));
}
newPath.addAll(bestTransition.points);
newPath.get(newPath.size() - 1).action = "RETURN";
d.path = newPath;
d.time = newPath.get(newPath.size() - 1).t;
}
}
}
static double getBatteryAt(List<PathPoint> path, int limitIdx, Map<String, Double> deliveryWeights) {
double battery = 500.0;
double curX = warehouseX, curY = warehouseY;
double payloadWeight = 0.0;
Set<String> carried = new HashSet<>();
for (int i = 0; i <= limitIdx; i++) {
PathPoint pt = path.get(i);
double d = dist(curX, curY, pt.x, pt.y);
if (d > 1e-9) {
battery -= d * (1.0 + payloadWeight);
}
if (pt.action.equals("PICKUP")) {
battery = 500.0;
if (pt.deliveryIds != null) {
for (String id : pt.deliveryIds) {
carried.add(id);
payloadWeight += deliveryWeights.getOrDefault(id, 0.0);
}
}
} else if (pt.action.equals("DELIVER")) {
if (pt.deliveryId != null) {
carried.remove(pt.deliveryId);
payloadWeight -= deliveryWeights.getOrDefault(pt.deliveryId, 0.0);
}
} else if (pt.action.equals("CHARGE_COMPLETE")) {
double chargeStartT = pt.t;
double waitStartT = -1;
for (int j = i - 1; j >= 0; j--) {
String act = path.get(j).action;
if (act.equals("WAIT")) {
waitStartT = path.get(j).t;
}
if (act.equals("CHARGE")) {
chargeStartT = path.get(j).t;
break;
}
}
double actualStart = (waitStartT >= 0) ? waitStartT : chargeStartT;
double duration = pt.t - actualStart;
battery = Math.min(500.0, battery + duration * 2.0);
}
curX = pt.x;
curY = pt.y;
}
return Math.max(0.0, battery);
}
static void rebuildReservations(List<Drone> drones) {
for (Station st : stations) {
st.reservations.clear();
}
for (Drone d : drones) {
for (int i = 0; i < d.path.size(); i++) {
PathPoint pt = d.path.get(i);
if (pt.action.equals("CHARGE")) {
double tStart = pt.t;
double tEnd = tStart;
for (int j = i + 1; j < d.path.size(); j++) {
if (d.path.get(j).action.equals("CHARGE_COMPLETE") && dist(d.path.get(j).x, d.path.get(j).y, pt.x, pt.y) < 1.0) {
tEnd = d.path.get(j).t;
break;
}
}
for (Station st : stations) {
if (dist(st.x, st.y, pt.x, pt.y) < 1.0) {
st.reservations.add(new double[]{tStart, tEnd});
break;
}
}
}
}
}
}
static void addBatchIfUnique(List<List<Delivery>> batches, List<Delivery> batch) {
if (batch.isEmpty()) return;
for (List<Delivery> existing : batches) {
if (sameBatch(existing, batch)) return;
}
batches.add(batch);
}
static List<List<Delivery>> getCandidateBatches(List<Delivery> remaining, double maxPayload) {
List<List<Delivery>> batches = new ArrayList<>();
List<Delivery> batch1 = new ArrayList<>();
double w1 = 0;
for (Delivery d : remaining) {
if (w1 + d.weight <= maxPayload + EPS) { batch1.add(d); w1 += d.weight; }
}
addBatchIfUnique(batches, batch1);
List<Delivery> batch2 = new ArrayList<>();
double w2 = 0;
Set<Integer> used2 = new HashSet<>();
double cx = warehouseX, cy = warehouseY;
while (true) {
int bestIdx = -1;
double bestDist = Double.MAX_VALUE;
for (int i = 0; i < remaining.size(); i++) {
if (used2.contains(i)) continue;
Delivery d = remaining.get(i);
if (w2 + d.weight > maxPayload + EPS) continue;
double dd = dist(cx, cy, d.x, d.y);
if (dd < bestDist) { bestDist = dd; bestIdx = i; }
}
if (bestIdx == -1) break;
Delivery d = remaining.get(bestIdx);
batch2.add(d);
w2 += d.weight;
cx = d.x; cy = d.y;
used2.add(bestIdx);
}
addBatchIfUnique(batches, batch2);
int limit = Math.min(remaining.size(), 15);
for (int i = 0; i < limit; i++) {
Delivery center = remaining.get(i);
List<Delivery> cluster = new ArrayList<>();
cluster.add(center);
double w = center.weight;
List<Delivery> others = new ArrayList<>(remaining);
others.remove(center);
others.sort(Comparator.comparingDouble(a -> dist(center.x, center.y, a.x, a.y)));
for (Delivery o : others) {
if (w + o.weight <= maxPayload + EPS) { cluster.add(o); w += o.weight; }
}
addBatchIfUnique(batches, cluster);
}
List<Delivery> sweepPool = new ArrayList<>(remaining);
sweepPool.sort(Comparator.comparingDouble(d -> Math.atan2(d.y - warehouseY, d.x - warehouseX)));
int nSweep = sweepPool.size();
int sweepLimit = Math.min(nSweep, 15);
for (int i = 0; i < sweepLimit; i++) {
List<Delivery> sweepBatch = new ArrayList<>();
double w = 0;
for (int j = 0; j < nSweep; j++) {
Delivery d = sweepPool.get((i + j) % nSweep);
if (w + d.weight <= maxPayload + EPS) { sweepBatch.add(d); w += d.weight; }
}
addBatchIfUnique(batches, sweepBatch);
}
int singleLimit = Math.min(remaining.size(), 15);
for (int i = 0; i < singleLimit; i++) {
List<Delivery> single = new ArrayList<>();
single.add(remaining.get(i));
addBatchIfUnique(batches, single);
}
List<Delivery> knapsack = new ArrayList<>();
double wK = 0;
List<Delivery> sortedRem = new ArrayList<>(remaining);
sortedRem.sort((a, b) -> {
double d1 = dist(warehouseX, warehouseY, a.x, a.y);
double d2 = dist(warehouseX, warehouseY, b.x, b.y);
return Double.compare(d1, d2);
});
for (Delivery d : sortedRem) {
if (wK + d.weight <= maxPayload + EPS) {
knapsack.add(d);
wK += d.weight;
}
}
addBatchIfUnique(batches, knapsack);
if (remaining.size() <= 8) {
int n = remaining.size();
for (int mask = 1; mask < (1 << n); mask++) {
List<Delivery> subset = new ArrayList<>();
double w = 0;
boolean valid = true;
for (int i = 0; i < n; i++) {
if (((mask >> i) & 1) == 1) {
if (w + remaining.get(i).weight > maxPayload + EPS) {
valid = false;
break;
}
subset.add(remaining.get(i));
w += remaining.get(i).weight;
}
}
if (valid && !subset.isEmpty()) addBatchIfUnique(batches, subset);
}
}
return batches;
}
static boolean sameBatch(List<Delivery> a, List<Delivery> b) {
if (a.size() != b.size()) return false;
Set<String> sa = new HashSet<>();
for (Delivery d : a) sa.add(d.id);
for (Delivery d : b) if (!sa.contains(d.id)) return false;
return true;
}
static class TripPlan {
List<Delivery> batch;
List<PathPoint> points;
double endTime;
double loadBalanceScore;
double netValue;
double totalEnergy;
List<StationReservation> reservations;
TripPlan(List<Delivery> batch, List<PathPoint> points, double endTime, double loadBalanceScore, double netValue, double totalEnergy, List<StationReservation> reservations) {
this.batch = batch; this.points = points; this.endTime = endTime;
this.loadBalanceScore = loadBalanceScore; this.netValue = netValue;
this.totalEnergy = totalEnergy; this.reservations = reservations;
}
}
static class StationReservation {
Station station;
double tStart, tEnd;
StationReservation(Station station, double tStart, double tEnd) {
this.station = station; this.tStart = tStart; this.tEnd = tEnd;
}
}
static List<Delivery> euclidean2Opt(List<Delivery> batch) {
List<Delivery> current = nearestNeighborOrder(batch);
boolean improved = true;
while (improved) {
improved = false;
double currentDist = getEuclideanTourLength(current);
for (int i = 0; i < current.size() - 1; i++) {
for (int j = i + 1; j < current.size(); j++) {
List<Delivery> neighbor = new ArrayList<>(current);
int left = i, right = j;
while (left < right) {
Collections.swap(neighbor, left, right);
left++; right--;
}
double neighborDist = getEuclideanTourLength(neighbor);
if (neighborDist < currentDist - EPS) {
current = neighbor;
currentDist = neighborDist;
improved = true;
}
}
}
}
return current;
}
static double getEuclideanTourLength(List<Delivery> path) {
double distSum = 0.0;
double cx = warehouseX, cy = warehouseY;
for (Delivery d : path) {
distSum += dist(cx, cy, d.x, d.y);
cx = d.x; cy = d.y;
}
distSum += dist(cx, cy, warehouseX, warehouseY);
return distSum;
}
static TripPlan findBestPermutation(List<Delivery> batch, double tStart) {
for (Delivery d : batch) {
if (tStart + dist(warehouseX, warehouseY, d.x, d.y) > d.deadline) {
return null;
}
}
TripPlan bestPlan = null;
if (batch.size() > 3) {
List<Delivery> current = euclidean2Opt(batch);
bestPlan = attemptTrip(current, tStart);
if (bestPlan == null) {
bestPlan = attemptTrip(weightSheddingOrder(batch), tStart);
}
} else {
List<List<Delivery>> perms = new ArrayList<>();
generatePermutations(new ArrayList<>(batch), 0, perms);
for (List<Delivery> perm : perms) {
TripPlan plan = attemptTrip(perm, tStart);
if (plan != null && (bestPlan == null || plan.loadBalanceScore > bestPlan.loadBalanceScore)) {
bestPlan = plan;
}
}
}
if (bestPlan == null && batch.size() > 1) {
Delivery tightest = null;
for (Delivery d : batch) {
if (tightest == null || d.deadline < tightest.deadline) tightest = d;
}
List<Delivery> reduced = new ArrayList<>(batch);
reduced.remove(tightest);
return findBestPermutation(reduced, tStart);
}
return bestPlan;
}
static List<Delivery> nearestNeighborOrder(List<Delivery> batch) {
List<Delivery> result = new ArrayList<>();
List<Delivery> pool = new ArrayList<>(batch);
double cx = warehouseX, cy = warehouseY;
while (!pool.isEmpty()) {
int bestIdx = 0;
double bestDist = Double.MAX_VALUE;
for (int i = 0; i < pool.size(); i++) {
double d = dist(cx, cy, pool.get(i).x, pool.get(i).y);
if (d < bestDist) { bestDist = d; bestIdx = i; }
}
Delivery d = pool.remove(bestIdx);
result.add(d);
cx = d.x; cy = d.y;
}
return result;
}
static List<Delivery> weightSheddingOrder(List<Delivery> batch) {
List<Delivery> result = new ArrayList<>();
List<Delivery> pool = new ArrayList<>(batch);
double cx = warehouseX, cy = warehouseY;
while (!pool.isEmpty()) {
int bestIdx = 0;
double bestScore = Double.MAX_VALUE;
for (int i = 0; i < pool.size(); i++) {
double d = dist(cx, cy, pool.get(i).x, pool.get(i).y);
double score = d / (pool.get(i).weight + 0.01);
if (score < bestScore) {
bestScore = score;
bestIdx = i;
}
}
Delivery d = pool.remove(bestIdx);
result.add(d);
cx = d.x; cy = d.y;
}
return result;
}
static void generatePermutations(List<Delivery> list, int k, List<List<Delivery>> res) {
if (k == list.size() - 1) {
res.add(new ArrayList<>(list));
return;
}
for (int i = k; i < list.size(); i++) {
Collections.swap(list, i, k);
generatePermutations(list, k + 1, res);
Collections.swap(list, k, i);
}
}
static TripPlan attemptTrip(List<Delivery> ordered, double tStart) {
List<PathPoint> pathPoints = new ArrayList<>();
List<StationReservation> reservations = new ArrayList<>();
double payload = 0.0;
for (Delivery d : ordered) payload += d.weight;
double curX = warehouseX, curY = warehouseY, t = tStart;
double battery = BATTERY_MAX;
PathPoint pk = new PathPoint(warehouseX, warehouseY, t, "PICKUP");
pk.deliveryIds = new ArrayList<>();
for (Delivery d : ordered) pk.deliveryIds.add(d.id);
pathPoints.add(pk);
double totalEnergy = 0.0;
for (int j = 0; j < ordered.size(); j++) {
Delivery d = ordered.get(j);
double payloadAfterDeliver = payload - d.weight;
Transition result = makeTransition(curX, curY, d.x, d.y, t, battery, payload, payloadAfterDeliver, reservations);
if (result == null) return null;
double arrTime = result.points.get(result.points.size() - 1).t;
if (arrTime > d.deadline + EPS) return null;
PathPoint last = result.points.get(result.points.size() - 1);
last.action = "DELIVER";
last.deliveryId = d.id;
pathPoints.addAll(result.points);
totalEnergy += result.energyConsumed;
battery = result.endBattery;
t = arrTime;
curX = d.x;
curY = d.y;
payload -= d.weight;
}
Transition retResult = makeTransition(curX, curY, warehouseX, warehouseY, t, battery, 0.0, 0.0, reservations);
if (retResult == null) return null;
PathPoint retLast = retResult.points.get(retResult.points.size() - 1);
retLast.action = "RETURN";
pathPoints.addAll(retResult.points);
totalEnergy += retResult.energyConsumed;
t = retLast.t;
double tripDuration = t - tStart;
double loadBalanceScore = (ordered.size() * 100.0) - (totalEnergy * 0.1) - (t * 0.05);
double netValue = (ordered.size() * 100.0) - (totalEnergy * 0.1) - (tripDuration * 0.05);
return new TripPlan(ordered, pathPoints, t, loadBalanceScore, netValue, totalEnergy, reservations);
}
static class Transition {
List<PathPoint> points;
double energyConsumed;
double endBattery;
StationReservation reservation;
Transition(List<PathPoint> points, double energyConsumed, double endBattery) {
this.points = points; this.energyConsumed = energyConsumed; this.endBattery = endBattery;
}
}
static Transition makeTransition(double ax, double ay, double bx, double by, double tStart,
double startBattery, double payload, double payloadAtDest,
List<StationReservation> reservations) {
double euclidDist = dist(ax, ay, bx, by);
if (startBattery >= euclidDist * (1.0 + payload) - EPS) {
List<PathPoint> dirPath = findShortestPath(ax, ay, bx, by, tStart, payload);
if (dirPath != null) {
double energy = getPathEnergy(ax, ay, dirPath, payload);
if (startBattery - energy >= -EPS) {
double remBat = startBattery - energy;
if (canReachAnyStation(bx, by, remBat, payloadAtDest)) {
return new Transition(dirPath, energy, Math.max(0, remBat));
}
}
}
}
Transition bestOption = null;
double bestArrivalTime = Double.MAX_VALUE;
List<Station> candidates = new ArrayList<>(stations);
if (candidates.size() > 2) {
final double fax = ax, fay = ay, fbx = bx, fby = by;
candidates.sort(Comparator.comparingDouble(st -> dist(fax, fay, st.x, st.y) + dist(st.x, st.y, fbx, fby)));
candidates = candidates.subList(0, 2);
}
for (Station st : candidates) {
double distToSt = dist(ax, ay, st.x, st.y);
boolean stationAtCurrentPos = distToSt < 1.0;
if (!stationAtCurrentPos && startBattery < distToSt * (1.0 + payload) - EPS) continue;
List<PathPoint> path1 = null;
double e1 = 0.0;
double arrTimeC = tStart;
double batAtC = startBattery;
if (!stationAtCurrentPos) {
path1 = findShortestPath(ax, ay, st.x, st.y, tStart, payload);
if (path1 == null) continue;
e1 = getPathEnergy(ax, ay, path1, payload);
if (startBattery - e1 < -EPS) continue;
arrTimeC = path1.get(path1.size() - 1).t;
batAtC = startBattery - e1;
}
double distStToB = dist(st.x, st.y, bx, by);
double distBToWH = dist(bx, by, warehouseX, warehouseY);
double minEnergyToB = distStToB * (1.0 + payload);
if (minEnergyToB > BATTERY_MAX - EPS) continue;
double minEnergyBToReturn = distBToWH * (1.0 + payloadAtDest);
double targetBat = Math.min(BATTERY_MAX, minEnergyToB + minEnergyBToReturn + 10.0);
targetBat = Math.max(targetBat, minEnergyToB + 10.0);
targetBat = Math.min(BATTERY_MAX, targetBat);
for (int attempt = 0; attempt < 2; attempt++) {
double chargeNeeded = Math.max(0.0, targetBat - batAtC);
double chargeTime = chargeNeeded / CHARGE_RATE;
double actualChargeStart = getActualChargeStartTime(st, arrTimeC, chargeTime, reservations);
double tDep = actualChargeStart + chargeTime;
List<PathPoint> path2 = findShortestPath(st.x, st.y, bx, by, tDep, payload);
if (path2 == null) break;
double e2 = getPathEnergy(st.x, st.y, path2, payload);
if (targetBat - e2 < -EPS) {
targetBat = Math.min(BATTERY_MAX, e2 + minEnergyBToReturn + 10.0);
continue;
}
double arrTimeB = path2.get(path2.size() - 1).t;
double endBat = targetBat - e2;
if (endBat >= -EPS && canReachAnyStation(bx, by, endBat, payloadAtDest)) {
if (arrTimeB < bestArrivalTime) {
bestArrivalTime = arrTimeB;
List<PathPoint> combined = new ArrayList<>();
double waitTime = actualChargeStart - arrTimeC;
if (stationAtCurrentPos) {
if (waitTime > EPS) {
combined.add(new PathPoint(st.x, st.y, actualChargeStart, "WAIT"));
}
combined.add(new PathPoint(st.x, st.y, actualChargeStart, "CHARGE"));
} else {
if (waitTime > EPS) {
combined.add(new PathPoint(ax, ay, tStart + waitTime, "WAIT"));
}
for (PathPoint p : path1) {
combined.add(new PathPoint(p.x, p.y, p.t + waitTime, p.action));
}
combined.add(new PathPoint(st.x, st.y, actualChargeStart, "CHARGE"));
}
combined.add(new PathPoint(st.x, st.y, tDep, "CHARGE_COMPLETE"));
for (PathPoint p : path2) {
combined.add(new PathPoint(p.x, p.y, p.t, p.action));
}
bestOption = new Transition(combined, e1 + e2, endBat);
bestOption.reservation = new StationReservation(st, actualChargeStart, tDep);
}
}
break;
}
}
if (bestOption != null) {
if (bestOption.reservation != null) {
reservations.add(bestOption.reservation);
}
return bestOption;
}
return null;
}
static double getPathEnergy(double ax, double ay, List<PathPoint> path, double payload) {
double e = 0.0;
double cx = ax, cy = ay;
for (PathPoint p : path) {
if (p.action.equals("WAIT")) continue;
double d = dist(cx, cy, p.x, p.y);
e += d * (1.0 + payload);
cx = p.x; cy = p.y;
}
return e;
}
static boolean canReachAnyStation(double x, double y, double battery, double payload) {
double energyToWH = dist(x, y, warehouseX, warehouseY) * (1.0 + payload);
if (energyToWH <= battery + EPS) return true;
for (Station s : stations) {
double energyToSt = dist(x, y, s.x, s.y) * (1.0 + payload);
if (energyToSt <= battery + EPS) return true;
}
return false;
}
static double getActualChargeStartTime(Station st, double tArr, double chargeDuration, List<StationReservation> localReservations) {
if (chargeDuration < EPS) return tArr;
double t = tArr;
List<double[]> allRes = new ArrayList<>();
for (double[] r : st.reservations) allRes.add(r);
for (StationReservation sr : localReservations) {
if (sr.station == st) {
allRes.add(new double[]{sr.tStart, sr.tEnd});
}
}
for (int iter = 0; iter < 1000; iter++) {
int concurrent = 0;
double nextFree = Double.MAX_VALUE;
for (double[] res : allRes) {
if (res[0] < t + chargeDuration - EPS && res[1] > t + EPS) {
concurrent++;
if (res[1] < nextFree) nextFree = res[1];
}
}
if (concurrent < st.slots) return t;
if (nextFree == Double.MAX_VALUE) {
t += 1.0;
} else {
t = nextFree;
}
}
return t;
}
static boolean isNFZNearSegment(double ax, double ay, double bx, double by, NFZ nfz) {
double margin = 30.0;
double seg_xmin = Math.min(ax, bx) - margin;
double seg_xmax = Math.max(ax, bx) + margin;
double seg_ymin = Math.min(ay, by) - margin;
double seg_ymax = Math.max(ay, by) + margin;
return !(nfz.xmax < seg_xmin || nfz.xmin > seg_xmax || nfz.ymax < seg_ymin || nfz.ymin > seg_ymax);
}
static List<PathPoint> findShortestPath(double ax, double ay, double bx, double by, double tStart, double payload) {
pathfindCalls++;
double directDist = dist(ax, ay, bx, by);
boolean directClear = true;
for (NFZ nfz : nfzList) {
if (intersects(ax, ay, tStart, bx, by, tStart + directDist, nfz)) {
directClear = false;
break;
}
}
if (directClear) {
directClearCalls++;
List<PathPoint> result = new ArrayList<>();
result.add(new PathPoint(ax, ay, tStart, "WAYPOINT"));
result.add(new PathPoint(bx, by, tStart + directDist, "WAYPOINT"));
return result;
}
dijkstraCalls++;
long t0 = System.currentTimeMillis();
List<NFZ> activeNFZs = new ArrayList<>();
for (NFZ nfz : nfzList) {
if (isNFZNearSegment(ax, ay, bx, by, nfz)) {
activeNFZs.add(nfz);
}
}
List<Node> nodes = new ArrayList<>();
nodes.add(new Node(ax, ay));
nodes.add(new Node(bx, by));
for (Node node : fixedNodes) {
if (node.parentNFZ != null && activeNFZs.contains(node.parentNFZ)) {
nodes.add(node);
}
}
int n = nodes.size();
double[] distVal = new double[n];
double[] arrivalTime = new double[n];
int[] parent = new int[n];
boolean[] visited = new boolean[n];
Arrays.fill(distVal, Double.MAX_VALUE);
Arrays.fill(arrivalTime, Double.MAX_VALUE);
Arrays.fill(parent, -1);
distVal[0] = 0.0;
arrivalTime[0] = tStart;
for (int i = 0; i < n; i++) {
if (System.currentTimeMillis() - startTime > TIME_LIMIT_MS) {
dijkstraTimeMs += (System.currentTimeMillis() - t0);
return null;
}
int u = -1;
double minF = Double.MAX_VALUE;
for (int j = 0; j < n; j++) {
if (!visited[j] && distVal[j] < Double.MAX_VALUE) {
double f = distVal[j] + dist(nodes.get(j).x, nodes.get(j).y, bx, by) * (0.1 * (1.0 + payload) + 0.05);
if (f < minF) {
minF = f;
u = j;
}
}
}
if (u == -1 || u == 1) break;
visited[u] = true;
Node nu = nodes.get(u);
double tu = arrivalTime[u];
for (int v = 0; v < n; v++) {
if (visited[v]) continue;
Node nv = nodes.get(v);
double d = dist(nu.x, nu.y, nv.x, nv.y);
if (d < EPS) continue;
double waitTime = getMinWaitTime(nu, nv, tu, activeNFZs);
if (waitTime < 0) continue;
double tv = tu + waitTime + d;
double cost = d * (0.1 * (1.0 + payload) + 0.05) + 0.05 * waitTime;
if (distVal[u] + cost < distVal[v]) {
distVal[v] = distVal[u] + cost;
arrivalTime[v] = tv;
parent[v] = u;
}
}
}
if (distVal[1] == Double.MAX_VALUE) {
dijkstraTimeMs += (System.currentTimeMillis() - t0);
return null;
}
List<Integer> pathIndices = new ArrayList<>();
int curr = 1;
while (curr != -1) {
pathIndices.add(curr);
curr = parent[curr];
}
Collections.reverse(pathIndices);
List<PathPoint> detailedPath = new ArrayList<>();
double cx = ax, cy = ay, ct = tStart;
for (int i = 1; i < pathIndices.size(); i++) {
int uIdx = pathIndices.get(i - 1);
int vIdx = pathIndices.get(i);
Node nv = nodes.get(vIdx);
double d = dist(cx, cy, nv.x, nv.y);
double waitTime = getMinWaitTime(nodes.get(uIdx), nv, ct, activeNFZs);
if (waitTime > EPS) {
detailedPath.add(new PathPoint(cx, cy, ct + waitTime, "WAIT"));
ct += waitTime;
}
ct += d;
detailedPath.add(new PathPoint(nv.x, nv.y, ct, "WAYPOINT"));
cx = nv.x;
cy = nv.y;
}
dijkstraTimeMs += (System.currentTimeMillis() - t0);
return detailedPath;
}
static double getMinWaitTime(Node nu, Node nv, double tStart, List<NFZ> activeNFZs) {
double t = tStart;
double d = dist(nu.x, nu.y, nv.x, nv.y);
List<NFZ> listToCheck;
if (nu.id >= 0 && nv.id >= 0) {
String key = nu.id + "," + nv.id;
listToCheck = intersectingNFZs.get(key);
if (listToCheck == null) {
listToCheck = new ArrayList<>();
for (NFZ nfz : activeNFZs) {
if (intersectsGeom(nu.x, nu.y, nv.x, nv.y, nfz)) {
listToCheck.add(nfz);
}
}
intersectingNFZs.put(key, listToCheck);
}
} else {
listToCheck = activeNFZs;
}
for (int iter = 0; iter < 100; iter++) {
NFZ blocker = null;
double earliestEnd = Double.MAX_VALUE;
for (NFZ nfz : listToCheck) {
if (intersects(nu.x, nu.y, t, nv.x, nv.y, t + d, nfz)) {
if (nfz.tEnd < earliestEnd) {
earliestEnd = nfz.tEnd;
blocker = nfz;
}
}
}
if (blocker == null) return t - tStart;
t = blocker.tEnd + 0.01;
if (t > 1e6) return -1;
}
return -1;
}
static boolean intersectsGeom(double x1, double y1, double x2, double y2, NFZ nfz) {
double dt = dist(x1, y1, x2, y2);
return intersects(x1, y1, nfz.tStart, x2, y2, nfz.tStart + dt, nfz);
}
static boolean intersects(double x1, double y1, double t1, double x2, double y2, double t2, NFZ nfz) {
if (Math.max(t1, nfz.tStart) > Math.min(t2, nfz.tEnd) + EPS) return false;
double seg_xmin = Math.min(x1, x2);
double seg_xmax = Math.max(x1, x2);
double seg_ymin = Math.min(y1, y2);
double seg_ymax = Math.max(y1, y2);
if (seg_xmax < nfz.xmin - EPS || seg_xmin > nfz.xmax + EPS ||
seg_ymax < nfz.ymin - EPS || seg_ymin > nfz.ymax + EPS) {
return false;
}
if (nfz.shape.equals("circle"))
return intersectsCircle(x1, y1, t1, x2, y2, t2, nfz);
else
return intersectsRect(x1, y1, t1, x2, y2, t2, nfz);
}
static boolean intersectsCircle(double x1, double y1, double t1, double x2, double y2, double t2, NFZ nfz) {
double oStart = Math.max(t1, nfz.tStart);
double oEnd = Math.min(t2, nfz.tEnd);
if (oStart > oEnd + EPS) return false;
double totalT = t2 - t1;
if (totalT < EPS) {
return dist(x1, y1, nfz.cx, nfz.cy) <= nfz.radius + EPS;
}
double fStart = (oStart - t1) / totalT;
double fEnd = (oEnd - t1) / totalT;
double dx = x2 - x1;
double dy = y2 - y1;
double a = dx * dx + dy * dy;
double b = 2.0 * ((x1 - nfz.cx) * dx + (y1 - nfz.cy) * dy);
double c = (x1 - nfz.cx) * (x1 - nfz.cx) + (y1 - nfz.cy) * (y1 - nfz.cy) - nfz.radius * nfz.radius;
if (a < EPS) return c <= EPS;
double disc = b * b - 4.0 * a * c;
if (disc < 0) return false;
double f1 = (-b - Math.sqrt(disc)) / (2.0 * a);
double f2 = (-b + Math.sqrt(disc)) / (2.0 * a);
return Math.max(f1, fStart) <= Math.min(f2, fEnd) + EPS;
}
static boolean intersectsRect(double x1, double y1, double t1, double x2, double y2, double t2, NFZ nfz) {
double oStart = Math.max(t1, nfz.tStart);
double oEnd = Math.min(t2, nfz.tEnd);
if (oStart > oEnd + EPS) return false;
double totalT = t2 - t1;
if (totalT < EPS) {
return x1 >= nfz.xmin - EPS && x1 <= nfz.xmax + EPS &&
y1 >= nfz.ymin - EPS && y1 <= nfz.ymax + EPS;
}
double fStart = (oStart - t1) / totalT;
double fEnd = (oEnd - t1) / totalT;
double dx = x2 - x1;
double dy = y2 - y1;
double tEnter = 0.0;
double tLeave = 1.0;
double p, q;
p = -dx; q = x1 - nfz.xmin;
if (Math.abs(p) < EPS) { if (q < -EPS) return false; }
else { double tVal = q / p; if (p < 0) tEnter = Math.max(tEnter, tVal); else tLeave = Math.min(tLeave, tVal); }
p = dx; q = nfz.xmax - x1;
if (Math.abs(p) < EPS) { if (q < -EPS) return false; }
else { double tVal = q / p; if (p < 0) tEnter = Math.max(tEnter, tVal); else tLeave = Math.min(tLeave, tVal); }
p = -dy; q = y1 - nfz.ymin;
if (Math.abs(p) < EPS) { if (q < -EPS) return false; }
else { double tVal = q / p; if (p < 0) tEnter = Math.max(tEnter, tVal); else tLeave = Math.min(tLeave, tVal); }
p = dy; q = nfz.ymax - y1;
if (Math.abs(p) < EPS) { if (q < -EPS) return false; }
else { double tVal = q / p; if (p < 0) tEnter = Math.max(tEnter, tVal); else tLeave = Math.min(tLeave, tVal); }
if (tEnter > tLeave + EPS) return false;
return Math.max(tEnter, fStart) <= Math.min(tLeave, fEnd) + EPS;
}
static double dist(double x1, double y1, double x2, double y2) {
double dx = x1 - x2;
double dy = y1 - y2;
return Math.sqrt(dx * dx + dy * dy);
}
}