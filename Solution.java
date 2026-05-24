import java.io.*;
import java.util.*;
import com.google.gson.*;
public class Solution {
static final double BATTERY_MAX=500.0,CHARGE_RATE=2.0,EPS=1e-9;
static double mapWidth,mapHeight,warehouseX,warehouseY;
static List<NFZ> nfzList=new ArrayList<>();
static List<Station> stations=new ArrayList<>();
static List<Node> fixedNodes=new ArrayList<>();
static List<NFZ>[][] nfzEdgeCache;
static long startTime;
static final long TIME_LIMIT_MS=1600;
static class Delivery{
String id;double x,y,weight,deadline,distToWarehouse,tempDist;
boolean done=false;
Delivery(String id,double x,double y,double weight,double deadline){
this.id=id;this.x=x;this.y=y;this.weight=weight;this.deadline=deadline;
this.distToWarehouse=dist(x,y,warehouseX,warehouseY);}
}
static class Drone{
String id;double maxPayload,time=0.0;
List<PathPoint> path=new ArrayList<>();
Drone(String id,double maxPayload){this.id=id;this.maxPayload=maxPayload;}
}
static class NFZ{
String shape;double cx,cy,radius,xmin,ymin,xmax,ymax,tStart,tEnd;
}
static class Station{
double x,y;int slots;List<double[]> reservations=new ArrayList<>();
Station(double x,double y,int slots){this.x=x;this.y=y;this.slots=slots;}
}
static class PathPoint{
double x,y,t;String action,deliveryId;List<String> deliveryIds;
PathPoint(double x,double y,double t,String action){this.x=x;this.y=y;this.t=t;this.action=action;}
}
static class Node{
double x,y;int id=-1;NFZ parentNFZ=null;
Node(double x,double y){this.x=x;this.y=y;}
Node(double x,double y,int id,NFZ parentNFZ){this.x=x;this.y=y;this.id=id;this.parentNFZ=parentNFZ;}
}
static class TripPlan{
List<Delivery> batch;List<PathPoint> points;double endTime,loadBalanceScore,netValue,totalEnergy;
List<StationReservation> reservations;
TripPlan(List<Delivery> batch,List<PathPoint> points,double endTime,double lbs,double nv,double te,List<StationReservation> res){
this.batch=batch;this.points=points;this.endTime=endTime;this.loadBalanceScore=lbs;this.netValue=nv;this.totalEnergy=te;this.reservations=res;}
}
static class StationReservation{
Station station;double tStart,tEnd;
StationReservation(Station s,double ts,double te){station=s;tStart=ts;tEnd=te;}
}
static class Transition{
List<PathPoint> points;double energyConsumed,endBattery;StationReservation reservation;
Transition(List<PathPoint> p,double e,double b){points=p;energyConsumed=e;endBattery=b;}
}

public static void main(String[] args) throws Exception {
startTime=System.currentTimeMillis();
BufferedReader br=new BufferedReader(new InputStreamReader(System.in));
StringBuilder sb=new StringBuilder();char[] buf=new char[65536];int len;
while((len=br.read(buf))!=-1)sb.append(buf,0,len);
JsonObject input=JsonParser.parseString(sb.toString()).getAsJsonObject();
JsonArray mapSize=input.getAsJsonArray("map_size");
mapWidth=mapSize.get(0).getAsDouble();mapHeight=mapSize.get(1).getAsDouble();
warehouseX=mapWidth/2.0;warehouseY=mapHeight/2.0;
JsonArray dronesArr=input.getAsJsonArray("drones");
JsonArray deliveriesArr=input.getAsJsonArray("deliveries");
JsonArray nfzArr=input.has("no_fly_zones")?input.getAsJsonArray("no_fly_zones"):new JsonArray();
JsonArray stArr=input.has("charging_stations")?input.getAsJsonArray("charging_stations"):new JsonArray();
for(JsonElement e:nfzArr){
JsonObject o=e.getAsJsonObject();NFZ n=new NFZ();
n.shape=o.get("shape").getAsString();n.tStart=o.get("T_start").getAsDouble();n.tEnd=o.get("T_end").getAsDouble();
if(n.shape.equals("circle")){
JsonArray c=o.getAsJsonArray("center");n.cx=c.get(0).getAsDouble();n.cy=c.get(1).getAsDouble();
n.radius=o.get("radius").getAsDouble();n.xmin=n.cx-n.radius;n.xmax=n.cx+n.radius;n.ymin=n.cy-n.radius;n.ymax=n.cy+n.radius;
}else{if(o.has("corners")){
JsonArray c=o.getAsJsonArray("corners");JsonArray a=c.get(0).getAsJsonArray();JsonArray b=c.get(1).getAsJsonArray();
n.xmin=Math.min(a.get(0).getAsDouble(),b.get(0).getAsDouble());n.xmax=Math.max(a.get(0).getAsDouble(),b.get(0).getAsDouble());
n.ymin=Math.min(a.get(1).getAsDouble(),b.get(1).getAsDouble());n.ymax=Math.max(a.get(1).getAsDouble(),b.get(1).getAsDouble());
}else if(o.has("bottom_left")&&o.has("top_right")){
JsonArray bl=o.getAsJsonArray("bottom_left");JsonArray tr=o.getAsJsonArray("top_right");
n.xmin=bl.get(0).getAsDouble();n.ymin=bl.get(1).getAsDouble();n.xmax=tr.get(0).getAsDouble();n.ymax=tr.get(1).getAsDouble();
}}nfzList.add(n);}
for(JsonElement e:stArr){JsonObject o=e.getAsJsonObject();int slots=o.has("slots")?o.get("slots").getAsInt():1;
stations.add(new Station(o.get("x").getAsDouble(),o.get("y").getAsDouble(),slots));}
List<Delivery> allDeliveries=new ArrayList<>();
for(JsonElement e:deliveriesArr){JsonObject o=e.getAsJsonObject();
allDeliveries.add(new Delivery(o.get("id").getAsString(),o.get("x").getAsDouble(),o.get("y").getAsDouble(),
o.get("weight").getAsDouble(),o.get("deadline").getAsDouble()));}
List<Drone> drones=new ArrayList<>();
for(JsonElement e:dronesArr){JsonObject o=e.getAsJsonObject();
drones.add(new Drone(o.get("id").getAsString(),o.get("max_payload").getAsDouble()));}
fixedNodes=new ArrayList<>();double delta=0.1;int nc=0;
for(NFZ nfz:nfzList){
if(nfz.shape.equals("circle")){
for(int angle=0;angle<360;angle+=90){double rad=Math.toRadians(angle),r=nfz.radius+delta;
fixedNodes.add(new Node(nfz.cx+Math.cos(rad)*r,nfz.cy+Math.sin(rad)*r,nc++,nfz));}
}else{fixedNodes.add(new Node(nfz.xmin-delta,nfz.ymin-delta,nc++,nfz));fixedNodes.add(new Node(nfz.xmin-delta,nfz.ymax+delta,nc++,nfz));
fixedNodes.add(new Node(nfz.xmax+delta,nfz.ymin-delta,nc++,nfz));fixedNodes.add(new Node(nfz.xmax+delta,nfz.ymax+delta,nc++,nfz));}}
for(Station s:stations)fixedNodes.add(new Node(s.x,s.y,nc++,null));
fixedNodes.add(new Node(warehouseX,warehouseY,nc++,null));
for(Node n:fixedNodes){n.x=Math.max(0.1,Math.min(mapWidth-0.1,n.x));n.y=Math.max(0.1,Math.min(mapHeight-0.1,n.y));}
nfzEdgeCache=new List[nc][nc];
scheduleAll(allDeliveries,drones);
JsonArray manifest=new JsonArray();
for(Drone d:drones){if(d.path.isEmpty())continue;
JsonObject droneObj=new JsonObject();droneObj.addProperty("drone_id",d.id);
JsonArray pathArray=new JsonArray();
for(PathPoint pp:d.path){JsonObject pt=new JsonObject();pt.addProperty("x",pp.x);pt.addProperty("y",pp.y);pt.addProperty("t",pp.t);pt.addProperty("action",pp.action);
if(pp.deliveryIds!=null){JsonArray ids=new JsonArray();for(String s:pp.deliveryIds)ids.add(s);pt.add("delivery_ids",ids);}
if(pp.deliveryId!=null)pt.addProperty("delivery_id",pp.deliveryId);pathArray.add(pt);}
droneObj.add("path",pathArray);manifest.add(droneObj);}
JsonObject out=new JsonObject();out.add("flight_manifest",manifest);
System.out.println(new Gson().toJson(out));}

static void scheduleAll(List<Delivery> deliveries,List<Drone> drones){
// Fast O(1) pre-filter
List<Delivery> feasible=new ArrayList<>(deliveries.size());
for(Delivery d:deliveries){
if(d.distToWarehouse>d.deadline)continue;
if(d.distToWarehouse*(1.0+d.weight)>BATTERY_MAX)continue;
feasible.add(d);}
// Sort by urgency: tightest deadline-slack first
feasible.sort(Comparator.comparingDouble(d->d.deadline-d.distToWarehouse));
boolean fastMode=feasible.size()>300;
if(fastMode){
scheduleFast(feasible,drones);
}else{
scheduleBatch(feasible,drones);}
optimizeFinalReturns(drones,feasible);rebuildReservations(drones);}

// ULTRA FAST: pure single-delivery greedy for large inputs
static class BatchWithScore{List<Delivery> batch;double est;BatchWithScore(List<Delivery>b,double s){batch=b;est=s;}}
static void scheduleFast(List<Delivery> deliveries,List<Drone> drones){
int n=deliveries.size();
boolean anyProgress=true;
while(anyProgress&&System.currentTimeMillis()-startTime<TIME_LIMIT_MS-50){
anyProgress=false;
for(Drone drone:drones){
if(System.currentTimeMillis()-startTime>TIME_LIMIT_MS-50)break;
for(int i=0;i<n;i++){
Delivery d=deliveries.get(i);
if(d.done)continue;
if(d.weight>drone.maxPayload+EPS)continue;
if(drone.time+d.distToWarehouse>d.deadline)continue;
TripPlan plan=attemptTrip(Collections.singletonList(d),drone.time);
if(plan!=null&&plan.netValue>0){
d.done=true;anyProgress=true;
drone.path.addAll(plan.points);drone.time=plan.endTime;
for(StationReservation r:plan.reservations){
r.station.reservations.add(new double[]{r.tStart,r.tEnd});
r.station.reservations.sort(Comparator.comparingDouble(a->a[0]));}
break;}}}}}

static void scheduleBatch(List<Delivery> deliveries,List<Drone> drones){
int prevDoneCount=-1;int totalDone=0;int maxIter=deliveries.size()*3;
while(totalDone!=prevDoneCount&&maxIter-->0){
if(System.currentTimeMillis()-startTime>TIME_LIMIT_MS-100)break;
prevDoneCount=totalDone;
for(Drone drone:drones){
if(System.currentTimeMillis()-startTime>TIME_LIMIT_MS-100)break;
TripPlan bestTrip=null;double bestScore=-Double.MAX_VALUE;Set<String> skipped=new HashSet<>();
while(true){
if(System.currentTimeMillis()-startTime>TIME_LIMIT_MS-100)break;
Delivery seed=null;
for(Delivery d:deliveries){
if(d.done||skipped.contains(d.id)||d.weight>drone.maxPayload+EPS)continue;
if(drone.time+d.distToWarehouse<=d.deadline){seed=d;break;}}
if(seed==null)break;
TripPlan sp=attemptTrip(Collections.singletonList(seed),drone.time);
if(sp==null){skipped.add(seed.id);continue;}
List<Delivery> cands=new ArrayList<>();cands.add(seed);
for(Delivery d:deliveries){
if(d==seed||d.done||skipped.contains(d.id)||d.weight>drone.maxPayload+EPS)continue;
if(drone.time+d.distToWarehouse>d.deadline)continue;
d.tempDist=dist(seed.x,seed.y,d.x,d.y);
if(d.tempDist<=200.0)cands.add(d);}
cands.sort((a,b)->Double.compare(a.tempDist,b.tempDist));
if(cands.size()>31)cands=cands.subList(0,31);
List<List<Delivery>> batches=getCandidateBatches(cands,drone.maxPayload);
List<BatchWithScore> sorted=new ArrayList<>();
for(List<Delivery> b:batches){double e=estimateBatchScore(b,drone.time);if(e>-Double.MAX_VALUE+EPS)sorted.add(new BatchWithScore(b,e));}
sorted.sort((a,b)->Double.compare(b.est,a.est));
for(int idx=0;idx<Math.min(1,sorted.size());idx++){
List<Delivery> batch=sorted.get(idx).batch;
TripPlan plan=findBestPermutation(batch,drone.time);
if(plan!=null&&plan.netValue>0){
double urg=0;for(Delivery d:batch)urg+=1.0/Math.max(1.0,d.deadline-drone.time);
double sc=plan.batch.size()*100.0+urg*1000-plan.totalEnergy*0.1;
if(bestTrip==null||sc>bestScore){bestTrip=plan;bestScore=sc;}}}
if(bestTrip!=null)break;else skipped.add(seed.id);}
if(bestTrip!=null){
for(Delivery d:bestTrip.batch){d.done=true;totalDone++;}
drone.path.addAll(bestTrip.points);drone.time=bestTrip.endTime;
for(StationReservation r:bestTrip.reservations){
r.station.reservations.add(new double[]{r.tStart,r.tEnd});
r.station.reservations.sort(Comparator.comparingDouble(a->a[0]));}}}}}
static void optimizeFinalReturns(List<Drone> drones,List<Delivery> allDeliveries){
Map<String,Double> dw=new HashMap<>();for(Delivery d:allDeliveries)dw.put(d.id,d.weight);
for(Drone d:drones){if(d.path.isEmpty())continue;
int li=-1;for(int i=0;i<d.path.size();i++)if(d.path.get(i).action.equals("DELIVER"))li=i;
if(li==-1){d.path.clear();d.time=0.0;continue;}
PathPoint pp=d.path.get(li);double bat=getBatteryAt(d.path,li,dw);
double bestScore=Double.MAX_VALUE;Transition best=null;List<StationReservation> tr=new ArrayList<>();
Transition tw=makeTransition(pp.x,pp.y,warehouseX,warehouseY,pp.t,bat,0.0,0.0,tr);
if(tw!=null){double et=tw.points.get(tw.points.size()-1).t;bestScore=tw.energyConsumed*0.1+et*0.05;best=tw;}
for(Station st:stations){tr=new ArrayList<>();Transition ts=makeTransition(pp.x,pp.y,st.x,st.y,pp.t,bat,0.0,0.0,tr);
if(ts!=null){double et=ts.points.get(ts.points.size()-1).t;double sc=ts.energyConsumed*0.1+et*0.05;if(sc<bestScore){bestScore=sc;best=ts;}}}
if(best!=null){List<PathPoint> np=new ArrayList<>();for(int i=0;i<=li;i++)np.add(d.path.get(i));
np.addAll(best.points);np.get(np.size()-1).action="RETURN";d.path=np;d.time=np.get(np.size()-1).t;}}}

static double getBatteryAt(List<PathPoint> path,int limitIdx,Map<String,Double> dw){
double bat=500.0,cx=warehouseX,cy=warehouseY,pw=0.0;
for(int i=0;i<=limitIdx;i++){PathPoint pt=path.get(i);double d=dist(cx,cy,pt.x,pt.y);
if(d>1e-9)bat-=d*(1.0+pw);
if(pt.action.equals("PICKUP")){bat=500.0;if(pt.deliveryIds!=null)for(String id:pt.deliveryIds)pw+=dw.getOrDefault(id,0.0);}
else if(pt.action.equals("DELIVER")&&pt.deliveryId!=null)pw-=dw.getOrDefault(pt.deliveryId,0.0);
else if(pt.action.equals("CHARGE_COMPLETE")){double cs=pt.t;for(int j=i-1;j>=0;j--)if(path.get(j).action.equals("CHARGE")){cs=path.get(j).t;break;}bat=Math.min(500.0,bat+(pt.t-cs)*2.0);}
cx=pt.x;cy=pt.y;}return Math.max(0.0,bat);}

static void rebuildReservations(List<Drone> drones){
for(Station st:stations)st.reservations.clear();
for(Drone d:drones)for(int i=0;i<d.path.size();i++){PathPoint pt=d.path.get(i);
if(pt.action.equals("CHARGE")){double ts=pt.t,te=ts;
for(int j=i+1;j<d.path.size();j++)if(d.path.get(j).action.equals("CHARGE_COMPLETE")&&dist(d.path.get(j).x,d.path.get(j).y,pt.x,pt.y)<1.0){te=d.path.get(j).t;break;}
for(Station st:stations)if(dist(st.x,st.y,pt.x,pt.y)<1.0){st.reservations.add(new double[]{ts,te});break;}}}}

static void addBatchIfUnique(List<List<Delivery>> batches,List<Delivery> batch){
if(batch.isEmpty())return;for(List<Delivery> e:batches)if(sameBatch(e,batch))return;batches.add(batch);}

static List<List<Delivery>> getCandidateBatches(List<Delivery> rem,double maxPay){
List<List<Delivery>> batches=new ArrayList<>();
List<Delivery> b1=new ArrayList<>();double w1=0;
for(Delivery d:rem)if(w1+d.weight<=maxPay+EPS){b1.add(d);w1+=d.weight;}
addBatchIfUnique(batches,b1);
List<Delivery> b2=new ArrayList<>();double w2=0;boolean[] u2=new boolean[rem.size()];
double cx=warehouseX,cy=warehouseY;
while(true){int bi=-1;double bd=Double.MAX_VALUE;
for(int i=0;i<rem.size();i++){if(u2[i])continue;if(w2+rem.get(i).weight>maxPay+EPS)continue;double dd=dist(cx,cy,rem.get(i).x,rem.get(i).y);if(dd<bd){bd=dd;bi=i;}}
if(bi==-1)break;Delivery d=rem.get(bi);b2.add(d);w2+=d.weight;cx=d.x;cy=d.y;u2[bi]=true;}
addBatchIfUnique(batches,b2);
int lim=Math.min(rem.size(),8);
for(int i=0;i<lim;i++){Delivery ctr=rem.get(i);List<Delivery> cl=new ArrayList<>();cl.add(ctr);double w=ctr.weight;
for(Delivery o:rem){if(o==ctr)continue;o.tempDist=dist(ctr.x,ctr.y,o.x,o.y);}
List<Delivery> oth=new ArrayList<>(rem);oth.remove(ctr);oth.sort((a,b)->Double.compare(a.tempDist,b.tempDist));
for(Delivery o:oth)if(w+o.weight<=maxPay+EPS){cl.add(o);w+=o.weight;}
addBatchIfUnique(batches,cl);}
if(rem.size()<=8){int n=rem.size();for(int mask=1;mask<(1<<n);mask++){List<Delivery> sub=new ArrayList<>();double w=0;boolean v=true;
for(int i=0;i<n;i++)if(((mask>>i)&1)==1){if(w+rem.get(i).weight>maxPay+EPS){v=false;break;}sub.add(rem.get(i));w+=rem.get(i).weight;}
if(v&&!sub.isEmpty())addBatchIfUnique(batches,sub);}}
return batches;}

static boolean sameBatch(List<Delivery> a,List<Delivery> b){if(a.size()!=b.size())return false;Set<String> sa=new HashSet<>();for(Delivery d:a)sa.add(d.id);for(Delivery d:b)if(!sa.contains(d.id))return false;return true;}

static double estimateBatchScore(List<Delivery> batch,double tStart){
List<Delivery> pool=new ArrayList<>(batch);double cx=warehouseX,cy=warehouseY;
List<Delivery> ord=new ArrayList<>();
while(!pool.isEmpty()){int bi=0;double bd=Double.MAX_VALUE;for(int i=0;i<pool.size();i++){double d=dist(cx,cy,pool.get(i).x,pool.get(i).y);if(d<bd){bd=d;bi=i;}}Delivery d=pool.remove(bi);ord.add(d);cx=d.x;cy=d.y;}
double t=tStart,pl=0,ee=0,curX=warehouseX,curY=warehouseY;for(Delivery d:ord)pl+=d.weight;
for(Delivery d:ord){double dd=dist(curX,curY,d.x,d.y);t+=dd;if(t>d.deadline)return -Double.MAX_VALUE;ee+=dd*(1.0+pl);curX=d.x;curY=d.y;pl-=d.weight;}
double rd=dist(curX,curY,warehouseX,warehouseY);ee+=rd;
if(ee>BATTERY_MAX){double md=Double.MAX_VALUE;for(Station s:stations){double det=dist(curX,curY,s.x,s.y)+dist(s.x,s.y,warehouseX,warehouseY)-rd;if(det<md)md=det;}if(md==Double.MAX_VALUE)return -Double.MAX_VALUE;ee+=md;}
double ub=0;for(Delivery d:ord)ub+=1.0/Math.max(1.0,d.deadline-tStart);
return ord.size()*100.0+ub*1000-ee*0.1;}

static TripPlan findBestPermutation(List<Delivery> batch,double tStart){
for(Delivery d:batch)if(tStart+d.distToWarehouse>d.deadline)return null;
TripPlan best=null;
if(batch.size()>3){
best=attemptTrip(euclidean2Opt(batch),tStart);
if(best==null)best=attemptTrip(weightSheddingOrder(batch),tStart);
}else{
List<List<Delivery>> perms=new ArrayList<>();generatePermutations(new ArrayList<>(batch),0,perms);
for(List<Delivery> p:perms){TripPlan pl=attemptTrip(p,tStart);if(pl!=null&&(best==null||pl.loadBalanceScore>best.loadBalanceScore))best=pl;}}
if(best==null&&batch.size()>1){Delivery t=null;for(Delivery d:batch)if(t==null||d.deadline<t.deadline)t=d;List<Delivery> r=new ArrayList<>(batch);r.remove(t);return findBestPermutation(r,tStart);}
return best;}

static List<Delivery> nearestNeighborOrder(List<Delivery> b){List<Delivery> res=new ArrayList<>();List<Delivery> pool=new ArrayList<>(b);double cx=warehouseX,cy=warehouseY;
while(!pool.isEmpty()){int bi=0;double bd=Double.MAX_VALUE;for(int i=0;i<pool.size();i++){double d=dist(cx,cy,pool.get(i).x,pool.get(i).y);if(d<bd){bd=d;bi=i;}}Delivery d=pool.remove(bi);res.add(d);cx=d.x;cy=d.y;}return res;}
static List<Delivery> euclidean2Opt(List<Delivery> b){List<Delivery> cur=nearestNeighborOrder(b);boolean imp=true;
while(imp){imp=false;double cd=getEuclideanTourLength(cur);for(int i=0;i<cur.size()-1;i++)for(int j=i+1;j<cur.size();j++){List<Delivery> nb=new ArrayList<>(cur);int l=i,r=j;while(l<r){Collections.swap(nb,l,r);l++;r--;}double nd=getEuclideanTourLength(nb);if(nd<cd-EPS){cur=nb;cd=nd;imp=true;}}}return cur;}
static double getEuclideanTourLength(List<Delivery> p){double d=0,cx=warehouseX,cy=warehouseY;for(Delivery dd:p){d+=dist(cx,cy,dd.x,dd.y);cx=dd.x;cy=dd.y;}d+=dist(cx,cy,warehouseX,warehouseY);return d;}
static List<Delivery> weightSheddingOrder(List<Delivery> b){List<Delivery> res=new ArrayList<>();List<Delivery> pool=new ArrayList<>(b);double cx=warehouseX,cy=warehouseY;
while(!pool.isEmpty()){int bi=0;double bs=Double.MAX_VALUE;for(int i=0;i<pool.size();i++){double d=dist(cx,cy,pool.get(i).x,pool.get(i).y);double s=d/(pool.get(i).weight+0.01);if(s<bs){bs=s;bi=i;}}Delivery d=pool.remove(bi);res.add(d);cx=d.x;cy=d.y;}return res;}
static void generatePermutations(List<Delivery> list,int k,List<List<Delivery>> res){
if(k==list.size()-1){res.add(new ArrayList<>(list));return;}
for(int i=k;i<list.size();i++){Collections.swap(list,i,k);generatePermutations(list,k+1,res);Collections.swap(list,k,i);}}

static TripPlan attemptTrip(List<Delivery> ordered,double tStart){
List<PathPoint> pts=new ArrayList<>();List<StationReservation> res=new ArrayList<>();
double payload=0;for(Delivery d:ordered)payload+=d.weight;
double cx=warehouseX,cy=warehouseY,t=tStart,bat=BATTERY_MAX,te=0;
PathPoint pk=new PathPoint(warehouseX,warehouseY,t,"PICKUP");pk.deliveryIds=new ArrayList<>();
for(Delivery d:ordered)pk.deliveryIds.add(d.id);pts.add(pk);
for(Delivery d:ordered){double pa=payload-d.weight;
Transition tr=makeTransition(cx,cy,d.x,d.y,t,bat,payload,pa,res);if(tr==null)return null;
double arr=tr.points.get(tr.points.size()-1).t;if(arr>d.deadline+EPS)return null;
PathPoint last=tr.points.get(tr.points.size()-1);last.action="DELIVER";last.deliveryId=d.id;
pts.addAll(tr.points);te+=tr.energyConsumed;bat=tr.endBattery;t=arr;cx=d.x;cy=d.y;payload-=d.weight;}
Transition ret=makeTransition(cx,cy,warehouseX,warehouseY,t,bat,0.0,0.0,res);if(ret==null)return null;
ret.points.get(ret.points.size()-1).action="RETURN";pts.addAll(ret.points);te+=ret.energyConsumed;t=ret.points.get(ret.points.size()-1).t;
double dur=t-tStart,lbs=ordered.size()*100.0-te*0.1-t*0.05,nv=ordered.size()*100.0-te*0.1-dur*0.05;
return new TripPlan(ordered,pts,t,lbs,nv,te,res);}

static Transition makeTransition(double ax,double ay,double bx,double by,double tS,double sBat,double pay,double payD,List<StationReservation> res){
double ed=dist(ax,ay,bx,by);
if(sBat>=ed*(1.0+pay)-EPS){List<PathPoint> dp=findShortestPath(ax,ay,bx,by,tS,pay);
if(dp!=null){double e=getPathEnergy(ax,ay,dp,pay);if(sBat-e>=-EPS){double rb=sBat-e;if(canReach(bx,by,rb,payD))return new Transition(dp,e,Math.max(0,rb));}}}
Transition best=null;double bat=Double.MAX_VALUE;
List<Station> cands=new ArrayList<>();
if(stations.size()<=2)cands.addAll(stations);
else{Station s1=null,s2=null;double d1=Double.MAX_VALUE,d2=Double.MAX_VALUE;
for(Station st:stations){double d=dist(ax,ay,st.x,st.y)+dist(st.x,st.y,bx,by);if(d<d1){d2=d1;s2=s1;d1=d;s1=st;}else if(d<d2){d2=d;s2=st;}}
if(s1!=null)cands.add(s1);if(s2!=null)cands.add(s2);}
for(Station st:cands){double dS=dist(ax,ay,st.x,st.y);boolean atSt=dS<1.0;
if(!atSt&&sBat<dS*(1.0+pay)-EPS)continue;
List<PathPoint> p1=null;double e1=0,aTC=tS,bAC=sBat;
if(!atSt){p1=findShortestPath(ax,ay,st.x,st.y,tS,pay);if(p1==null)continue;e1=getPathEnergy(ax,ay,p1,pay);if(sBat-e1<-EPS)continue;aTC=p1.get(p1.size()-1).t;bAC=sBat-e1;}
double dSB=dist(st.x,st.y,bx,by),dBW=dist(bx,by,warehouseX,warehouseY);
double mEB=dSB*(1.0+pay);if(mEB>BATTERY_MAX-EPS)continue;
double tgt=Math.min(BATTERY_MAX,mEB+dBW*(1.0+payD)+10.0);tgt=Math.max(tgt,mEB+10.0);tgt=Math.min(BATTERY_MAX,tgt);
for(int att=0;att<2;att++){
double cn=Math.max(0.0,tgt-bAC),ct=cn/CHARGE_RATE,acs=getActualChargeStartTime(st,aTC,ct,res),tDep=acs+ct;
List<PathPoint> p2=findShortestPath(st.x,st.y,bx,by,tDep,pay);if(p2==null)break;
double e2=getPathEnergy(st.x,st.y,p2,pay);if(tgt-e2<-EPS){tgt=Math.min(BATTERY_MAX,e2+dBW*(1.0+payD)+10.0);continue;}
double arr=p2.get(p2.size()-1).t,eb=tgt-e2;
if(eb>=-EPS&&canReach(bx,by,eb,payD)&&arr<bat){bat=arr;
List<PathPoint> cm=new ArrayList<>();double wt=acs-aTC;
if(atSt){if(wt>EPS)cm.add(new PathPoint(st.x,st.y,acs,"WAIT"));cm.add(new PathPoint(st.x,st.y,acs,"CHARGE"));}
else{if(wt>EPS)cm.add(new PathPoint(ax,ay,tS+wt,"WAIT"));for(PathPoint p:p1)cm.add(new PathPoint(p.x,p.y,p.t+wt,p.action));cm.add(new PathPoint(st.x,st.y,acs,"CHARGE"));}
cm.add(new PathPoint(st.x,st.y,tDep,"CHARGE_COMPLETE"));for(PathPoint p:p2)cm.add(new PathPoint(p.x,p.y,p.t,p.action));
best=new Transition(cm,e1+e2,eb);best.reservation=new StationReservation(st,acs,tDep);}break;}}
if(best!=null){if(best.reservation!=null)res.add(best.reservation);return best;}return null;}

static double getPathEnergy(double ax,double ay,List<PathPoint> path,double pay){
double e=0,cx=ax,cy=ay;for(PathPoint p:path){if(p.action.equals("WAIT"))continue;double d=dist(cx,cy,p.x,p.y);e+=d*(1.0+pay);cx=p.x;cy=p.y;}return e;}
static boolean canReach(double x,double y,double bat,double pay){
if(dist(x,y,warehouseX,warehouseY)*(1.0+pay)<=bat+EPS)return true;
for(Station s:stations)if(dist(x,y,s.x,s.y)*(1.0+pay)<=bat+EPS)return true;return false;}
static double getActualChargeStartTime(Station st,double tArr,double dur,List<StationReservation> lRes){
if(dur<EPS)return tArr;double t=tArr;
List<double[]> all=new ArrayList<>(st.reservations);for(StationReservation sr:lRes)if(sr.station==st)all.add(new double[]{sr.tStart,sr.tEnd});
for(int i=0;i<1000;i++){int cc=0;double nf=Double.MAX_VALUE;
for(double[] r:all)if(r[0]<t+dur-EPS&&r[1]>t+EPS){cc++;if(r[1]<nf)nf=r[1];}
if(cc<st.slots)return t;t=nf==Double.MAX_VALUE?t+1.0:nf;}return t;}
static boolean isNear(double ax,double ay,double bx,double by,NFZ nfz){
double m=30.0;return!(nfz.xmax<Math.min(ax,bx)-m||nfz.xmin>Math.max(ax,bx)+m||nfz.ymax<Math.min(ay,by)-m||nfz.ymin>Math.max(ay,by)+m);}
static List<PathPoint> findShortestPath(double ax,double ay,double bx,double by,double tS,double pay){
double dd=dist(ax,ay,bx,by);boolean clear=true;
for(NFZ nfz:nfzList)if(intersects(ax,ay,tS,bx,by,tS+dd,nfz)){clear=false;break;}
if(clear){List<PathPoint> r=new ArrayList<>();r.add(new PathPoint(ax,ay,tS,"WAYPOINT"));r.add(new PathPoint(bx,by,tS+dd,"WAYPOINT"));return r;}
if(System.currentTimeMillis()-startTime>TIME_LIMIT_MS)return null;
List<NFZ> active=new ArrayList<>();for(NFZ nfz:nfzList)if(isNear(ax,ay,bx,by,nfz))active.add(nfz);
List<Node> nodes=new ArrayList<>();nodes.add(new Node(ax,ay));nodes.add(new Node(bx,by));
for(Node nd:fixedNodes)if(nd.parentNFZ!=null&&active.contains(nd.parentNFZ))nodes.add(nd);
int n=nodes.size();double[] dv=new double[n],at=new double[n];int[] par=new int[n];boolean[] vis=new boolean[n];
Arrays.fill(dv,Double.MAX_VALUE);Arrays.fill(at,Double.MAX_VALUE);Arrays.fill(par,-1);dv[0]=0;at[0]=tS;
for(int i=0;i<n;i++){if(System.currentTimeMillis()-startTime>TIME_LIMIT_MS)return null;
int u=-1;double mf=Double.MAX_VALUE;
for(int j=0;j<n;j++)if(!vis[j]&&dv[j]<Double.MAX_VALUE){double f=dv[j]+dist(nodes.get(j).x,nodes.get(j).y,bx,by)*(0.1*(1.0+pay)+0.05);if(f<mf){mf=f;u=j;}}
if(u==-1||u==1)break;vis[u]=true;Node nu=nodes.get(u);double tu=at[u];
for(int v=0;v<n;v++){if(vis[v])continue;Node nv=nodes.get(v);double d=dist(nu.x,nu.y,nv.x,nv.y);if(d<EPS)continue;
double wt=getMinWaitTime(nu,nv,tu,active);if(wt<0)continue;
double tv=tu+wt+d,cost=d*(0.1*(1.0+pay)+0.05)+0.05*wt;
if(dv[u]+cost<dv[v]){dv[v]=dv[u]+cost;at[v]=tv;par[v]=u;}}}
if(dv[1]==Double.MAX_VALUE)return null;
List<Integer> pi=new ArrayList<>();int c=1;while(c!=-1){pi.add(c);c=par[c];}Collections.reverse(pi);
List<PathPoint> path=new ArrayList<>();double cx=ax,cy=ay,ct=tS;
for(int i=1;i<pi.size();i++){Node nv=nodes.get(pi.get(i));double d=dist(cx,cy,nv.x,nv.y);
double wt=getMinWaitTime(nodes.get(pi.get(i-1)),nv,ct,active);
if(wt>EPS){path.add(new PathPoint(cx,cy,ct+wt,"WAIT"));ct+=wt;}
ct+=d;path.add(new PathPoint(nv.x,nv.y,ct,"WAYPOINT"));cx=nv.x;cy=nv.y;}
return path;}
static double getMinWaitTime(Node nu,Node nv,double tS,List<NFZ> active){
double t=tS,d=dist(nu.x,nu.y,nv.x,nv.y);List<NFZ> chk;
if(nu.id>=0&&nv.id>=0){chk=nfzEdgeCache[nu.id][nv.id];
if(chk==null){chk=new ArrayList<>();for(NFZ nfz:active)if(intersectsGeom(nu.x,nu.y,nv.x,nv.y,nfz))chk.add(nfz);nfzEdgeCache[nu.id][nv.id]=chk;}
}else chk=active;
for(int i=0;i<100;i++){NFZ bl=null;double ee=Double.MAX_VALUE;
for(NFZ nfz:chk)if(intersects(nu.x,nu.y,t,nv.x,nv.y,t+d,nfz)&&nfz.tEnd<ee){ee=nfz.tEnd;bl=nfz;}
if(bl==null)return t-tS;t=bl.tEnd+0.01;if(t>1e6)return -1;}return -1;}
static boolean intersectsGeom(double x1,double y1,double x2,double y2,NFZ nfz){return intersects(x1,y1,nfz.tStart,x2,y2,nfz.tStart+dist(x1,y1,x2,y2),nfz);}
static boolean intersects(double x1,double y1,double t1,double x2,double y2,double t2,NFZ nfz){
if(Math.max(t1,nfz.tStart)>Math.min(t2,nfz.tEnd)+EPS)return false;
if(Math.max(x1,x2)<nfz.xmin-EPS||Math.min(x1,x2)>nfz.xmax+EPS||Math.max(y1,y2)<nfz.ymin-EPS||Math.min(y1,y2)>nfz.ymax+EPS)return false;
return nfz.shape.equals("circle")?iCircle(x1,y1,t1,x2,y2,t2,nfz):iRect(x1,y1,t1,x2,y2,t2,nfz);}
static boolean iCircle(double x1,double y1,double t1,double x2,double y2,double t2,NFZ n){
double oS=Math.max(t1,n.tStart),oE=Math.min(t2,n.tEnd);if(oS>oE+EPS)return false;
double tT=t2-t1;if(tT<EPS)return dist(x1,y1,n.cx,n.cy)<=n.radius+EPS;
double fS=(oS-t1)/tT,fE=(oE-t1)/tT,dx=x2-x1,dy=y2-y1;
double a=dx*dx+dy*dy,b=2.0*((x1-n.cx)*dx+(y1-n.cy)*dy),c=(x1-n.cx)*(x1-n.cx)+(y1-n.cy)*(y1-n.cy)-n.radius*n.radius;
if(a<EPS)return c<=EPS;double disc=b*b-4.0*a*c;if(disc<0)return false;
double f1=(-b-Math.sqrt(disc))/(2.0*a),f2=(-b+Math.sqrt(disc))/(2.0*a);
return Math.max(f1,fS)<=Math.min(f2,fE)+EPS;}
static boolean iRect(double x1,double y1,double t1,double x2,double y2,double t2,NFZ n){
double oS=Math.max(t1,n.tStart),oE=Math.min(t2,n.tEnd);if(oS>oE+EPS)return false;
double tT=t2-t1;if(tT<EPS)return x1>=n.xmin-EPS&&x1<=n.xmax+EPS&&y1>=n.ymin-EPS&&y1<=n.ymax+EPS;
double fS=(oS-t1)/tT,fE=(oE-t1)/tT,dx=x2-x1,dy=y2-y1,tEn=0,tLe=1,p,q;
p=-dx;q=x1-n.xmin;if(Math.abs(p)<EPS){if(q<-EPS)return false;}else{double tv=q/p;if(p<0)tEn=Math.max(tEn,tv);else tLe=Math.min(tLe,tv);}
p=dx;q=n.xmax-x1;if(Math.abs(p)<EPS){if(q<-EPS)return false;}else{double tv=q/p;if(p<0)tEn=Math.max(tEn,tv);else tLe=Math.min(tLe,tv);}
p=-dy;q=y1-n.ymin;if(Math.abs(p)<EPS){if(q<-EPS)return false;}else{double tv=q/p;if(p<0)tEn=Math.max(tEn,tv);else tLe=Math.min(tLe,tv);}
p=dy;q=n.ymax-y1;if(Math.abs(p)<EPS){if(q<-EPS)return false;}else{double tv=q/p;if(p<0)tEn=Math.max(tEn,tv);else tLe=Math.min(tLe,tv);}
if(tEn>tLe+EPS)return false;return Math.max(tEn,fS)<=Math.min(tLe,fE)+EPS;}
static double dist(double x1,double y1,double x2,double y2){double dx=x1-x2,dy=y1-y2;return Math.sqrt(dx*dx+dy*dy);}
}
