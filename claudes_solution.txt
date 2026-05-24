import java.io.*;
import java.util.*;
import com.google.gson.*;

public class Solution {

    static final double BATTERY_MAX = 500.0;
    static final double CHARGE_RATE = 2.0;
    static final double SPEED       = 1.0;
    static final double EPS         = 1e-9;

    static double warehouseX, warehouseY;
    static List<NFZ>     nfzList  = new ArrayList<>();
    static List<Station> stations = new ArrayList<>();

    // ── Data types ──────────────────────────────────────────────────────────
    static class Delivery {
        String id; double x, y, weight, deadline;
        Delivery(String id,double x,double y,double weight,double deadline){
            this.id=id;this.x=x;this.y=y;this.weight=weight;this.deadline=deadline;
        }
    }
    static class NFZ {
        String shape;
        double cx,cy,radius,xmin,ymin,xmax,ymax,tStart,tEnd;
    }
    static class Station { double x,y; int slots; Station(double x,double y,int s){this.x=x;this.y=y;slots=s;} }
    static class PathPoint {
        double x,y,t; String action,deliveryId; List<String> deliveryIds;
        PathPoint(double x,double y,double t,String a){this.x=x;this.y=y;this.t=t;this.action=a;}
    }

    // ── Main ────────────────────────────────────────────────────────────────
    public static void main(String[] args) throws Exception {
        BufferedReader br = new BufferedReader(new InputStreamReader(System.in));
        StringBuilder sb = new StringBuilder(); String l;
        while((l=br.readLine())!=null) sb.append(l);
        JsonObject inp = JsonParser.parseString(sb.toString()).getAsJsonObject();

        JsonArray ms = inp.getAsJsonArray("map_size");
        warehouseX = ms.get(0).getAsDouble()/2; warehouseY = ms.get(1).getAsDouble()/2;

        JsonArray dronesArr = inp.getAsJsonArray("drones");
        JsonArray delivArr  = inp.getAsJsonArray("deliveries");
        JsonArray nfzArr    = inp.has("no_fly_zones")      ? inp.getAsJsonArray("no_fly_zones")      : new JsonArray();
        JsonArray stArr     = inp.has("charging_stations") ? inp.getAsJsonArray("charging_stations") : new JsonArray();

        for(JsonElement e:nfzArr){
            JsonObject o=e.getAsJsonObject(); NFZ n=new NFZ();
            n.shape=o.get("shape").getAsString();
            n.tStart=o.get("T_start").getAsDouble(); n.tEnd=o.get("T_end").getAsDouble();
            if(n.shape.equals("circle")){
                JsonArray c=o.getAsJsonArray("center"); n.cx=c.get(0).getAsDouble(); n.cy=c.get(1).getAsDouble();
                n.radius=o.get("radius").getAsDouble();
            } else {
                JsonArray c=o.getAsJsonArray("corners");
                n.xmin=c.get(0).getAsJsonArray().get(0).getAsDouble();
                n.ymin=c.get(0).getAsJsonArray().get(1).getAsDouble();
                n.xmax=c.get(1).getAsJsonArray().get(0).getAsDouble();
                n.ymax=c.get(1).getAsJsonArray().get(1).getAsDouble();
            }
            nfzList.add(n);
        }
        for(JsonElement e:stArr){
            JsonObject o=e.getAsJsonObject();
            int slots=o.has("slots")?o.get("slots").getAsInt():1;
            stations.add(new Station(o.get("x").getAsDouble(),o.get("y").getAsDouble(),slots));
        }

        List<Delivery> allDeliveries=new ArrayList<>();
        for(JsonElement e:delivArr){
            JsonObject o=e.getAsJsonObject();
            allDeliveries.add(new Delivery(o.get("id").getAsString(),o.get("x").getAsDouble(),
                o.get("y").getAsDouble(),o.get("weight").getAsDouble(),o.get("deadline").getAsDouble()));
        }
        // Sort by deadline ascending
        allDeliveries.sort(Comparator.comparingDouble(d->d.deadline));

        int nDrones=dronesArr.size();
        double[] droneAvail  = new double[nDrones];
        double[] droneBat    = new double[nDrones];
        String[] droneId     = new String[nDrones];
        double[] dronePayMax = new double[nDrones];
        List<List<PathPoint>> manifests = new ArrayList<>();
        for(int i=0;i<nDrones;i++){
            JsonObject o=dronesArr.get(i).getAsJsonObject();
            droneId[i]=o.get("id").getAsString();
            dronePayMax[i]=o.get("max_payload").getAsDouble();
            droneBat[i]=BATTERY_MAX;
            manifests.add(new ArrayList<>());
        }

        Set<String> done=new HashSet<>();
        // Multiple rounds until all assigned or no progress
        int prevDone=-1;
        while(done.size()<allDeliveries.size() && done.size()!=prevDone){
            prevDone=done.size();
            for(int di=0;di<nDrones;di++){
                if(done.size()>=allDeliveries.size()) break;
                double avail=droneAvail[di];
                double bat=droneBat[di];
                double maxPay=dronePayMax[di];

                // Build best batch for this drone
                List<Delivery> batch=selectBatch(allDeliveries,done,maxPay,avail,bat);
                if(batch.isEmpty()) continue;

                List<PathPoint> trip=planTrip(batch,avail,bat);
                if(trip==null||trip.isEmpty()) {
                    // try individual deliveries
                    for(Delivery d:batch){
                        if(done.contains(d.id)) continue;
                        List<Delivery> single=Collections.singletonList(d);
                        List<PathPoint> t2=planTrip(single,avail,bat);
                        if(t2!=null&&!t2.isEmpty()){
                            trip=t2; batch=single; break;
                        }
                    }
                    if(trip==null||trip.isEmpty()) continue;
                }

                for(Delivery d:batch) done.add(d.id);
                PathPoint last=trip.get(trip.size()-1);
                droneAvail[di]=last.t;
                droneBat[di]=BATTERY_MAX; // recharged at warehouse
                manifests.get(di).addAll(trip);
            }
        }

        // Build JSON
        JsonArray fm=new JsonArray();
        for(int di=0;di<nDrones;di++){
            if(manifests.get(di).isEmpty()) continue;
            JsonObject entry=new JsonObject(); entry.addProperty("drone_id",droneId[di]);
            JsonArray pa=new JsonArray();
            for(PathPoint pp:manifests.get(di)){
                JsonObject pt=new JsonObject();
                pt.addProperty("x",r2(pp.x)); pt.addProperty("y",r2(pp.y)); pt.addProperty("t",r2(pp.t));
                pt.addProperty("action",pp.action);
                if(pp.deliveryIds!=null){JsonArray ids=new JsonArray();for(String s:pp.deliveryIds)ids.add(s);pt.add("delivery_ids",ids);}
                if(pp.deliveryId!=null) pt.addProperty("delivery_id",pp.deliveryId);
                pa.add(pt);
            }
            entry.add("path",pa); fm.add(entry);
        }
        JsonObject out=new JsonObject(); out.add("flight_manifest",fm);
        System.out.println(new Gson().toJson(out));
    }

    // ── Select a batch of deliveries for one trip ────────────────────────────
    static List<Delivery> selectBatch(List<Delivery> all, Set<String> done,
                                       double maxPay, double avail, double bat) {
        // Try nearest-neighbour grouping respecting weight limit
        List<Delivery> candidates=new ArrayList<>();
        for(Delivery d:all) if(!done.contains(d.id)) candidates.add(d);
        if(candidates.isEmpty()) return new ArrayList<>();

        // Sort by deadline then weight (heaviest first for same deadline)
        candidates.sort((a,b)->{
            int c=Double.compare(a.deadline,b.deadline);
            return c!=0?c:Double.compare(b.weight,a.weight);
        });

        List<Delivery> batch=new ArrayList<>();
        double totalW=0;
        for(Delivery d:candidates){
            if(totalW+d.weight<=maxPay+EPS){
                batch.add(d); totalW+=d.weight;
            }
            if(totalW>=maxPay-EPS) break;
        }
        return batch;
    }

    // ── Plan a complete trip ─────────────────────────────────────────────────
    static List<PathPoint> planTrip(List<Delivery> batch, double startTime, double startBat) {
        // Try multiple orderings: deadline, nearest-neighbour, reversed
        List<List<Delivery>> orderings=new ArrayList<>();
        orderings.add(orderByNearest(batch));
        orderings.add(orderByDeadline(batch));
        List<Delivery> rev=new ArrayList<>(orderByNearest(batch)); Collections.reverse(rev);
        orderings.add(rev);

        for(List<Delivery> ordered:orderings){
            List<PathPoint> result=attemptTrip(ordered,startTime,startBat);
            if(result!=null) return result;
        }
        return null;
    }

    static List<PathPoint> attemptTrip(List<Delivery> ordered, double startTime, double startBat) {
        List<PathPoint> path=new ArrayList<>();
        double payload=0; for(Delivery d:ordered) payload+=d.weight;
        double curX=warehouseX,curY=warehouseY,t=startTime,bat=startBat;

        // PICKUP
        PathPoint pk=new PathPoint(warehouseX,warehouseY,t,"PICKUP");
        pk.deliveryIds=new ArrayList<>(); for(Delivery d:ordered) pk.deliveryIds.add(d.id);
        path.add(pk);

        // Deliver each
        for(Delivery d:ordered){
            List<PathPoint> seg=navigate(curX,curY,d.x,d.y,t,payload,"DELIVER",d.id);
            if(seg==null) return null;
            PathPoint arr=seg.get(seg.size()-1);
            if(arr.t>d.deadline+EPS) return null; // missed deadline

            // consume battery
            double[] batT=consumeBat(curX,curY,seg,bat,payload);
            if(batT==null) return null; // battery died
            bat=batT[0]; payload-=d.weight;
            t=arr.t; curX=d.x; curY=d.y;
            path.addAll(seg);
        }

        // Check if we need to charge before returning home
        double distHome=dist(curX,curY,warehouseX,warehouseY);
        double energyHome=distHome*(1+0); // payload=0 on return

        if(bat<energyHome-EPS){
            // Need to charge — find best reachable station
            Station st=findBestStation(curX,curY,bat);
            if(st==null) return null; // stranded

            // Navigate to station
            List<PathPoint> toSt=navigate(curX,curY,st.x,st.y,t,0,"CHARGE",null);
            if(toSt==null) return null;
            double[] b2=consumeBat(curX,curY,toSt,bat,0); if(b2==null) return null;
            bat=b2[0];
            PathPoint arrSt=toSt.get(toSt.size()-1);
            // Add all but last as waypoints, last as CHARGE
            for(int i=0;i<toSt.size()-1;i++){PathPoint wp=toSt.get(i);wp.action="WAYPOINT";path.add(wp);}
            arrSt.action="CHARGE"; path.add(arrSt);
            t=arrSt.t; curX=st.x; curY=st.y;

            // Charge enough to get home
            distHome=dist(curX,curY,warehouseX,warehouseY);
            energyHome=distHome;
            double chargeNeeded=Math.max(0,energyHome-bat)+5; // small buffer
            double chargeSteps=Math.ceil(chargeNeeded/CHARGE_RATE);
            bat=Math.min(BATTERY_MAX,bat+chargeSteps*CHARGE_RATE);
            t+=chargeSteps;
            PathPoint cc=new PathPoint(curX,curY,t,"CHARGE_COMPLETE"); path.add(cc);
        }

        // Navigate home
        List<PathPoint> home=navigate(curX,curY,warehouseX,warehouseY,t,0,"RETURN",null);
        if(home==null) return null;
        double[] b3=consumeBat(curX,curY,home,bat,0); if(b3==null) return null;
        // Mark intermediate as WAYPOINT, last as RETURN
        for(int i=0;i<home.size()-1;i++){PathPoint wp=home.get(i);wp.action="WAYPOINT";path.add(wp);}
        PathPoint ret=home.get(home.size()-1); ret.action="RETURN"; path.add(ret);
        return path;
    }

    // ── Battery consumption for a path segment ───────────────────────────────
    // Returns [remaining_bat] or null if battery dies
    // fromX,fromY = starting position (before first point in seg)
    static double[] consumeBat(double fromX, double fromY, List<PathPoint> seg, double bat, double payload) {
        double px=fromX,py=fromY;
        for(PathPoint pp:seg){
            // skip WAIT points (no movement)
            if(pp.action!=null&&pp.action.equals("WAIT")){px=pp.x;py=pp.y;continue;}
            double d=dist(px,py,pp.x,pp.y);
            bat-=d*(1+payload);
            if(bat<-EPS) return null;
            px=pp.x;py=pp.y;
        }
        return new double[]{bat};
    }

    // ── Find nearest reachable charging station ──────────────────────────────
    static Station findBestStation(double x, double y, double bat) {
        Station best=null; double bestDist=Double.MAX_VALUE;
        for(Station st:stations){
            double d=dist(x,y,st.x,st.y);
            if(d*(1+0)<=bat+EPS && d<bestDist){bestDist=d;best=st;}
        }
        return best;
    }

    // ── Navigate from A to B with NFZ avoidance ──────────────────────────────
    static List<PathPoint> navigate(double ax,double ay,double bx,double by,
                                     double t,double payload,String endAct,String delId){
        List<double[]> waypoints=routePoints(ax,ay,bx,by,t,0);
        List<PathPoint> result=new ArrayList<>();
        double cx=ax,cy=ay,ct=t;
        for(int wi=0;wi<waypoints.size();wi++){
            double nx=waypoints.get(wi)[0],ny=waypoints.get(wi)[1];
            // Wait if segment is blocked
            double waitUntil=segWaitTime(cx,cy,nx,ny,ct);
            if(waitUntil>ct+EPS){
                result.add(new PathPoint(cx,cy,ct,"WAIT"));
                ct=waitUntil;
            }
            double segDist=dist(cx,cy,nx,ny);
            double arrT=ct+segDist;
            boolean isLast=(wi==waypoints.size()-1);
            PathPoint pp=new PathPoint(nx,ny,arrT,isLast?endAct:"WAYPOINT");
            if(isLast&&delId!=null) pp.deliveryId=delId;
            result.add(pp);
            cx=nx;cy=ny;ct=arrT;
        }
        return result;
    }

    // ── Build list of waypoints from A to B routing around NFZs ─────────────
    static List<double[]> routePoints(double ax,double ay,double bx,double by,double t,int depth){
        if(depth>6){List<double[]> r=new ArrayList<>();r.add(new double[]{bx,by});return r;}
        // Check each NFZ at traversal time
        for(NFZ nfz:nfzList){
            if(!segIntersectsNFZ(ax,ay,bx,by,nfz)) continue;
            // Does timing overlap?
            double[] te=entryExit(ax,ay,bx,by,nfz); if(te==null) continue;
            double tEntry=t+te[0], tExit=t+te[1];
            if(tEntry>nfz.tEnd+EPS||tExit<nfz.tStart-EPS) continue; // NFZ not active during traversal
            // Blocked - route around
            List<double[]> detour=detourAround(ax,ay,bx,by,nfz);
            if(detour==null) continue;
            // Build sub-routes recursively
            List<double[]> full=new ArrayList<>();
            double prevX=ax,prevY=ay,prevT=t;
            for(double[] wp:detour){
                List<double[]> sub=routePoints(prevX,prevY,wp[0],wp[1],prevT,depth+1);
                full.addAll(sub);
                if(!sub.isEmpty()){
                    double[] last=sub.get(sub.size()-1);
                    prevT+=dist(prevX,prevY,last[0],last[1]);
                    prevX=last[0];prevY=last[1];
                }
            }
            return full;
        }
        List<double[]> r=new ArrayList<>(); r.add(new double[]{bx,by}); return r;
    }

    // ── Find how long to wait at A before path A→B is clear ─────────────────
    static double segWaitTime(double ax,double ay,double bx,double by,double t){
        double wait=t;
        for(NFZ nfz:nfzList){
            if(!segIntersectsNFZ(ax,ay,bx,by,nfz)) continue;
            double[] te=entryExit(ax,ay,bx,by,nfz); if(te==null) continue;
            // We depart at 'wait'; check if NFZ active during traversal
            double tEntry=wait+te[0], tExit=wait+te[1];
            if(tEntry<=nfz.tEnd+EPS && tExit>=nfz.tStart-EPS){
                // Must wait until NFZ expires
                wait=Math.max(wait,nfz.tEnd);
            }
        }
        return wait;
    }

    // ── Detour waypoints around an NFZ ──────────────────────────────────────
    static List<double[]> detourAround(double ax,double ay,double bx,double by,NFZ nfz){
        List<double[]> candidates=new ArrayList<>();
        if(nfz.shape.equals("circle")){
            // Try multiple tangent angles
            for(int angle=0;angle<360;angle+=30){
                double rad=Math.toRadians(angle);
                double r=nfz.radius+4;
                candidates.add(new double[]{nfz.cx+Math.cos(rad)*r, nfz.cy+Math.sin(rad)*r});
            }
            // Also add the standard tangent points
            candidates.add(tangentWp(ax,ay,bx,by,nfz.cx,nfz.cy,nfz.radius+4,true));
            candidates.add(tangentWp(ax,ay,bx,by,nfz.cx,nfz.cy,nfz.radius+4,false));
        } else {
            double buf=4;
            candidates.add(new double[]{nfz.xmin-buf,nfz.ymin-buf});
            candidates.add(new double[]{nfz.xmax+buf,nfz.ymin-buf});
            candidates.add(new double[]{nfz.xmin-buf,nfz.ymax+buf});
            candidates.add(new double[]{nfz.xmax+buf,nfz.ymax+buf});
            candidates.add(new double[]{(nfz.xmin+nfz.xmax)/2,nfz.ymin-buf});
            candidates.add(new double[]{(nfz.xmin+nfz.xmax)/2,nfz.ymax+buf});
            candidates.add(new double[]{nfz.xmin-buf,(nfz.ymin+nfz.ymax)/2});
            candidates.add(new double[]{nfz.xmax+buf,(nfz.ymin+nfz.ymax)/2});
        }
        // Pick shortest VALID detour (both legs clear of THIS nfz)
        double bestLen=Double.MAX_VALUE; double[] bestWp=null;
        for(double[] wp:candidates){
            if(segIntersectsNFZ(ax,ay,wp[0],wp[1],nfz)) continue;
            if(segIntersectsNFZ(wp[0],wp[1],bx,by,nfz)) continue;
            double len=dist(ax,ay,wp[0],wp[1])+dist(wp[0],wp[1],bx,by);
            if(len<bestLen){bestLen=len;bestWp=wp;}
        }
        if(bestWp==null) return null;
        List<double[]> res=new ArrayList<>(); res.add(bestWp); res.add(new double[]{bx,by});
        return res;
    }

    static double[] tangentWp(double ax,double ay,double bx,double by,
                                double cx,double cy,double r,boolean left){
        double dx=bx-ax,dy=by-ay,len=Math.sqrt(dx*dx+dy*dy);
        if(len<EPS) return new double[]{cx+r,cy};
        dx/=len;dy/=len;
        double px=left?-dy:dy, py=left?dx:-dx;
        return new double[]{cx+px*r, cy+py*r};
    }

    // ── NFZ geometry ─────────────────────────────────────────────────────────
    static boolean segIntersectsNFZ(double ax,double ay,double bx,double by,NFZ nfz){
        if(nfz.shape.equals("circle")) return entryExit(ax,ay,bx,by,nfz)!=null;
        else return segRectIntersect(ax,ay,bx,by,nfz.xmin,nfz.ymin,nfz.xmax,nfz.ymax);
    }

    // Returns [distEntry, distExit] along segment, or null if no intersection
    static double[] entryExit(double ax,double ay,double bx,double by,NFZ nfz){
        if(nfz.shape.equals("circle")){
            double dx=bx-ax,dy=by-ay;
            double fx=ax-nfz.cx,fy=ay-nfz.cy;
            double a=dx*dx+dy*dy,b=2*(fx*dx+fy*dy),c=fx*fx+fy*fy-nfz.radius*nfz.radius;
            double disc=b*b-4*a*c;
            if(disc<0||a<EPS) return null;
            double sq=Math.sqrt(disc);
            double t1=(-b-sq)/(2*a),t2=(-b+sq)/(2*a);
            if(t2<-EPS||t1>1+EPS) return null;
            t1=Math.max(0,t1);t2=Math.min(1,t2);
            double len=Math.sqrt(a);
            return new double[]{t1*len,t2*len};
        } else {
            // Rect: approximate as full segment
            if(segRectIntersect(ax,ay,bx,by,nfz.xmin,nfz.ymin,nfz.xmax,nfz.ymax))
                return new double[]{0,dist(ax,ay,bx,by)};
            return null;
        }
    }

    static boolean segRectIntersect(double ax,double ay,double bx,double by,
                                     double xmin,double ymin,double xmax,double ymax){
        // Liang-Barsky algorithm for segment vs AABB intersection
        double dx=bx-ax, dy=by-ay;
        double tMin=0, tMax=1;
        // Left
        double p=-dx, q=ax-xmin;
        if(Math.abs(p)<EPS){ if(q<0) return false; } else {
            double t=q/p; if(p<0){ tMin=Math.max(tMin,t); } else { tMax=Math.min(tMax,t); }
        }
        // Right
        p=dx; q=xmax-ax;
        if(Math.abs(p)<EPS){ if(q<0) return false; } else {
            double t=q/p; if(p<0){ tMin=Math.max(tMin,t); } else { tMax=Math.min(tMax,t); }
        }
        // Bottom
        p=-dy; q=ay-ymin;
        if(Math.abs(p)<EPS){ if(q<0) return false; } else {
            double t=q/p; if(p<0){ tMin=Math.max(tMin,t); } else { tMax=Math.min(tMax,t); }
        }
        // Top
        p=dy; q=ymax-ay;
        if(Math.abs(p)<EPS){ if(q<0) return false; } else {
            double t=q/p; if(p<0){ tMin=Math.max(tMin,t); } else { tMax=Math.min(tMax,t); }
        }
        return tMin<=tMax+EPS;
    }
    static boolean ptInRect(double x,double y,double xmin,double ymin,double xmax,double ymax){
        return x>xmin-EPS&&x<xmax+EPS&&y>ymin-EPS&&y<ymax+EPS;
    }
    static boolean segsCross(double ax,double ay,double bx,double by,
                               double cx,double cy,double dx,double dy){
        double d1=cross(cx,cy,dx,dy,ax,ay),d2=cross(cx,cy,dx,dy,bx,by);
        double d3=cross(ax,ay,bx,by,cx,cy),d4=cross(ax,ay,bx,by,dx,dy);
        return((d1>0&&d2<0)||(d1<0&&d2>0))&&((d3>0&&d4<0)||(d3<0&&d4>0));
    }
    static double cross(double ax,double ay,double bx,double by,double px,double py){
        return(bx-ax)*(py-ay)-(by-ay)*(px-ax);
    }

    // ── Order utilities ──────────────────────────────────────────────────────
    static List<Delivery> orderByNearest(List<Delivery> batch){
        List<Delivery> rem=new ArrayList<>(batch),ord=new ArrayList<>();
        double cx=warehouseX,cy=warehouseY;
        while(!rem.isEmpty()){
            Delivery best=null;double bestS=Double.MAX_VALUE;
            for(Delivery d:rem){double s=dist(cx,cy,d.x,d.y)/(d.deadline+1);if(s<bestS){bestS=s;best=d;}}
            ord.add(best);cx=best.x;cy=best.y;rem.remove(best);
        }
        return ord;
    }
    static List<Delivery> orderByDeadline(List<Delivery> batch){
        List<Delivery> ord=new ArrayList<>(batch);
        ord.sort(Comparator.comparingDouble(d->d.deadline));
        return ord;
    }

    // ── Utilities ─────────────────────────────────────────────────────────────
    static double dist(double ax,double ay,double bx,double by){return Math.sqrt((bx-ax)*(bx-ax)+(by-ay)*(by-ay));}
    static double r2(double v){return Math.round(v*100.0)/100.0;}
}