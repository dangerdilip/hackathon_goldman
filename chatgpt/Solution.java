import java.io.*;
import java.util.*;
import com.google.gson.*;

public class Solution {

    static final double BATTERY = 500.0;
    static final double SPEED = 1.0;
    static final double EPS = 1e-7;

    static class Drone {
        String id;
        double maxPayload;

        Drone(String id, double maxPayload) {
            this.id = id;
            this.maxPayload = maxPayload;
        }
    }

    static class Delivery {
        String id;
        double x, y, weight, deadline;

        Delivery(JsonObject o) {
            id = o.get("id").getAsString();
            x = o.get("x").getAsDouble();
            y = o.get("y").getAsDouble();
            weight = o.get("weight").getAsDouble();
            deadline = o.get("deadline").getAsDouble();
        }
    }

    static class NFZ {
        String shape;
        double t1, t2;

        // circle
        double cx, cy, r;

        // rect
        double x1, y1, x2, y2;

        NFZ(JsonObject o) {

            shape = o.get("shape").getAsString();

            t1 = o.get("T_start").getAsDouble();
            t2 = o.get("T_end").getAsDouble();

            if (shape.equals("circle")) {

                JsonArray c = o.getAsJsonArray("center");

                cx = c.get(0).getAsDouble();
                cy = c.get(1).getAsDouble();

                r = o.get("radius").getAsDouble();

            } else {

                JsonArray corners = o.getAsJsonArray("corners");

                JsonArray a = corners.get(0).getAsJsonArray();
                JsonArray b = corners.get(1).getAsJsonArray();

                x1 = Math.min(
                        a.get(0).getAsDouble(),
                        b.get(0).getAsDouble());

                x2 = Math.max(
                        a.get(0).getAsDouble(),
                        b.get(0).getAsDouble());

                y1 = Math.min(
                        a.get(1).getAsDouble(),
                        b.get(1).getAsDouble());

                y2 = Math.max(
                        a.get(1).getAsDouble(),
                        b.get(1).getAsDouble());
            }
        }
    }

    static double dist(
            double x1,
            double y1,
            double x2,
            double y2) {

        return Math.hypot(x1 - x2, y1 - y2);
    }

    static double round2(double v) {
        return Math.round(v * 100.0) / 100.0;
    }

    static boolean pointInRect(
            double x,
            double y,
            NFZ z) {

        return x >= z.x1 - EPS &&
               x <= z.x2 + EPS &&
               y >= z.y1 - EPS &&
               y <= z.y2 + EPS;
    }

    static boolean segmentHitsRect(
            double x1,
            double y1,
            double x2,
            double y2,
            NFZ z) {

        int STEPS = 200;

        for (int i = 0; i <= STEPS; i++) {

            double t = (double)i / STEPS;

            double x = x1 + (x2 - x1) * t;
            double y = y1 + (y2 - y1) * t;

            if (pointInRect(x, y, z))
                return true;
        }

        return false;
    }

    static boolean segmentHitsCircle(
            double x1,
            double y1,
            double x2,
            double y2,
            NFZ z) {

        double dx = x2 - x1;
        double dy = y2 - y1;

        double fx = x1 - z.cx;
        double fy = y1 - z.cy;

        double a = dx * dx + dy * dy;

        double b = 2 * (fx * dx + fy * dy);

        double c =
                fx * fx +
                fy * fy -
                z.r * z.r;

        double disc = b * b - 4 * a * c;

        return disc >= 0;
    }

    static boolean blocked(
            double x1,
            double y1,
            double x2,
            double y2,
            double startTime,
            List<NFZ> nfzs) {

        double d = dist(x1, y1, x2, y2);

        double endTime = startTime + d;

        for (NFZ z : nfzs) {

            if (endTime < z.t1 || startTime > z.t2)
                continue;

            boolean hit;

            if (z.shape.equals("circle")) {

                hit = segmentHitsCircle(
                        x1, y1,
                        x2, y2,
                        z);

            } else {

                hit = segmentHitsRect(
                        x1, y1,
                        x2, y2,
                        z);
            }

            if (hit)
                return true;
        }

        return false;
    }

    static JsonObject event(
            double x,
            double y,
            double t,
            String action) {

        JsonObject o = new JsonObject();

        o.addProperty("x", round2(x));
        o.addProperty("y", round2(y));
        o.addProperty("t", round2(t));
        o.addProperty("action", action);

        return o;
    }

    public static void main(String[] args) throws Exception {

        BufferedReader br =
                new BufferedReader(
                        new InputStreamReader(System.in));

        StringBuilder sb = new StringBuilder();

        String line;

        while ((line = br.readLine()) != null)
            sb.append(line);

        JsonObject input =
                JsonParser.parseString(
                        sb.toString()).getAsJsonObject();

        JsonArray map =
                input.getAsJsonArray("map_size");

        double wx =
                map.get(0).getAsDouble() / 2.0;

        double wy =
                map.get(1).getAsDouble() / 2.0;

        List<Drone> drones = new ArrayList<>();

        for (JsonElement e :
                input.getAsJsonArray("drones")) {

            JsonObject o = e.getAsJsonObject();

            drones.add(
                    new Drone(
                            o.get("id").getAsString(),
                            o.get("max_payload").getAsDouble()));
        }

        List<Delivery> deliveries =
                new ArrayList<>();

        for (JsonElement e :
                input.getAsJsonArray("deliveries")) {

            deliveries.add(
                    new Delivery(e.getAsJsonObject()));
        }

        deliveries.sort(
                Comparator.comparingDouble(d -> d.deadline));

        List<NFZ> nfzs =
                new ArrayList<>();

        if (input.has("no_fly_zones")) {

            for (JsonElement e :
                    input.getAsJsonArray("no_fly_zones")) {

                nfzs.add(
                        new NFZ(e.getAsJsonObject()));
            }
        }

        JsonArray manifest =
                new JsonArray();

        int ptr = 0;

        for (Drone drone : drones) {

            JsonObject droneObj =
                    new JsonObject();

            droneObj.addProperty(
                    "drone_id",
                    drone.id);

            JsonArray path =
                    new JsonArray();

            double time = 0;

            while (ptr < deliveries.size()) {

                Delivery d =
                        deliveries.get(ptr);

                if (d.weight > drone.maxPayload) {
                    ptr++;
                    continue;
                }

                double tripDist =
                        dist(wx, wy, d.x, d.y);

                double energy =
                        tripDist * (1 + d.weight)
                        + tripDist;

                if (energy > BATTERY)
                    break;

                JsonObject pickup =
                        event(
                                wx,
                                wy,
                                time,
                                "PICKUP");

                JsonArray ids =
                        new JsonArray();

                ids.add(d.id);

                pickup.add("delivery_ids", ids);

                path.add(pickup);

                while (blocked(
                        wx,
                        wy,
                        d.x,
                        d.y,
                        time,
                        nfzs)) {

                    time += 1;

                    path.add(
                            event(
                                    wx,
                                    wy,
                                    time,
                                    "WAIT"));
                }

                double arrive =
                        time + tripDist;

                if (arrive > d.deadline)
                    break;

                JsonObject deliver =
                        event(
                                d.x,
                                d.y,
                                arrive,
                                "DELIVER");

                deliver.addProperty(
                        "delivery_id",
                        d.id);

                path.add(deliver);

                time = arrive;

                while (blocked(
                        d.x,
                        d.y,
                        wx,
                        wy,
                        time,
                        nfzs)) {

                    time += 1;

                    path.add(
                            event(
                                    d.x,
                                    d.y,
                                    time,
                                    "WAIT"));
                }

                time += tripDist;

                path.add(
                        event(
                                wx,
                                wy,
                                time,
                                "RETURN"));

                ptr++;
            }

            droneObj.add("path", path);

            manifest.add(droneObj);
        }

        JsonObject out =
                new JsonObject();

        out.add("flight_manifest", manifest);

        System.out.println(
                new Gson().toJson(out));
    }
}