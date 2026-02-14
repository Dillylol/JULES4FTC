
package org.firstinspires.ftc.teamcode.jules.bridge.util;

import com.google.gson.JsonObject;
import java.util.*;

public class JulesCommandRouter {
    public interface Handler { boolean handle(String name, JsonObject args); }

    private final Map<String, Handler> map = new HashMap<>();

    public void register(String name, Handler h) {
        map.put(name.toLowerCase(Locale.US), h);
    }

    public boolean dispatch(JsonObject frame) {
        if (frame == null) return false;
        String name = frame.has("name") ? frame.get("name").getAsString() : null;
        JsonObject args = frame.has("args") && frame.get("args").isJsonObject() ? frame.getAsJsonObject("args") : new JsonObject();
        if (name == null) return false;
        Handler h = map.get(name.toLowerCase(Locale.US));
        return h != null && h.handle(name, args);
    }
}
