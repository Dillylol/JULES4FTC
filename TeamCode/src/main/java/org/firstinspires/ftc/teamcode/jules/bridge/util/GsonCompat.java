package org.firstinspires.ftc.teamcode.jules.bridge.util;

import com.google.gson.Gson;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

public final class GsonCompat {
    private static final Gson GSON = new Gson();

    private GsonCompat() {}

    /** Parse JSON in a way that works on both old and new Gson. */
    public static JsonElement parse(String json) {
        try {
            // Newer Gson (2.8.6+)
            return (JsonElement) JsonParser.class
                    .getMethod("parseString", String.class)
                    .invoke(null, json);
        } catch (Throwable ignore) {
            // Older Gson
            return new JsonParser().parse(json);
        }
    }

    /** Deep copy that doesn't rely on JsonObject#deepCopy visibility. */
    public static JsonObject deepCopy(JsonObject src) {
        if (src == null) return null;
        // round-trip via Gson
        return GSON.fromJson(src, JsonObject.class);
    }
}
