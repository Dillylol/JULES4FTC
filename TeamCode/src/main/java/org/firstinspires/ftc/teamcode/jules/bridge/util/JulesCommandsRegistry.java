package org.firstinspires.ftc.teamcode.jules.bridge.util;


import android.content.Context;
import androidx.annotation.NonNull;


import org.firstinspires.ftc.teamcode.R; // <-- make sure this import resolves


import com.google.gson.JsonArray;
import com.google.gson.JsonObject;


import java.io.InputStream;
import java.io.ByteArrayOutputStream;
import java.nio.charset.StandardCharsets;


/** Loads/supplies the JULES commands manifest from res/raw/jules_commands.json. */
public final class JulesCommandsRegistry {
    private final JsonObject manifest;
    private JulesCommandsRegistry(JsonObject manifest) { this.manifest = manifest; }


    /** Load from R.raw.jules_commands with a safe fallback. */
    public static @NonNull JulesCommandsRegistry load(Context ctx) {
        JsonObject m;
        try (InputStream is = ctx.getResources().openRawResource(R.raw.jules_commands)) {
            String json = readAllUtf8(is);
// Use your existing compat helper (works on FTC's bundled Gson)
            m = GsonCompat.parse(json).getAsJsonObject();
        } catch (Throwable ignore) {
            m = new JsonObject();
            m.addProperty("schema", "jules-commands/v1");
            m.add("commands", new JsonArray());
        }
        return new JulesCommandsRegistry(m);
    }


    /** Copy without JsonObject#deepCopy() (not public on FTC Gson). */
    public @NonNull JsonObject asJson() {
        return GsonCompat.parse(manifest.toString()).getAsJsonObject();
    }


    public @NonNull JsonArray commands() {
        return manifest.has("commands") && manifest.get("commands").isJsonArray()
                ? manifest.getAsJsonArray("commands")
                : new JsonArray();
    }


    private static String readAllUtf8(InputStream is) throws Exception {
        ByteArrayOutputStream baos = new ByteArrayOutputStream();
        byte[] buf = new byte[4096];
        int n;
        while ((n = is.read(buf)) != -1) baos.write(buf, 0, n);
        return baos.toString(StandardCharsets.UTF_8.name());
    }
}