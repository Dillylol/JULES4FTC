package org.firstinspires.ftc.teamcode.jules.bridge;

import android.content.Context;
import android.content.SharedPreferences;

import java.util.UUID;

public final class JulesTokenStore {
    private static final String PREFS = "jules_prefs";
    private static final String KEY   = "http_token";

    private JulesTokenStore() {}

    public static String getOrCreate(Context ctx) {
        SharedPreferences p = ctx.getSharedPreferences(PREFS, Context.MODE_PRIVATE);
        String tok = p.getString(KEY, null);
        if (tok == null || tok.isEmpty()) {
            tok = generateToken();
            p.edit().putString(KEY, tok).apply();
        }
        return tok;
    }

    public static void reset(Context ctx) {
        ctx.getSharedPreferences(PREFS, Context.MODE_PRIVATE).edit().remove(KEY).apply();
    }

    private static String generateToken() {
        // Short, URL-safe base36 token (stable enough, ~20 chars)
        return Long.toString(UUID.randomUUID().getMostSignificantBits() & Long.MAX_VALUE, 36)
                +  Long.toString(UUID.randomUUID().getLeastSignificantBits() & Long.MAX_VALUE, 36).substring(0, 4);
    }
}
