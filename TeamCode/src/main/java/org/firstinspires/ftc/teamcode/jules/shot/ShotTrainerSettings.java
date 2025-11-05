package org.firstinspires.ftc.teamcode.jules.shot;

import android.content.Context;
import android.content.SharedPreferences;

/**
 * Centralized configuration access for the shot trainer bridge.
 */
public final class ShotTrainerSettings {

    private static final String PREFS_NAME = "shot_trainer";
    private static final String KEY_CLIENT_IP = "client_ip";
    private static final String KEY_CLIENT_PORT = "client_port";
    private static final String KEY_BOT_ID = "bot_id";

    private static final String DEFAULT_CLIENT_IP = "192.168.49.1";
    private static final int DEFAULT_CLIENT_PORT = 58080;
    private static final String DEFAULT_BOT_ID = "ftc-bot";

    private ShotTrainerSettings() {
    }

    public static String getClientIp(Context context) {
        if (context == null) {
            return DEFAULT_CLIENT_IP;
        }
        SharedPreferences prefs = context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);
        return prefs.getString(KEY_CLIENT_IP, DEFAULT_CLIENT_IP);
    }

    public static int getClientPort(Context context) {
        if (context == null) {
            return DEFAULT_CLIENT_PORT;
        }
        SharedPreferences prefs = context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);
        return prefs.getInt(KEY_CLIENT_PORT, DEFAULT_CLIENT_PORT);
    }

    public static String getBotId(Context context) {
        if (context == null) {
            return DEFAULT_BOT_ID;
        }
        SharedPreferences prefs = context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);
        return prefs.getString(KEY_BOT_ID, DEFAULT_BOT_ID);
    }
}
