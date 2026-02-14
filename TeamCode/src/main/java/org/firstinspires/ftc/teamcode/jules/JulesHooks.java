package org.firstinspires.ftc.teamcode.jules;

import android.content.Context;
import org.firstinspires.ftc.ftccommon.external.OnCreate;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesBridgeManager;

/**
 * This class uses the @OnCreate annotation to automatically initialize
 * the JulesService when the Robot Controller app starts.
 */
public class JulesHooks {

    /**
     * This method is called once when the Robot Controller application is started.
     * @param context The application context.
     */
    @OnCreate
    public static void onCreate(Context context) {
        JulesBridgeManager.getInstance().prepare(context);
    }
}