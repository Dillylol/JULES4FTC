package org.firstinspires.ftc.teamcode.jules.telemetry;

import androidx.annotation.Nullable;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.Locale;

/**
 * Lightweight proxy that mirrors FTC telemetry calls into the {@link JulesDataOrganizer}.
 */
public final class JulesTelemetry {

    private final Telemetry delegate;
    private final JulesDataOrganizer organizer;
    private final Telemetry proxy;

    public JulesTelemetry(Telemetry delegate, JulesDataOrganizer organizer) {
        this.delegate = delegate;
        this.organizer = organizer;
        this.proxy = buildProxy();
    }

    public Telemetry getFtcTelemetry() {
        return proxy;
    }

    private Telemetry buildProxy() {
        ClassLoader loader = delegate.getClass().getClassLoader();
        Class<?>[] interfaces = new Class<?>[]{Telemetry.class};
        InvocationHandler handler = new TelemetryInvocationHandler();
        return (Telemetry) Proxy.newProxyInstance(loader, interfaces, handler);
    }

    private final class TelemetryInvocationHandler implements InvocationHandler {

        @Override
        public Object invoke(Object proxy, Method method, Object[] args) throws Throwable {
            String name = method.getName();

            if ("equals".equals(name) && args != null && args.length == 1) {
                return proxy == args[0];
            }
            if ("hashCode".equals(name)) {
                return System.identityHashCode(proxy);
            }
            if ("toString".equals(name)) {
                return "JulesTelemetryProxy(" + delegate + ")";
            }

            if ("addData".equals(name) && args != null && args.length >= 2) {
                String key = String.valueOf(args[0]);
                String value = extractValue(args);
                organizer.recordTelemetry(key, value);
            } else if (("clear".equals(name) || "clearAll".equals(name)) && args == null) {
                organizer.clearTelemetry();
            }

            Object result = method.invoke(delegate, args);
            if ("log".equals(name) && result != null) {
                // pass-through, nothing to mirror for log entries
            }
            return result;
        }

        private String extractValue(@Nullable Object[] args) {
            if (args == null || args.length < 2) {
                return "";
            }

            if (args.length == 2) {
                Object value = args[1];
                return value == null ? "" : String.valueOf(value);
            }

            Object format = args[1];
            Object[] formatArgs = new Object[args.length - 2];
            System.arraycopy(args, 2, formatArgs, 0, formatArgs.length);
            if (format instanceof String) {
                try {
                    return String.format(Locale.US, (String) format, formatArgs);
                } catch (Throwable ignored) {
                    return String.valueOf(format);
                }
            }
            return String.valueOf(format);
        }
    }
}

