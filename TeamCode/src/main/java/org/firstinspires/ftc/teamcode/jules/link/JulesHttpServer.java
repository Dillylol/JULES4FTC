package org.firstinspires.ftc.teamcode.jules.link;

import com.google.gson.JsonObject;

import org.firstinspires.ftc.teamcode.jules.telemetry.JulesDataOrganizer;

import java.io.IOException;

import fi.iki.elonen.NanoHTTPD;

/**
 * Lightweight HTTP endpoint that exposes the last snapshot.
 */
public class JulesHttpServer extends NanoHTTPD {

    private static final int PORT = 8770;

    private final JulesDataOrganizer organizer;

    public JulesHttpServer(JulesDataOrganizer organizer) {
        super(PORT);
        this.organizer = organizer;
    }

    public void startServer() {
        try {
            start(NanoHTTPD.SOCKET_READ_TIMEOUT, false);
        } catch (IOException ignored) {
            // optional, ignore failures
        }
    }

    public void stopServer() {
        stop();
    }

    @Override
    public Response serve(IHTTPSession session) {
        if (Method.GET.equals(session.getMethod()) && "/vitals".equals(session.getUri())) {
            JsonObject snapshot = organizer.getLatestSnapshotMessage();
            if (snapshot == null) {
                snapshot = organizer.buildSnapshot();
            }
            return NanoHTTPD.newFixedLengthResponse(Response.Status.OK, "application/json", snapshot.toString());
        }
        return NanoHTTPD.newFixedLengthResponse(Response.Status.NOT_FOUND, "text/plain", "Not found");
    }
}

