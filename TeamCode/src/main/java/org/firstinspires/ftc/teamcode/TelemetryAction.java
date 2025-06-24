package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.nio.channels.ClosedByInterruptException;

public class TelemetryAction implements Action {
    private boolean initialized = false;
    private String line;
    private Telemetry telemetry;

    public TelemetryAction(Telemetry telemetry, String line) {
        this.line = line;
        this.telemetry = telemetry;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (!initialized) {
            telemetry.addLine(line);
            telemetry.update();
            initialized = true;
        }
        telemetryPacket.put("Telemetry Posted", line);
        return false;
    }
}

