package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Wrist {
    public final DcMotorEx wristMotor;
    private static final double TICKS_PER_DEGREE = 288 / 360.0;

    public Wrist(HardwareMap hardwareMap) {
        this.wristMotor = hardwareMap.get(DcMotorEx.class, "wrist_motor");
        wristMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wristMotor.setTargetPosition(0);
        wristMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public class WristTo implements Action {
        private boolean initialized = false;
        private final double angle;

        public WristTo(double angle) {
            this.angle = angle;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                setTargetAngle(angle);
                initialized = true;
            }
            double pos = getCurrentAngle();
            telemetryPacket.put("wristAngle", pos);
            return stationary();
        }
    }

    public boolean setTargetAngle(double newTargetAngle) {
        //validation
        if (newTargetAngle < -90 || newTargetAngle > 90) {
            return false;
        } else {
            wristMotor.setTargetPosition(angleToTicks(newTargetAngle));
            return true;
        }
    }

    public double getCurrentAngle() {
        return ticksToAngle(wristMotor.getCurrentPosition());
    }

    private int angleToTicks(double angle) {
        return (int) (angle * TICKS_PER_DEGREE);
    }

    private double ticksToAngle(int ticks) {
        return ticks / TICKS_PER_DEGREE;

    }

    public boolean stationary() {
        return !wristMotor.isBusy();
    }

    public Action wristToHome()
    {
        return new Wrist.WristTo(0);
    }
    public Action wristToAngle(double angle)
    {
        return new Wrist.WristTo(angle);
    }

}

