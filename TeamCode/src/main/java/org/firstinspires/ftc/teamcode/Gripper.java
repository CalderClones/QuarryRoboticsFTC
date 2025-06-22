package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Gripper {
    //servo positions need to be confirmed empirically
    private static final double GRIPPER_OPEN = 0.0;
    private static final double GRIPPER_CLOSED = 1.0;
    private static final double SERVO_DELAY = 1000.0;
    private final Servo gripperServo;
    private boolean gripperBusy;
    private String position;


    public Gripper(HardwareMap hardwareMap) {
        this.gripperServo = hardwareMap.get(Servo.class, "gripper_servo");
        gripperServo.setPosition(GRIPPER_CLOSED);
        gripperBusy = false;


    }

    public String getPosition() {
        return position;
    }

    public void setPosition(String position) {
        this.position = position;
    }

    public class GripperTo implements Action {
        private final ElapsedTime gripperTimer;
        private boolean initialized = false;


        public GripperTo(String newPosition) {
            gripperTimer = new ElapsedTime();
            setPosition(newPosition);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {

                if (getPosition() == "Open") {
                    open();
                } else if (getPosition() == "Closed") {
                    close();
                }
                initialized = true;
                gripperBusy = true;
                gripperTimer.reset();
            }
            telemetryPacket.put("gripperTimer", gripperTimer.milliseconds());
            if (gripperTimer.milliseconds() > SERVO_DELAY){
                gripperBusy = false;
            }
            //return true if the servo still has time to close otherwise return false to indicate movement has finished
            return (gripperBusy);
        }
    }

    public void close() {
        gripperServo.setPosition(GRIPPER_CLOSED);
        position = "Closed";
    }

    public void open() {
        gripperServo.setPosition(GRIPPER_OPEN);
        position = "Open";
    }

    public boolean stationary()
    {
        return !gripperBusy;
    }

    public Action gripperToOpen()
    {
        return new Gripper.GripperTo("Open");
    }

    public Action gripperToClosed()
    {
        return new Gripper.GripperTo("Closed");
    }
}

