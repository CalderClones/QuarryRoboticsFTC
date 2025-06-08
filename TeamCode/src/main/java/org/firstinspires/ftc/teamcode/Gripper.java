package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Gripper {
    //servo positions need to be confirmed empirically
    private static final double GRIPPER_OPEN = 0.0;
    private static final double GRIPPER_CLOSED = 1.0;
    private static final double SERVO_DELAY = 250.0;
    private Servo gripperServo;
    private boolean gripperBusy;

    public Gripper(HardwareMap hardwareMap) {
        this.gripperServo = hardwareMap.get(Servo.class, "gripper_servo");
        gripperServo.setPosition(GRIPPER_CLOSED);
        gripperBusy = false;


    }

    public class GripperTo implements Action {
        private ElapsedTime gripperTimer;
        private boolean initialized = false;
        private String position;

        public GripperTo(String position) {
            gripperTimer = new ElapsedTime();
            this.position = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {

                if (position == "Open") {
                    open();
                } else if (position == "Closed") {
                    close();
                }
                initialized = true;
                gripperBusy = true;
                gripperTimer.reset();
            }
            double timeToClose = SERVO_DELAY - gripperTimer.milliseconds();
            telemetryPacket.put("timeToClose", timeToClose);
            if (timeToClose <= 0){
                gripperBusy = false;
            }
            //return false if the servo still has time to close otherwise return true to indicate movement has finished
            return (!gripperBusy);
        }
    }

    public void close() {
        gripperServo.setPosition(GRIPPER_CLOSED);
    }

    public void open() {
        gripperServo.setPosition(GRIPPER_OPEN);
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

