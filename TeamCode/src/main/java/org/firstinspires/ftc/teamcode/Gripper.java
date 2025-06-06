package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gripper {
    //servo positions need to be confirmed empirically
    private static final double GRIPPER_OPEN = 0.0;
    private static final double GRIPPER_CLOSED = 1.0;
    private Servo gripperServo;
    public Gripper(HardwareMap hardwareMap)
    {
        this.gripperServo = hardwareMap.get(Servo.class, "gripper_servo");
        gripperServo.setPosition(GRIPPER_CLOSED);
    }

    public void close()
    {
        gripperServo.setPosition(GRIPPER_CLOSED);
    }

    public void open()
    {
        gripperServo.setPosition(GRIPPER_OPEN);
    }
}
