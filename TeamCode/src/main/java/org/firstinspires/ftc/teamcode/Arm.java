package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Arm {
    private DcMotorEx armMotor;
    private TouchSensor limitSwitch;
    private DistanceSensor distanceSensor;


    //raw motor has 28 ticks per rotation
    //gearbox has 5:1, 5:1, 4:1 and 3:1 cardridges
    private static final double TICKS_PER_DEGREE = 28.0 * (68.0/13.0) * (68.0 / 13.0) * (7.0 / 21.0) * (84.0 / 29.0) / 360.0;

    public Arm(HardwareMap hardwareMap)
    {
        this.armMotor = hardwareMap.get(DcMotorEx.class, "arm_motor");
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.limitSwitch = hardwareMap.get(TouchSensor.class, "arm_limit_switch");
        this.distanceSensor = hardwareMap.get(DistanceSensor.class, "arm_height_sensor");

    }

    public class ArmTo implements Action {
        private boolean initialized = false;
        private final String position;

        public ArmTo(String position) {
            this.position = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                if (position == "Horizontal") {
                    setTargetAngle(90);
                } else if (position == "Vertical") {
                    setTargetAngle(0);
                }
//                else if (position == "Basket") {
//                    setTargetAngle(45);
//                }
                initialized = true;
            }
            double pos = getCurrentAngle();
            telemetryPacket.put("armAngle", pos);
            return stationary();
        }
    }


    public void reset()
    {
        //TODO: this should be a routine that raises the arm to vertical and then resets the encoder when the limit switch is triggered

    }
    public boolean setTargetAngle(double newTargetAngle)
    {
        //validation - motor must be somewhere between vertical (0) and horizontal (90)
        if(newTargetAngle < 0 || newTargetAngle > 90) {
            return false;
        }
        else
        {
            armMotor.setTargetPosition(angleToTicks(newTargetAngle));
            return true;
        }
    }

    public double getCurrentAngle()
    {
        return ticksToAngle(armMotor.getCurrentPosition());
    }
    private int angleToTicks(double angle)
    {
        return (int) (angle * TICKS_PER_DEGREE);
    }

    private double ticksToAngle(int ticks)
    {
        return ticks / TICKS_PER_DEGREE;

    }

    public boolean stationary()
    {
        return !armMotor.isBusy();
    }

    public Action armToVertical()
    {
        return new Arm.ArmTo("Vertical");
    }
    public Action armToHorizontal()
    {
        return new Arm.ArmTo("Horizontal");
    }
    public Action armToBasket()
    {
        return new Arm.ArmTo("Basket");
    }


}