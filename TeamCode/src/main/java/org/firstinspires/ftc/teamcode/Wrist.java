package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Wrist {
    private DcMotorEx wristMotor;
    private static final double TICKS_PER_DEGREE = 288/360.0;

    public Wrist(HardwareMap hardwareMap)
    {
        this.wristMotor = hardwareMap.get(DcMotorEx.class, "wrist_motor");
        wristMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wristMotor.setTargetPosition(0);
        wristMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            }

    public boolean setTargetAngle(double newTargetAngle)
    {
        //validation
        if(newTargetAngle < -90 || newTargetAngle > 90) {
            return false;
        }
        else
        {
            wristMotor.setTargetPosition(angleToTicks(newTargetAngle));
            return true;
        }
    }
    public double getCurrentAngle()
    {
        return ticksToAngle(wristMotor.getCurrentPosition());
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
        return !wristMotor.isBusy();
    }

}