package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Arm {
    public DcMotorEx armMotor;
    private TouchSensor limitSwitch;
    private DistanceSensor distanceSensor;

    private double targetAngle = 0;
    private double SPEED = 30;


    //raw motor has 28 ticks per rotation
    //gearbox has 5:1, 5:1, 4:1 and 3:1 cardridges
    private static final double TICKS_PER_DEGREE = 28.0 * (68.0/13.0) * (68.0 / 13.0) * (76.0 / 21.0) * (84.0 / 29.0) / 360.0;

    public Arm(HardwareMap hardwareMap)
    {
        this.armMotor = hardwareMap.get(DcMotorEx.class, "arm_motor");
        //TODO: Does this motor need to be reversed?
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setPower(0.8);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.limitSwitch = hardwareMap.get(TouchSensor.class, "arm_limit_switch");
        this.distanceSensor = hardwareMap.get(DistanceSensor.class, "arm_height_sensor");

    }

    public void reset()
    {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setPower(0);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(limitSwitch.isPressed()) {
            armMotor.setPower(0.1);
        }

        while(! limitSwitch.isPressed()) {
        armMotor.setPower(-0.2);
        }

        armMotor.setPower(0);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.5);
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
               else if (position == "Basket") {
                  setTargetAngle(45);
                }
                initialized = true;
            }
            double pos = getCurrentAngle();
            telemetryPacket.put("armAngle", pos);
            //This needs to return true when action is still running and false when not.
            return !stationary();
        }
    }


    public boolean setTargetAngle(double newTargetAngle)
    {
        //validation - motor must be somewhere between vertical (0) and horizontal (90)
        if(newTargetAngle < 0 || newTargetAngle > 90) {
            return false;
        }
        else
        {
            this.targetAngle = newTargetAngle;
            armMotor.setTargetPosition(angleToTicks(targetAngle));
            return true;
        }
    }

    public double getTargetAngle(){
        return this.targetAngle;
    }

    public void moveArm(double multiplier)
    {
        setTargetAngle(getCurrentAngle() + SPEED * multiplier);

    }

    public void moveDown()
    {
        setTargetAngle(getTargetAngle()+SPEED);
    }

    public void moveUp()
    {
        setTargetAngle(getTargetAngle()-SPEED);
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
        return !(armMotor.isBusy());
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