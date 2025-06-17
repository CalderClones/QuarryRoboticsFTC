package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;

public class Lift {
    public DcMotorEx liftMotor;
    private TouchSensor limitSwitch;

    private double offset_from_floor;
    private double target_height;
    private double current_height;
    private double tolerance;
    public LinkedHashMap<String, Integer> presets = new LinkedHashMap<String, Integer>();
    public String liftTargetPreset = "Floor";

    private static final double TICKS_PER_MM = 28.0 * (76.0 / 21.0) * (76.0 / 21.0) * (84.0 / 29.0)  / 44.0 / 3.0 / 3.0;

    public Lift(HardwareMap hardwareMap) {
        //these presets need to be tuned empirically
        this.presets.put("Floor", 11);  //Arm measures 11mm above floor when horizontal at lift lower limit
        this.presets.put("LowChamberClipped", 60); //Calculated - will definitely need tuning
        this.presets.put("Grabbing", 98); //Arm measures 98mm above floor when horizontal and at the right height to grab a sample
        this.presets.put("LowChamber", 149); //Calculated - will definitely need tuning
        this.presets.put("Scanning", 240); // We measured 230mm as the best height for scanning
        this.presets.put("HighChamberClipped", 386); //Calculated - will definitely need tuning
        this.presets.put("HighChamber", 475); //Calculated - will definitely need tuning
        this.presets.put("LowBasket", 562); // Calculated. Assumes arm at 45 degrees
        this.presets.put("HighBasket", 1000); // Calculated - 1092mm to lip of basket with arm at 45, sample should be 1117mm above floor - 25mm clearance
        this.presets.put("Ceiling", 1050); // derived from cad model. Should give a little clearance to avoid smashing into top stop.

        offset_from_floor = presets.get("Floor");

        this.liftMotor = hardwareMap.get(DcMotorEx.class, "lift_motor");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        move_to_preset("Floor");
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(1.0);


        this.limitSwitch = hardwareMap.get(TouchSensor.class, "lift_limit_switch");

        this.target_height = 0.0;
        this.current_height = 0.0;
        this.tolerance = 3.0;
    }

    public class LiftTo implements Action {
        private boolean initialized = false;
        private String preset;
        public LiftTo(String preset) {
            this.preset = preset;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                move_to_preset(preset);
                initialized = true;
            }
            double pos = getCurrentHeight();
            telemetryPacket.put("liftHeight", pos);
            if (stationary())
                return true;
            else
            {
                return false;
            }
        }
    }



    public boolean move_to_preset(String preset) {
        if (presets.containsKey(preset))
        {
            set_target_height(presets.get(preset));
            return true;
        }
        else
        {
            return false;
        }

    }

    public double getCurrentHeight()
    {
        return ticks_to_height(liftMotor.getCurrentPosition());
    }
    public void set_target_height(double height) {
        target_height = height;
        liftMotor.setTargetPosition(height_to_ticks(Range.clip(target_height, presets.get("Floor"), presets.get("Ceiling"))));

    }

    public double get_target_height()
    {
        return target_height;
    }

    public boolean stationary()
    {
        return !liftMotor.isBusy();
    }

    private int height_to_ticks(double height)
    {
        return (int) ((height - offset_from_floor) * TICKS_PER_MM);
    }

    private double ticks_to_height(int ticks)
    {
        return (ticks / TICKS_PER_MM) + offset_from_floor;
    }

    public Action liftToFloor()
    {
        return new LiftTo("Floor");
    }
    public Action liftToGrabbing()
    {
        return new LiftTo("Grabbing");
    }
    public Action liftToScanning()
    {
        return new LiftTo("Scanning");
    }
    public Action liftToLowBasket()
    {
        return new LiftTo("LowBasket");
    }
    public Action liftToHighBasket()
    {
        return new LiftTo("HighBasket");
    }
    public Action liftToLowChamber()
    {
        return new LiftTo("LowChamber");
    }
    public Action liftToLowChamberClipped()
    {
        return new LiftTo("LowChamberClipped");
    }
    public Action liftToHighChamber()
    {
        return new LiftTo("HighChamber");
    }
    public Action liftToHighChamberClipped()
    {
        return new LiftTo("HighChamberClipped");
    }
    public Action liftToCeiling()
    {
        return new LiftTo("Ceiling");
    }

    public void nextPresetPosition() {
        List<String> keys = new ArrayList<>(presets.keySet());
        for (int k = 0; k < keys.size(); k ++) {
            if (keys.get(k) == liftTargetPreset && k < keys.size() -1) {
                liftTargetPreset = keys.get(k + 1);
                move_to_preset(liftTargetPreset);
                return;
            }
        }
    }

    public void previousPresetPosition() {
        List<String> keys = new ArrayList<>(presets.keySet());
        for (int k = 0; k < keys.size(); k ++) {
            if (keys.get(k) == liftTargetPreset && k > 0) {
                liftTargetPreset = keys.get(k - 1);
                move_to_preset(liftTargetPreset);
                return;
            }
        }
    }
}
