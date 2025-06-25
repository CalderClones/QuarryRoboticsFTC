package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;

public class Lift {
    private static final double TICKS_PER_MM = 28.0 * (76.0 / 21.0) * (76.0 / 21.0) * (84.0 / 29.0) / 44.0 / 3.0 / 3.0;
    private final double SPEED = 30.0;
    public DcMotorEx liftMotor;
    public LinkedHashMap<String, Integer> presets = new LinkedHashMap<String, Integer>();
    public String liftTargetPreset = "Floor";
    private final TouchSensor limitSwitch;
    private final double offset_from_floor;
    private double target_height;
    private final double current_height;
    private final double tolerance;

    public Lift(HardwareMap hardwareMap) {
        //these presets need to be tuned empirically
        this.presets.put("Floor", 11);  //Arm measures 11mm above floor when horizontal at lift lower limit
        this.presets.put("LowChamberClipped", 60); //Calculated - will definitely need tuning
        this.presets.put("Grabbing", 120); //Arm measures 98mm above floor when horizontal and at the right height to grab a sample
        this.presets.put("LowChamber", 149); //Calculated - will definitely need tuning
        this.presets.put("Scanning", 250); // We measured 230mm as the best height for scanning
        this.presets.put("HighChamberClipped", 250); //Calculated - will definitely need tuning
        this.presets.put("HighChamber", 400); //Calculated - will definitely need tuning
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

    public void reset() {
        ElapsedTime timer = new ElapsedTime();
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setPower(0);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //retract the lift at low power for 2 seconds - should ensure it bottoms out without braking anything
        timer.reset();
        while (timer.milliseconds() < 2000) {
            liftMotor.setPower(-0.3);
        }

        //now very gently drive the lift upwards so that it isn't hard against the stop
        timer.reset();
        while (timer.milliseconds() < 2000) {
            liftMotor.setPower(0.05);
        }
        liftMotor.setPower(0);

        //finally, make this the lift's new zero position.
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(1.0);
    }

    public void softReset() {
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(1.0);
    }

    public boolean move_to_preset(String preset) {
        if (presets.containsKey(preset)) {
            liftTargetPreset = preset;
            setTargetHeight(presets.get(preset));
            return true;
        } else {
            return false;
        }

    }

    public double getCurrentHeight() {
        return ticks_to_height(liftMotor.getCurrentPosition());
    }

    public double getTargetHeight() {
        return target_height;
    }

    public void setTargetHeight(double height) {
        target_height = height;
        liftMotor.setTargetPosition(height_to_ticks(Range.clip(target_height, presets.get("Floor"), presets.get("Ceiling"))));

    }

    public boolean stationary() {
        return !liftMotor.isBusy();
    }

    private int height_to_ticks(double height) {
        return (int) ((height - offset_from_floor) * TICKS_PER_MM);
    }

    private double ticks_to_height(int ticks) {
        return (ticks / TICKS_PER_MM) + offset_from_floor;
    }

    public Action liftToFloor() {
        return new LiftTo("Floor");
    }

    public Action liftToGrabbing() {
        return new LiftTo("Grabbing");
    }

    public Action liftToScanning() {
        return new LiftTo("Scanning");
    }

    public Action liftToLowBasket() {
        return new LiftTo("LowBasket");
    }

    public Action liftToHighBasket() {
        return new LiftTo("HighBasket");
    }

    public Action liftToLowChamber() {
        return new LiftTo("LowChamber");
    }

    public Action liftToLowChamberClipped() {
        return new LiftTo("LowChamberClipped");
    }

    public Action liftToHighChamber() {
        return new LiftTo("HighChamber");
    }

    public Action liftToHighChamberClipped() {
        return new LiftTo("HighChamberClipped");
    }

    public Action liftToCeiling() {
        return new LiftTo("Ceiling");
    }

    public void nextPresetPosition() {
        List<String> keys = new ArrayList<>(presets.keySet());
        for (int k = 0; k < keys.size(); k++) {
            if (keys.get(k) == liftTargetPreset && k < keys.size() - 1) {
                liftTargetPreset = keys.get(k + 1);
                move_to_preset(liftTargetPreset);
                return;
            }
        }
    }

    /* Abandoned re-write of this method
    public void nextPresetPosition() {
        presets.forEach(new BiConsumer<String, Integer>() {
            boolean found = false;

            public void accept(String name, Integer height) {
                //find the first preset higher than the current height
                if (height > getCurrentHeight() && !found) {
                    found = true;
                    move_to_preset(name);
                }
            }
        });
    }

     */

    public void previousPresetPosition() {
        List<String> keys = new ArrayList<>(presets.keySet());
        for (int k = 0; k < keys.size(); k++) {
            if (keys.get(k) == liftTargetPreset && k > 0) {
                liftTargetPreset = keys.get(k - 1);
                move_to_preset(liftTargetPreset);
                return;
            }
        }
    }


/* Abandoned re-write of this method
    public void previousPresetPosition() {
        presets.forEach(new BiConsumer<String, Integer>() {
            boolean found = false;
            String new_position = "Floor";

            public void accept(String name, Integer height) {
                //find the last preset lower than the current height
                if (height > getCurrentHeight() && !found) {
                    found = true;
                    move_to_preset(new_position);
                }
                else {
                    new_position = name;
                }
            }
        });
    }

 */

    public void moveLift(double multiplier) {
        String nextPresetHeight = "Ceiling";
        String previousPresetHeight = "Floor";

        List<String> keys = new ArrayList<>(presets.keySet());
        for (int k = 0; k < keys.size(); k++) {
            if (keys.get(k) == liftTargetPreset && k > 0) {
                previousPresetHeight = keys.get(k - 1);
            }
            if (keys.get(k) == liftTargetPreset && k < keys.size() - 1) {
                nextPresetHeight = keys.get(k + 1);
            }
        }

        double desiredHeight = getCurrentHeight() + SPEED * multiplier;
        setTargetHeight(desiredHeight);

        //ensure that if the lift moves past a preset position, we update the preset. This avoids unexpected behaviour when switching between manually driving the lift and using presets.
        if (desiredHeight >= presets.get(nextPresetHeight)) {
            liftTargetPreset = nextPresetHeight;
        } else if (desiredHeight <= presets.get(liftTargetPreset)) {
            liftTargetPreset = previousPresetHeight;
        }


    }

    public class LiftTo implements Action {
        private boolean initialized = false;
        private final String preset;

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
            //This needs to return true when action is still running and false when not.
            return !stationary();
        }
    }
}
