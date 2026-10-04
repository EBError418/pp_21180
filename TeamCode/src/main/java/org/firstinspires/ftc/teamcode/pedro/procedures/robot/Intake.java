package org.firstinspires.ftc.teamcode.pedro.procedures.robot;

//import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.util.MathHelpers;
//import org.firstinspires.ftc.teamcode.util.hardware.ServoEx;

//import smile.interpolation.LinearInterpolation;

public class Intake {

    //hardware
    DcMotor initIntakeMotor;
    DcMotor runningIntakeMotor;

    public Intake(HardwareMap hardwareMap) {
        initIntakeMotor = hardwareMap.get(DcMotor.class, "intakeL");
        runningIntakeMotor = hardwareMap.get(DcMotor.class, "intakeR");
    }

    public void initIntake(double power) {
        initIntakeMotor.setPower(-power);
    }

    public void runningIntake(double power) {
        runningIntakeMotor.setPower(-power);
    }

    public double getInitIntakePower() {
        return initIntakeMotor.getPower();
    }

    public double getRunningIntakePower() {
        return runningIntakeMotor.getPower();
    }
}
