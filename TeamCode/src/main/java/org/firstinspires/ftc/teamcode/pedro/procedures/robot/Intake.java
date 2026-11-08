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
    DcMotor intakeLeft;
    DcMotor intakeRight;

    public Intake(HardwareMap hardwareMap) {
        intakeLeft = hardwareMap.get(DcMotor.class, "intakeL");
        intakeRight = hardwareMap.get(DcMotor.class, "intakeR");
    }

    public void runningFrontIntake(double power) {
        intakeLeft.setPower(-power);
    }

    public void runningBackIntake(double power) {
        intakeRight.setPower(-power);
    }

    public double getRightIntakePower() {
        return intakeRight.getPower();
    }

    public double getLeftIntakePower() {
        return intakeRight.getPower();
    }
}
