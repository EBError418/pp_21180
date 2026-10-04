package org.firstinspires.ftc.teamcode.pedro.procedures.robot;

//import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.util.MathHelpers;
//import org.firstinspires.ftc.teamcode.util.hardware.ServoEx;

//import smile.interpolation.LinearInterpolation;

public class Launcher {

    //hardware
    DcMotorEx launcherMotor;
    DcMotorEx launcherMotor2;
    Servo switchServo;

    public Launcher(HardwareMap hardwareMap) {
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcherL");
        launcherMotor2 = hardwareMap.get(DcMotorEx.class, "launcherR");
        switchServo = hardwareMap.get(Servo.class, "switchServo");
    }

    public void setLauncherPower(double power) {
        launcherMotor.setPower(-power);
        launcherMotor2.setPower(power);
    }

    public void setLauncherVelocity(double vel) {
        launcherMotor.setVelocity(-vel);
        launcherMotor2.setVelocity(vel);
    }

    public double getLauncherPower() {
        return launcherMotor.getPower();
    }

    public void setSwitchPos(double pos) {
        switchServo.setPosition(pos);
    }

    public void switchOpen() { setSwitchPos(0.0); }

    public void switchClose() { setSwitchPos(0.25); }

    public double getSwitchPos() {
        return switchServo.getPosition();
    }
}
