package org.firstinspires.ftc.teamcode.pedro.procedures.robot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class Launcher {
    Servo switchServo;

    DcMotorEx launcherL;
    DcMotorEx launcherR;
    final double switchClosePos = 0.9;
    final double switchOpenPos = 0.63;
    final double launchVelYellowBall = 1500;
    public Launcher(HardwareMap hardwareMap) {

        launcherL = hardwareMap.get(DcMotorEx.class, "launchL");
        launcherR = hardwareMap.get(DcMotorEx.class, "launchR");

        launcherL.setDirection(DcMotorSimple.Direction.FORWARD);
        launcherR.setDirection(DcMotorSimple.Direction.REVERSE);

        launcherR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //update motor configs
        MotorConfigurationType config = launcherL.getMotorType().clone();
        config.setTicksPerRev(28);
        config.setMaxRPM(6000);
        launcherL.setMotorType(config);

        config = launcherR.getMotorType().clone();
        config.setTicksPerRev(28);
        config.setMaxRPM(6000);
        launcherR.setMotorType(config);

        // update launcher motor PID for quick ramp up and keep the speed
        launcherL.setVelocityPIDFCoefficients(100, 0.12, 1.2, 13.0);
        launcherR.setVelocityPIDFCoefficients(100, 0.12, 1.2, 13.0);

        switchServo = hardwareMap.get(Servo.class, "switchServo");
        switchClose();
    }
    private void setLauncherPower(double power) {

        launcherL.setPower(power);
        launcherR.setPower(power);
    }
    private void setLauncherVelocity(double vel) {
        launcherL.setVelocity(vel);
        launcherR.setVelocity(vel);
    }

    public void launcherOn() {
        setLauncherVelocity(launchVelYellowBall);
    }

    public void launcherOff() {
        setLauncherPower(0);
    }

    public double getLauncherPower() {
        return launcherR.getPower();
    }
    public double getLauncherVelocity() {
        return launcherR.getVelocity();
    }
    private void setSwitchPos(double pos) {
        switchServo.setPosition(pos);
    }
    public void switchOpen() { setSwitchPos(switchOpenPos); }
    public void switchClose() { setSwitchPos(switchClosePos); }
    public double getSwitchPos() {
        return switchServo.getPosition();
    }
}