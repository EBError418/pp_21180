package org.firstinspires.ftc.teamcode.pedro.procedures.robot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
public class Launcher {
    Servo switchServo;
    public Launcher(HardwareMap hardwareMap) {
        switchServo = hardwareMap.get(Servo.class, "switchServo");
    }
    public void setLauncherPower(double power) {
    }
    public void setLauncherVelocity(double vel) {
    }
    public double getLauncherPower() {
        return 0.0;
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