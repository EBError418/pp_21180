package org.firstinspires.ftc.teamcode.pedro.procedures;


import com.pedropathing.follower.Follower;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.procedures.robot.Launcher;
import org.firstinspires.ftc.teamcode.pedro.procedures.robot.Intake;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

@TeleOp(name = "Tele op 2027 in pp", group = "Teleop")
public class TeleOp_ri30h extends LinearOpMode {
    private Follower follower;
    private Intake intake;
    private Launcher launcher;

    private boolean intakeActive = false;
    private boolean launcherSpooling = false;
    private ElapsedTime intakeTimer = new ElapsedTime();
    private ElapsedTime launchTimer = new ElapsedTime();

    private final double RUNNING_INTAKE_LOW = 0.3;
    private final double RUNNING_INTAKE_HIGH = 1.0;

    private final double INIT_INTAKE_LOW = 0.3;
    private final double INIT_INTAKE_HIGH = 0.8;

    @Override
    public void runOpMode() {
        follower = Constants.create(hardwareMap);
        intake = new Intake(hardwareMap);
        launcher = new Launcher(hardwareMap);

        telemetry.addLine("Status: Ready for TeleOp");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            follower.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            follower.update();

            // --- Intake Logic (Gamepad 2 - A) ---
            if (gamepad2.a || gamepad1.left_bumper && !intakeActive) {
                intakeActive = true;
                intakeTimer.reset();
                intake.initIntake(1.0);
            } else if (!gamepad2.a && intakeActive) {
                // Keep running until manually stopped or toggle logic? 
                // Let's assume Button B stops everything.
            }

            if (gamepad2.b || gamepad1.right_bumper) {
                intakeActive = false;
                intake.initIntake(0);
                intake.runningIntake(0);
                launcher.setLauncherPower(0);
            }

            if (intakeActive) {
//                if (intakeTimer.seconds() > 1.0) {
//                    intake.initIntake(0);
//                }

                // ----- Alternate logic -----
                double initIntakePower = launcherSpooling ? INIT_INTAKE_LOW : INIT_INTAKE_HIGH;
                intake.initIntake(initIntakePower);

                launcher.setLauncherVelocity(2000);

                // runningIntake power depends on launcher spooling state
                double intakePower = launcherSpooling ? RUNNING_INTAKE_HIGH : RUNNING_INTAKE_LOW;
                intake.runningIntake(intakePower);
            }

            // --- Launcher Spooling (Gamepad 2 - X) ---
            if (gamepad2.x || gamepad1.dpad_left) {
                launcherSpooling = true;
                launcher.setLauncherVelocity(6000);
                sleep(2000);
                launcher.switchOpen();
            }
            if (gamepad2.right_bumper || gamepad1.dpad_right) { // Stop launcher
                launcherSpooling = false;
                launcher.setLauncherVelocity(0);
                launcher.switchClose();
            }

            if (gamepad2.x) {
                launcher.setSwitchPos(launcher.getSwitchPos() - 0.01);
            }

            if (gamepad2.y) {
                launcher.setSwitchPos(launcher.getSwitchPos() + 0.01);
            }

            // --- Launch Sequence (Gamepad 2 - Y) ---
            if (gamepad2.y) {
                launchTimer.reset();
                launcher.setSwitchPos(0.2);
            }
            
            if (launchTimer.seconds() > 1.0 && launchTimer.seconds() < 1.5) {
                launcher.setSwitchPos(0.25);
            }

            telemetry.addData("Intake Active", intakeActive);
            telemetry.addData("Launcher Spooling", launcherSpooling);
            telemetry.addData("X", follower.pose().x());
            telemetry.addData("Y", follower.pose().y());
            telemetry.addData("Heading", follower.pose().heading());
            telemetry.addData("Switch pos", launcher.getSwitchPos());
            telemetry.update();
        }
    }
}
