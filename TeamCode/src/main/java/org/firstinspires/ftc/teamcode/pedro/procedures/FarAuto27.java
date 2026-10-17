package org.firstinspires.ftc.teamcode.pedro.procedures;

import com.pedropathing.follower.Follower;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.procedures.robot.Launcher;
import org.firstinspires.ftc.teamcode.pedro.procedures.robot.Intake;
import org.firstinspires.ftc.teamcode.pedro.procedures.robot.Params;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

/**
 * A simple autonomous OpMode for ERROR418 2026-2027 season.
 */
@Autonomous(name = "Far Auto 27", group = "Autonomous")
public class FarAuto27 extends LinearOpMode {
    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose path1Start = poseFactory.of(56, 8, 90);
    private final Pose path1 = poseFactory.of(34.6881, 35.1232, 180);
    private final Pose path1Control1 = poseFactory.of(54.3529, 35.2088, 0);

    @Override
    public void runOpMode() {
        // Initialize the follower using the Constants class.
        follower = Constants.create(hardwareMap);
        Launcher launcher = new Launcher(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        if (follower == null) {
            telemetry.addLine("Error: Follower not initialized in Constants.java");
            telemetry.update();
            launcher.switchClose();
            waitForStart();
            return;
        }

        // Set the starting pose.
        follower.setPose(path1Start);

        // Create a path that moves to the target position.
        Path forward = curve(path1Start, path1Control1, path1).linear(path1Start, path1);

        telemetry.addLine("Initialized. Press Play to start.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Perform Launch Sequence immediately
        telemetry.addLine("Status: Spooling up Launcher & Intake...");
        telemetry.update();
        //launcher.setLauncherVelocity(6000); // Full power for spool up
        //intake.runningIntake(1.0); // High power during spool up
        //sleep(2500); // Wait for spool up

        telemetry.addLine("Status: Launching (Switch at 0.2)...");
        telemetry.update();
        launcher.switchOpen();
        //sleep(2000); // Wait for sequence

        telemetry.addLine("Status: Resetting Switch (Switch at 0.8)...");
        telemetry.update();
        launcher.switchClose();
        //sleep(500); // Small delay for servo reset

        telemetry.addLine("Status: Stopping Launcher & Intake.");
        telemetry.update();
        launcher.setLauncherPower(0.0);
        intake.runningIntake(0.0);

        // Start following the path.
        telemetry.addLine("Status: Starting Movement...");
        telemetry.update();
        follower.follow(forward);

        // Update the follower until it reaches the end of the path.
        while (opModeIsActive() && !isStopRequested() && !follower.atParametricEnd()) {
            follower.update();

            // Optional telemetry to monitor progress.
            telemetry.addData("Status", "Driving Path");
            telemetry.addData("X", follower.pose().x());
            telemetry.addData("Y", follower.pose().y());
            telemetry.addData("Heading", follower.pose().heading());
            telemetry.update();
        }

        //telemetry.addData("Status", "Driving Path");
        telemetry.addData("end X", follower.pose().x());
        telemetry.addData("end Y", follower.pose().y());
        telemetry.addData("Heading", follower.pose().heading());
        telemetry.addData("Expected end X: ", path1.x());
        telemetry.addData("Expected end Y: ", path1.y());

        telemetry.addLine("Autonomous Complete.");
        telemetry.update();
    }
}
