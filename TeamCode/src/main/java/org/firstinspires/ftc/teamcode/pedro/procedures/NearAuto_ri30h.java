package org.firstinspires.ftc.teamcode.pedro.procedures;

import static com.pedropathing.api.Paths.curve;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.procedures.robot.Launcher;
import org.firstinspires.ftc.teamcode.pedro.procedures.robot.Intake;

/**
 * A simple autonomous OpMode that moves the robot forward 5 inches using Pedro Pathing.
 */
@Autonomous(name = "Near auto in pp", group = "Autonomous")
public class NearAuto_ri30h extends LinearOpMode {
    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose start = poseFactory.of(81, 10, 90);
    private final Pose path1 = poseFactory.of(125, 26, 180);
    private final Pose path1Control1 = poseFactory.of(85, 33, 0);

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
        follower.setPose(start);

        // Create a path that moves to the target position.
        Path forward = curve(start, path1Control1, path1).linear(start, path1);

        telemetry.addLine("Initialized. Press Play to start.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Near Auto Logic: Wait 10 seconds before launching
        telemetry.addLine("Status: Waiting 8 seconds...");
        telemetry.update();
        sleep(10000);

        // Launch Sequence
        telemetry.addLine("Status: Spooling up Launcher & Intake...");
        telemetry.update();
        launcher.setLauncherVelocity(6000);
        intake.runningIntake(1.0); // High power during spool up
        sleep(2500);

        telemetry.addLine("Status: Launching (Switch at 0.2)...");
        telemetry.update();
        launcher.switchOpen();
        sleep(2000);

        telemetry.addLine("Status: Resetting Switch (Switch at 0.8)...");
        telemetry.update();
        launcher.switchClose();
        //sleep(500);

        telemetry.addLine("Status: Stopping Launcher & Intake.");
        telemetry.update();
        launcher.setLauncherVelocity(0);
        intake.runningIntake(0.0);

        // Start following the path.
        telemetry.addLine("Status: Starting Movement...");
        telemetry.update();
        follower.follow(forward);

        // Update the follower until it reaches the end of the path.
        while (opModeIsActive() && !isStopRequested()) {
            follower.update();

            // Check if the path is finished.
            if (follower.atParametricEnd()) {
                break;
            }

            // Optional telemetry to monitor progress.
            telemetry.addData("Status", "Driving Path");
            telemetry.addData("X", follower.pose().x());
            telemetry.addData("Y", follower.pose().y());
            telemetry.addData("Heading", follower.pose().heading());
            telemetry.update();
        }

        telemetry.addLine("Autonomous Complete.");
        telemetry.update();
    }
}
