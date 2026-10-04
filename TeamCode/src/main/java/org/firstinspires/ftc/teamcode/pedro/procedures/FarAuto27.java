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
 * A simple autonomous OpMode that moves the robot forward 5 inches using Pedro Pathing.
 */
@Autonomous(name = "Far auto 27 in pp", group = "Autonomous")
public class FarAuto27 extends LinearOpMode {
    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(4 * Params.HALF_MAT + Params.CHASSIS_HALF_WIDTH, Params.CHASSIS_HALF_LENGTH, 90);
    private final Pose path1 = poseFactory.of(4 * Params.HALF_MAT+Params.CHASSIS_HALF_WIDTH, Params.CHASSIS_HALF_LENGTH, 90);
    private final Pose point2 = poseFactory.of(16.161, 47.5841, -178.5441);
    private final Pose point2Control1 = poseFactory.of(47.7994, 48.5932, 0);
    private final Pose point3 = poseFactory.of(58.8433, 131.4224, -90);
    private final Pose point3Control1 = poseFactory.of(38.372, 98.761, 0);
    private final Pose point4 = poseFactory.of(58.8, 129.0073, -91.027);
    private final Pose point5 = poseFactory.of(13.2872, 94.8395, -143.1034);

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

        // Create paths that move to target positions.
        Path path2 = curve(path1, point2Control1, point2).tangent();
        Path path3 = curve(point2, point3Control1, point3).linear(point2.heading(), point3.heading());
        Path path4 = line(point3, point4).tangent();
        Path path5 = line(point4, point5).tangent();
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
        //launcher.switchClose();
        //sleep(500); // Small delay for servo reset

        telemetry.addLine("Status: Stopping Launcher & Intake.");
        telemetry.update();
        launcher.setLauncherPower(0.0);
        intake.runningIntake(0.0);

        // Start following the path.
        telemetry.addLine("Status: Starting Movement...");
        telemetry.update();
        follower.follow(path2);
        sleep(2500);
        follower.follow(path3);
        sleep(2500);
        follower.follow(path5);

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
