package org.firstinspires.ftc.teamcode.pedro.procedures;

import com.pedropathing.follower.Follower;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import static com.pedropathing.api.Paths.line;

@Autonomous(name = "Forward auto in pp", group = "Autonomous")
public class AutoBasic extends LinearOpMode {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(53, 9, 90);
    private final Pose path1 = poseFactory.of(53, 36, 90);

    @Override
    public void runOpMode() {
        Follower follower = Constants.create(hardwareMap);

        follower.setPose(start);

        Path testPath = line(start, path1).linear(start, path1);

        telemetry.addLine("Initialized. Press Play to start.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        follower.follow(testPath);

        while (opModeIsActive() && !isStopRequested()) {
            follower.update();

            if (follower.atParametricEnd()) {
                break;
            }

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
