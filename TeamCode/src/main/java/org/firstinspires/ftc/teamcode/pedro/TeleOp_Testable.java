package org.firstinspires.ftc.teamcode.pedro.procedures;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "TeleOp 27 New Robot", group = "Teleop")
public class TeleOp_Testable extends LinearOpMode {
    private Follower follower;
    private boolean fieldCentric = true;
    private boolean lastBackState = false;

    @Override
    public void runOpMode() {
        follower = Constants.create(hardwareMap);

        // Reset starting pose/heading to 0
        follower.setPose(new Pose(0, 0, 0));

        telemetry.addLine("Status: Ready for TeleOp");
        telemetry.addLine("Controls:");
        telemetry.addLine("  Left Stick Y  -> Forward / Backward");
        telemetry.addLine("  Left Stick X  -> Strafe Left / Right");
        telemetry.addLine("  Right Stick X -> Turn Left / Right");
        telemetry.addLine("  START Button  -> Reset Heading to 0");
        telemetry.addLine("  BACK Button   -> Toggle Field-Centric / Robot-Centric");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            double forward = gamepad1.left_stick_y; // Push up -> Forward
            double strafe = -gamepad1.left_stick_x;   // Push right -> Strafe Right
            double turn = gamepad1.right_stick_x;     // Push right -> Turn Right

            // Toggle Field-Centric vs Robot-Centric with gamepad1.back
            if (gamepad1.back && !lastBackState) {
                fieldCentric = !fieldCentric;
            }
            lastBackState = gamepad1.back;

            // Reset heading to 0 when START is pressed
            if (gamepad1.start) {
                follower.setHeading(0);
            }

            if (fieldCentric) {
                // Transform joystick inputs into Field-Centric drive powers using current heading
                double heading = follower.pose().heading();
                double fieldForward = forward * Math.cos(-heading) - strafe * Math.sin(-heading);
                double fieldStrafe = forward * Math.sin(-heading) + strafe * Math.cos(-heading);
                follower.manual(fieldForward, fieldStrafe, turn);
            } else {
                // Robot-Centric drive
                follower.manual(forward, strafe, turn);
            }

            follower.update();

            telemetry.addData("Drive Mode", fieldCentric ? "Field-Centric" : "Robot-Centric");
            telemetry.addData("X (in)", follower.pose().x());
            telemetry.addData("Y (in)", follower.pose().y());
            telemetry.addData("Heading (deg)", Math.toDegrees(follower.pose().heading()));
            telemetry.update();
        }
    }
}