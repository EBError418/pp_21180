package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static double HALFMAT = 12;
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig));
    }
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("FrontLeft");
        c.frontRightName.set("FrontRight");
        c.backLeftName.set("BackLeft");
        c.backRightName.set("BackRight");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(6.25);
        c.yPodOffset.set(-6.65);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.6097242416945863);
                Controller secondaryTranslationalForward = Controller.proportional(0.22527681517435502);
                Controller primaryTranslationalLateral = Controller.proportional(1.015123602352198);
                Controller secondaryTranslationalLateral = Controller.proportional(0.37506104646692134);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.014762952728847123));
                c.brake.set(Controller.proportionalFeedforward(0.012548509819520054));

                c.headingFeedback.set(Controller.proportional(1.0)); //73.89651802712554
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06069060450433853, 0.006059490393091433));

                c.linearBrakeCoefficients.set(Matrix.diag(0.24951078921499587, 0.08156911811396089));
                c.quadraticBrakeCoefficients.set(Matrix.diag(-2.6959477765456597E-4, 0.004296672639708987));

                c.maxAchievableForwardVelocity.set(69.49439315693988);
                c.maxAchievableStrafeVelocity.set(56.03263068458747);
                c.naturalForwardDeceleration.set(28.861703315247443);
                c.naturalStrafeDeceleration.set(47.41626176663805);
            }
    );
}