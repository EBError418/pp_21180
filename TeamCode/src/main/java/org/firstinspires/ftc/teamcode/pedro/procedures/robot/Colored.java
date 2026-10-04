package org.firstinspires.ftc.teamcode.pedro.procedures.robot;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.qualcomm.robotcore.hardware.Servo;
import com.pedropathing.math.Pose;
import java.util.ArrayList;
import java.util.List;

public final class Colored {
    public final Limelight3A limelight;
    public Servo indicator;
    private int currentPipeline = -1;

    private static final double RED = 0.279;
    private static final double YELLOW = 0.388;
    private static final double GREEN = 0.500;
    private static final double BLUE = 0.87;
    private static final double WHITE = 1.0;

    public Colored(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(2);
        limelight.pipelineSwitch(1);
        rotateLimelight(1);

        indicator = hardwareMap.get(Servo.class, "indicator");
        indicator.setPosition(WHITE);

    }

    public void colorIndicator(int color) {
        switch (color) {
            case 0: indicator.setPosition(WHITE); break;
            case 1: indicator.setPosition(GREEN); break;
            case 2: indicator.setPosition(RED); break;
        }
    }

    public Double findGoalTag(int blueOrRed) {
        int desiredTag = (blueOrRed == 1) ? 20 : 24;
//        rotateLimelight(1); // Safely ensures we are on pipeline 1 without spamming
        limelight.pipelineSwitch(1);

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;

        for (LLResultTypes.FiducialResult f : result.getFiducialResults()) {
            if (f.getFiducialId() == desiredTag) {
                return f.getTargetXDegrees();
            }
        }
        return null;
    }

    public void setOrientation(double robotYaw) {
        limelight.updateRobotOrientation(robotYaw);
    }

    /**
     * Updates Limelight orientation using a Pedro Pathing Pose.
     * Converts radians to degrees for the Limelight.
     */
    public void updatePose(Pose pose) {
        limelight.updateRobotOrientation(Math.toDegrees(pose.heading()));
    }

    public Pose3D returnPositionbeta() {
        LLResult result = limelight.getLatestResult();
        return (result != null && result.isValid()) ? result.getBotpose() : null;
    }

    /**
     * Returns the field-relative pose of the robot as a Pedro Pathing Pose.
     * Converts Limelight meters to inches.
     */
    public Pose getPedroPose() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            // Convert meters to inches: 1m = 39.3701 inches
            return new Pose(
                botpose.getPosition().x * 39.3701,
                botpose.getPosition().y * 39.3701,
                Math.toRadians(botpose.getOrientation().getYaw())
            );
        }
        return null;
    }

    public void rotateLimelight(int orient) {
        int pipeline = (orient == 0) ? 2 : 1;
        if (currentPipeline != pipeline) { // Only switch if needed!
            limelight.pipelineSwitch(pipeline);
            currentPipeline = pipeline;
        }
    }

    public double[] returnClosestBall() {
        limelight.pipelineSwitch(2);
        for (int i = 0; i < 10; i++) {
            LLResult result = limelight.getLatestResult();
            double closestY = Double.MAX_VALUE, x = 0, y = 0, classId = 0, confidence = 0;

            for (LLResultTypes.DetectorResult detection : result.getDetectorResults()) {
                if (detection.getTargetYDegrees() < closestY) {
                    closestY = detection.getTargetYDegrees();
                    x = detection.getTargetXDegrees();
                    y = detection.getTargetYDegrees();
                    classId = detection.getClassId();
                    confidence = detection.getConfidence();
                }
            }
            if (closestY != Double.MAX_VALUE) {
                return new double[]{x, y, classId, confidence};
            }
        }
        return new double[]{0, 0, 0, 0};
    }

    public double[] returnClosestBallPreferringClusters() {
        limelight.pipelineSwitch(2);
        final int max_attempts = 10;
        final double confidence_thres = 0.6;
        final double proximity_thres = 7.0;
        final int min_cluster_size = 2;

        for (int attempt = 0; attempt < max_attempts; attempt++) {
            LLResult result = limelight.getLatestResult();
            List<LLResultTypes.DetectorResult> balls = new ArrayList<>();

            for (LLResultTypes.DetectorResult d : result.getDetectorResults()) {
                if (d.getConfidence() >= confidence_thres) balls.add(d);
            }

            if (!balls.isEmpty()) {
                List<List<LLResultTypes.DetectorResult>> clusters = new ArrayList<>();

                for (LLResultTypes.DetectorResult ball : balls) {
                    boolean added = false;
                    for (List<LLResultTypes.DetectorResult> cluster : clusters) {
                        LLResultTypes.DetectorResult reference = cluster.get(0);
                        if (Math.hypot(ball.getTargetXDegrees() - reference.getTargetXDegrees(),
                                ball.getTargetYDegrees() - reference.getTargetYDegrees()) <= proximity_thres) {
                            cluster.add(ball);
                            added = true;
                            break;
                        }
                    }
                    if (!added) {
                        List<LLResultTypes.DetectorResult> new_cluster = new ArrayList<>();
                        new_cluster.add(ball);
                        clusters.add(new_cluster);
                    }
                }

                List<LLResultTypes.DetectorResult> best_cluster = null;
                double best_cluster_distance = Double.MAX_VALUE;

                for (List<LLResultTypes.DetectorResult> cluster : clusters) {
                    if (cluster.size() < min_cluster_size) continue;
                    double sum_x = 0, sum_y = 0;
                    for (LLResultTypes.DetectorResult b : cluster) {
                        sum_x += b.getTargetXDegrees();
                        sum_y += b.getTargetYDegrees();
                    }
                    double distance = Math.hypot(sum_x / cluster.size(), sum_y / cluster.size());
                    if (distance < best_cluster_distance) {
                        best_cluster_distance = distance;
                        best_cluster = cluster;
                    }
                }

                if (best_cluster != null) {
                    double sum_x = 0, sum_y = 0, sum_confidence = 0;
                    for (LLResultTypes.DetectorResult b : best_cluster) {
                        sum_x += b.getTargetXDegrees();
                        sum_y += b.getTargetYDegrees();
                        sum_confidence += b.getConfidence();
                    }
                    return new double[] {sum_x / best_cluster.size(), sum_y / best_cluster.size(), 0, sum_confidence / best_cluster.size()};
                }

                LLResultTypes.DetectorResult closest_ball = null;
                double closest_distance = Double.MAX_VALUE;
                for (LLResultTypes.DetectorResult b : balls) {
                    double distance = Math.hypot(b.getTargetXDegrees(), b.getTargetYDegrees());
                    if (distance < closest_distance) {
                        closest_distance = distance;
                        closest_ball = b;
                    }
                }
                if (closest_ball != null) {
                    return new double[] {closest_ball.getTargetXDegrees(), closest_ball.getTargetYDegrees(), closest_ball.getClassId(), closest_ball.getConfidence()};
                }
            }
            try { sleep(20); } catch (InterruptedException e) { break; }
        }
        return null;
    }

    /**
     * Detects up to 4 AprilTags and returns the robot's field-relative Pose.
     * Filtered by redOrBlue:
     * -1: IDs 30-37
     *  1: IDs 38-45
     */
    public Pose getMultiTagPose(int redOrBlue) {
        limelight.pipelineSwitch(1);
        LLResult result = limelight.getLatestResult();
        
        if (result == null || !result.isValid()) return null;

        boolean tagMatchFound = false;
        for (LLResultTypes.FiducialResult f : result.getFiducialResults()) {
            int id = f.getFiducialId();
            if (redOrBlue == -1) {
                if (id >= 30 && id <= 37) { tagMatchFound = true; break; }
            } else if (redOrBlue == 1) {
                if (id >= 38 && id <= 45) { tagMatchFound = true; break; }
            }
        }

        if (tagMatchFound) {
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                return new Pose(
                    botpose.getPosition().x * 39.3701,
                    botpose.getPosition().y * 39.3701,
                    Math.toRadians(botpose.getOrientation().getYaw())
                );
            }
        }
        return null;
    }

    public double[] returnPosition(int counter) {
        limelight.pipelineSwitch(1);
        double [] arr = new double[6];
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                int fiducialId = fiducial.getFiducialId();
                if (fiducialId == 20 || fiducialId == 24) {
                    double x = fiducial.getTargetXDegrees();
                    double y = fiducial.getTargetYDegrees();
                    double z = fiducial.getTargetArea();
                    Pose3D pose = fiducial.getTargetPoseCameraSpace();

                    double dx = pose.getPosition().x, dy = pose.getPosition().y, dz = pose.getPosition().z;
                    double d = Math.sqrt(dx*dx + dz*dz);

                    indicator.setPosition(Math.abs(x) < 4.0 ? GREEN : YELLOW);
                    return new double[]{x, y, z, d, dx, dy, dz};
                }
            }
        }

        indicator.setPosition(RED);
        return arr;
    }

    public double returnId(boolean goal) {
        limelight.pipelineSwitch(1);
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                int id = fiducial.getFiducialId();
                if (goal && (id == 20 || id == 24)) return id;
                if (!goal && (id == 21 || id == 22 || id == 23)) return id;
            }
        }
        return 0;
    }

    public String GetDiagnostics() { return new LLStatus().toString(); }
    public int returnPipeline() { return limelight.getLatestResult().getPipelineIndex(); }
}