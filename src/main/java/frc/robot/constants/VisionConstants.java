package frc.robot.constants;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

public class VisionConstants {
    public static class CameraProperties {
        public final String name;
        public final Transform3d robotToCamera;

        public CameraProperties(String name, Transform3d robotToCamera) {
            this.name = name;
            this.robotToCamera = robotToCamera;
        }
    }

    public static class CameraErrorStdDevModel {
        private final double betaA;
        private final double betaB;

        public CameraErrorStdDevModel(double betaA, double betaB) {
            this.betaA = betaA;
            this.betaB = betaB;
        }

        public double compute(double averageDistance, int count) {
            return betaA + betaB * Math.pow(averageDistance, 2) / Math.sqrt(count);
        }
    }

    public static class CameraErrorStdDevsModel {
        private final CameraErrorStdDevModel x;
        private final CameraErrorStdDevModel y;
        private final CameraErrorStdDevModel theta;

        public CameraErrorStdDevsModel(
            CameraErrorStdDevModel x,
            CameraErrorStdDevModel y,
            CameraErrorStdDevModel theta
        ) {
            this.x = x;
            this.y = y;
            this.theta = theta;
        }

        public Matrix<N3, N1> compute(double averageDistance, int count) {
            return VecBuilder.fill(
                x.compute(averageDistance, count),
                y.compute(averageDistance, count),
                theta.compute(averageDistance, count)
            );
        }
    }


    public static final List<CameraProperties> CAMERAS = List.of(
        new CameraProperties(
            "primary",
            new Transform3d(
                new Translation3d(Inches.of(-1.923), Inches.of(0.0), Inches.of(28.86)),
                new Rotation3d(Radians.of(0.0), Radians.of(-25.0), Radians.of(0.0))
            )
        ),
        new CameraProperties(
            "secondary",
            new Transform3d(
                new Translation3d(Inches.of(13.5), Inches.of(0.0), Inches.of(26.358)),
                new Rotation3d(Radians.of(0.0), Radians.of(0.0), Radians.of(0.0))
            )
        )
    );

    public static final CameraErrorStdDevsModel SINGLE_TAG_STD_DEVS = new CameraErrorStdDevsModel(
        new CameraErrorStdDevModel(0.1, 2.0),
        new CameraErrorStdDevModel(0.1, 2.0),
        new CameraErrorStdDevModel(0.1, 2.0)
    );

    public static final CameraErrorStdDevsModel MULTI_TAG_STD_DEVS = new CameraErrorStdDevsModel(
        new CameraErrorStdDevModel(0.1, 2.0),
        new CameraErrorStdDevModel(0.1, 2.0),
        new CameraErrorStdDevModel(0.1, 2.0)
    );

    public static final Distance SINGLE_TAG_DISTANCE_THRESHOLD = Meters.of(4.0);
    public static final double SINGLE_TAG_AMBIGUITY_THRESHOLD = 0.2;

    public static final int SIMULATION_CAMERA_RESOLUTION_WIDTH = 1280;
    public static final int SIMULATION_CAMERA_RESOLUTION_HEIGHT = 720;
    public static final double SIMULATION_CAMERA_DIAGONAL_FOV = 68.5;
    public static final double SIMULATION_CAMERA_AVERAGE_ERROR_PIXEL = 0.35;
    public static final double SIMULATION_CAMERA_ERROR_STD_DEV_PIXEL = 0.10;
    public static final int SIMULATION_CAMERA_FPS = 30;
    public static final int SIMULATION_CAMERA_LATENCY_MS = 100;
    public static final int SIMULATION_CAMERA_LATENCY_STD_DEV_MS = 30;

    public static final AprilTagFieldLayout TAG_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    private static final Pose3d ORIGIN_POSE = TAG_LAYOUT.getOrigin();

    private static final Pose2d getAllianceSpecificTagPose(int redTagId, int blueTagId, Transform2d transform) {
        boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        Pose2d tagPose = TAG_LAYOUT.getTagPose(isRed ? redTagId : blueTagId).orElse(ORIGIN_POSE).toPose2d();
        return tagPose.transformBy(transform);
    }

    public static final Supplier<Pose2d> HUB_POSE_SUPPLIER = () -> {
        Transform2d transform = new Transform2d(new Translation2d(Inches.of(-23.5), Inches.of(0.0)), new Rotation2d());
        return getAllianceSpecificTagPose(4, 20, transform);
    };

    public static final Supplier<Pose2d> OUTPOST_POSE_SUPPLIER = () -> {
        Transform2d transform = new Transform2d(
            Inches.of(17.0),
            Inches.of(0.0),
            new Rotation2d(Angle.ofRelativeUnits(180.0, Degrees))
        );

        return getAllianceSpecificTagPose(13, 29, transform);
    };
}
