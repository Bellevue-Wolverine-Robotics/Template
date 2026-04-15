package frc.robot.constants;

import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
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

    public static final int CAMERA_RESOLUTION_WIDTH = 1280;
    public static final int CAMERA_RESOLUTION_HEIGHT = 720;
    public static final double CAMERA_DIAGONAL_FOV = 68.5;
    public static final double CAMERA_AVERAGE_ERROR_PIXEL = 0.35;
    public static final double CAMERA_ERROR_STD_DEV_PIXEL = 0.10;
    public static final int CAMERA_FPS = 30;
    public static final int CAMERA_LATENCY_MS = 100;
    public static final int CAMERA_LATENCY_STD_DEV_MS = 30;

    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10.0));
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5.0));
    public static final Distance SINGLE_TAG_DISTANCE_THRESHOLD = Meters.of(4.0);
    public static final double STD_DEVS_SCALING_FACTOR = 30.0;

    public static final AprilTagFieldLayout TAG_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    private static final Pose3d ORIGIN_POSE = TAG_LAYOUT.getOrigin();

    private static final Pose2d getAllianceSpecificTagPose(int redTagId, int blueTagId, Transform2d transform) {
        boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red; 
        Pose2d tagPose = TAG_LAYOUT.getTagPose(isRed ? redTagId : blueTagId).orElse(ORIGIN_POSE).toPose2d();
        return tagPose.transformBy(transform);
    }
}
