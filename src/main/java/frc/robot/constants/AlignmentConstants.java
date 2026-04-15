package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

public class AlignmentConstants {
    public static final double TRANSLATIONAL_PID_KP = 3.0;
    public static final double TRANSLATIONAL_PID_KI = 0.0;
    public static final double TRANSLATIONAL_PID_KD = 0.15;

    public static final double ROTATIONAL_PID_KP = 3.0;
    public static final double ROTATIONAL_PID_KI = 0.0;
    public static final double ROTATIONAL_PID_KD = 0.15;

    public static final AngularVelocity MAXIMUM_ANGULAR_VELOCITY = DegreesPerSecond.of(360.0);
    public static final AngularAcceleration MAXIMUM_ANGULAR_ACCELERATION = DegreesPerSecondPerSecond.of(720.0);

    public static final LinearVelocity MAXIMUM_LINEAR_VELOCITY = MetersPerSecond.of(3.0);
    public static final LinearAcceleration MAXIMUM_LINEAR_ACCELERATION  = MetersPerSecondPerSecond.of(5.0);

    public static final Pose2d TOLERANCE = new Pose2d(
        new Translation2d(Inches.of(1.0), Inches.of(1.0)),
        new Rotation2d(Degrees.of(5.0))
    );
}
