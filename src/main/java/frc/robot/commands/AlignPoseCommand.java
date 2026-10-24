package frc.robot.commands;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.constants.AlignmentConstants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import static edu.wpi.first.units.Units.*;

public class AlignPoseCommand extends Command {
    private final HolonomicDriveController controller = new HolonomicDriveController(
        new PIDController(
            AlignmentConstants.TRANSLATIONAL_PID_KP,
            AlignmentConstants.TRANSLATIONAL_PID_KI,
            AlignmentConstants.TRANSLATIONAL_PID_KD
        ),
        new PIDController(
            AlignmentConstants.TRANSLATIONAL_PID_KP,
            AlignmentConstants.TRANSLATIONAL_PID_KI,
            AlignmentConstants.TRANSLATIONAL_PID_KD
        ),
        new ProfiledPIDController(
            AlignmentConstants.ROTATIONAL_PID_KP,
            AlignmentConstants.ROTATIONAL_PID_KI,
            AlignmentConstants.ROTATIONAL_PID_KD,
            new TrapezoidProfile.Constraints(
                AlignmentConstants.MAXIMUM_ANGULAR_VELOCITY.in(RadiansPerSecond),
                AlignmentConstants.MAXIMUM_ANGULAR_ACCELERATION.in(RadiansPerSecondPerSecond)
            )
        )
    );

    private final TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        AlignmentConstants.MAXIMUM_TRANSLATIONAL_VELOCITY,
        AlignmentConstants.MAXIMUM_TRANSLATIONAL_ACCELERATION
        );

    private final Timer timer = new Timer();
    private final SwerveSubsystem swerveSubsystem;
    private final LEDSubsystem ledSubsystem;
    private final Supplier<Pose2d> target;
    private Trajectory trajectory;

    /**
     * Constructs a new AlignPoseCommand
     *
     * @param swerveSubsystem The swerve subsystem.
     * @param ledSubsystem The LED subsystem.
     * @param target The target position to drive to.
     */
    public AlignPoseCommand(
        SwerveSubsystem swerveSubsystem,
        LEDSubsystem ledSubsystem,
        Supplier<Pose2d> target
    ) {
        this.swerveSubsystem = swerveSubsystem;
        this.ledSubsystem = ledSubsystem;
        this.target = target;

        controller.setTolerance(AlignmentConstants.POSITION_TOLERANCE);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        ChassisSpeeds currentVelocity = swerveSubsystem.getVelocity();
        Pose2d currentPosition = swerveSubsystem.getPosition();
        Pose2d targetPosition = target.get();

        if (currentPosition.getTranslation().getDistance(targetPosition.getTranslation()) > AlignmentConstants.MINIMUM_TRAJECTORY_DISTANCE.in(Meters)) {
            double velocityMagnitute = Math.hypot(
                currentVelocity.vxMetersPerSecond,
                currentVelocity.vyMetersPerSecond
            );

            Rotation2d endHeading = targetPosition.getTranslation().minus(currentPosition.getTranslation()).getAngle();
            Rotation2d startHeading;

            if (velocityMagnitute < AlignmentConstants.STARTING_VELOCITY_TOLERANCE.in(MetersPerSecond)) {
                startHeading = endHeading;
            } else {
                startHeading = new Rotation2d(currentVelocity.vxMetersPerSecond, currentVelocity.vyMetersPerSecond);
            }

            Pose2d startPosition = new Pose2d(
                currentPosition.getTranslation(),
                startHeading
            );

            Pose2d endPosition = new Pose2d(
                targetPosition.getTranslation(),
                endHeading
            );

            trajectoryConfig.setStartVelocity(velocityMagnitute);

            trajectory = TrajectoryGenerator.generateTrajectory(
                List.of(startPosition, endPosition),
                trajectoryConfig
            );
        } else {
            trajectory = new Trajectory(List.of(
                new Trajectory.State(0.0, 0.0, 0.0, targetPosition, 0.0)
            ));
        }

        controller.getThetaController().reset(
            swerveSubsystem.getPosition().getRotation().getRadians(),
            currentVelocity.omegaRadiansPerSecond
        );

        timer.restart();
        controller.getXController().reset();
        controller.getYController().reset();
        ledSubsystem.setAligning(true);
    }

    @Override
    public void execute() {
        Pose2d current = swerveSubsystem.getPosition();
        ChassisSpeeds chassisSpeeds = controller.calculate(
            current,
            trajectory.sample(timer.get()),
            target.get().getRotation()
        );
        swerveSubsystem.driveRobotCentric(chassisSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        ledSubsystem.setAligning(false);
    }

    @Override
    public boolean isFinished() {
        return timer.get() > trajectory.getTotalTimeSeconds() && controller.atReference();
    }
}
