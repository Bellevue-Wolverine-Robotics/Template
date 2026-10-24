package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import frc.robot.Robot;
import frc.robot.commands.AimPoseCommand;
import frc.robot.commands.AlignPoseCommand;
import frc.robot.constants.SwerveConstants;

import static edu.wpi.first.units.Units.*;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain = new SwerveDrivetrain<>(
        TalonFX::new, TalonFX::new, CANcoder::new,
        SwerveConstants.DRIVETRAIN_CONSTANTS,
        new SwerveModuleConstants[] {
            SwerveConstants.FRONT_LEFT_MODULE,
            SwerveConstants.FRONT_RIGHT_MODULE,
            SwerveConstants.BACK_LEFT_MODULE,
            SwerveConstants.BACK_RIGHT_MODULE
        }
    );

    private final SwerveRequest.FieldCentric fieldCentricRequest = new SwerveRequest.FieldCentric();
    private final SwerveRequest.ApplyFieldSpeeds fieldSpeedsRequest = new SwerveRequest.ApplyFieldSpeeds();
    private final SwerveRequest.ApplyRobotSpeeds robotSpeedsRequest = new SwerveRequest.ApplyRobotSpeeds();
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Field2d field = new Field2d();

    private final LEDSubsystem ledSubsystem;
    private final VisionSubsystem visionSubsystem;

    public SwerveSubsystem(LEDSubsystem ledSubsystem, VisionSubsystem visionSubsystem) {
        this.ledSubsystem = ledSubsystem;
        this.visionSubsystem = visionSubsystem;

        drivetrain.setStateStdDevs(SwerveConstants.STATE_STD_DEVS);

        try {
            RobotConfig config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                this::getPosition,
                drivetrain::resetPose,
                this::getVelocity,
                (speeds, feedforwards) -> drivetrain.setControl(
                    robotSpeedsRequest.withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    SwerveConstants.PATHPLANNER_TRANSLATIONAL_PID,
                    SwerveConstants.PATHPLANNER_ROTATIONAL_PID
                ),
                config,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this
            );
        } catch (Exception e) {
            DriverStation.reportError("Failed to initialize PathPlanner", e.getStackTrace());
        }

        SmartDashboard.putBoolean("Use Vision", true);

        // --- ELASTIC FIELD FIX ---
        // Forces WPILib to tag the Field2d object correctly for dashboards
        SmartDashboard.putData("Field", field);

        if (Robot.isSimulation()) {
            @SuppressWarnings("resource")
            Notifier simNotifier = new Notifier(() -> {
                drivetrain.updateSimState(0.004, RobotController.getBatteryVoltage());
                visionSubsystem.updateSimulation(getPosition());
            });

            simNotifier.startPeriodic(0.004);
        }
    }

    @Logged(name = "Position")
    public Pose2d getPosition() {
        return drivetrain.getState().Pose;
    }

    @Logged(name = "Velocity")
    public ChassisSpeeds getVelocity() {
        return drivetrain.getState().Speeds;
    }

    @Logged(name = "Measured States")
    public SwerveModuleState[] getMeasuredStates() {
        return drivetrain.getState().ModuleStates;
    }

    @Logged(name = "Target States")
    public SwerveModuleState[] getTargetStates() {
        return drivetrain.getState().ModuleTargets;
    }


    @Logged(name = "Widget")
    public Sendable getWidget() {
        return new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");

                builder.addDoubleProperty("Front Left Angle", () -> drivetrain.getState().ModuleStates[0].angle.getRadians(), null);
                builder.addDoubleProperty("Front Left Velocity", () -> drivetrain.getState().ModuleStates[0].speedMetersPerSecond, null);

                builder.addDoubleProperty("Front Right Angle", () -> drivetrain.getState().ModuleStates[1].angle.getRadians(), null);
                builder.addDoubleProperty("Front Right Velocity", () -> drivetrain.getState().ModuleStates[1].speedMetersPerSecond, null);

                builder.addDoubleProperty("Back Left Angle", () -> drivetrain.getState().ModuleStates[2].angle.getRadians(), null);
                builder.addDoubleProperty("Back Left Velocity", () -> drivetrain.getState().ModuleStates[2].speedMetersPerSecond, null);

                builder.addDoubleProperty("Back Right Angle", () -> drivetrain.getState().ModuleStates[3].angle.getRadians(), null);
                builder.addDoubleProperty("Back Right Velocity", () -> drivetrain.getState().ModuleStates[3].speedMetersPerSecond, null);

                builder.addDoubleProperty("Robot Angle", () -> drivetrain.getState().Pose.getRotation().getRadians(), null);
            }
        };
    }

    public double scaleTranslationalDutyCycle(DoubleSupplier input) {
        return input.getAsDouble() * SwerveConstants.MAXIMUM_TRANSLATIONAL_VELOCITY.in(MetersPerSecond);
    }

    public void driveRobotCentric(ChassisSpeeds velocity) {
        drivetrain.setControl(robotSpeedsRequest.withSpeeds(velocity));
    }

    public void driveFieldCentric(ChassisSpeeds velocity) {
        drivetrain.setControl(
            fieldSpeedsRequest
                .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
                .withSpeeds(velocity)
        );
    }

    public void brake() {
        drivetrain.setControl(brake);
    }

    public Command driveCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta) {
        return run(() -> drivetrain.setControl(
            fieldCentricRequest
                .withVelocityX(scaleTranslationalDutyCycle(x))
                .withVelocityY(scaleTranslationalDutyCycle(y))
                .withRotationalRate(theta.getAsDouble() * SwerveConstants.MAXIMUM_ANGULAR_VELOCITY.in(RadiansPerSecond))
        )).withName("Field Centric Drive");
    }

    public Command aimCommand(Supplier<Pose2d> target, DoubleSupplier x, DoubleSupplier y) {
        return new AimPoseCommand(this, ledSubsystem, target, x, y).withName("Aim Position");
    }

    public Command alignCommand(Supplier<Pose2d> target) {
        return new AlignPoseCommand(this, ledSubsystem, target).withName("Align Position");
    }

    public Command zeroGyroCommand() {
        return runOnce(drivetrain::seedFieldCentric).withName("Zero Gyroscope");
    }

    @Override
    public void periodic() {
        DriverStation.getAlliance().ifPresent(
            allianceColor -> drivetrain.setOperatorPerspectiveForward(
                allianceColor == Alliance.Red ? Rotation2d.k180deg : Rotation2d.kZero
            )
        );

        if (SmartDashboard.getBoolean("Use Vision", true)) {
            List<VisionSubsystem.PoseEstimate> estimates = visionSubsystem.getEstimates();

            for (VisionSubsystem.PoseEstimate estimate : estimates) {
                drivetrain.addVisionMeasurement(estimate.pose, estimate.timestamp, estimate.stdDevs);
            }
        }

        field.setRobotPose(getPosition());
    }
}
