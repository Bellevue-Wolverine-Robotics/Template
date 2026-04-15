package frc.robot.subsystems;

import java.io.File;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;

import frc.robot.constants.SwerveConstants;

import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

public class SwerveSubsystem extends SubsystemBase {
    private final VisionSubsystem visionSubsystem;
    private final SwerveDrive swerveDrive;

    public SwerveSubsystem(VisionSubsystem visionSubsystem) {
        this.visionSubsystem = visionSubsystem;

        SimulatedArena.overrideInstance(SwerveConstants.SIMULATED_ARENA);

        File swerveJsonDirectory = new File(
            Filesystem.getDeployDirectory(),
            Preferences.getString("swerveDirectory", "swerve")
        );

        try {
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(
                SwerveConstants.MAXIMUM_VELOCITY.in(MetersPerSecond)
            );
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        swerveDrive.chassisVelocityCorrection = true;
        swerveDrive.setHeadingCorrection(true);
        swerveDrive.setChassisDiscretization(true, 0.02);

        try {
            RobotConfig config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                swerveDrive::getPose,
                swerveDrive::resetOdometry,
                swerveDrive::getRobotVelocity,
                (speeds, feedforwards) -> {
                    swerveDrive.drive(
                        speeds,
                        swerveDrive.kinematics.toSwerveModuleStates(speeds),
                        feedforwards.linearForces()
                    );
                },
                new PPHolonomicDriveController(
                    new PIDConstants(
                        SwerveConstants.PATHPLANNER_TRANSLATIONAL_PID_KP,
                        SwerveConstants.PATHPLANNER_TRANSLATIONAL_PID_KI,
                        SwerveConstants.PATHPLANNER_TRANSLATIONAL_PID_KD
                    ),
                    new PIDConstants(
                        SwerveConstants.PATHPLANNER_ROTATIONAL_PID_KP,
                        SwerveConstants.PATHPLANNER_ROTATIONAL_PID_KI,
                        SwerveConstants.PATHPLANNER_ROTATIONAL_PID_KD
                    )
                ),
                config,
                () -> {
                    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this
            );
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        SmartDashboard.putBoolean("Swerve/Vision Enabled", false);
    }

    /**
     * Drives the robot with a specified robot relative velocity.
     * 
     * @param chassisSpeeds The translative and rotational velocities to the drive with.
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        swerveDrive.drive(chassisSpeeds);
    }

    /**
     * Drives the robot using field relative translative and angular magnitudes.
     * 
     * @param xMagnitude The velocity in the x direction, in terms of max linear velocity from [-1, 1].
     * @param yMagnitude The velocity in the y direction, in terms of max linear velocity from [-1, 1].
     * @param rotation   The rotational velocity, in terms of max rotational velocity from [-1, 1].
     */
    public void drive(double xMagnitude, double yMagnitude, double angularMagnitude) {
        double magnitude = Math.pow(
            Math.hypot(xMagnitude, yMagnitude),
            SwerveConstants.SMOOTHING_EXPONENT
        );

        double angle = Math.atan2(yMagnitude, xMagnitude);

        Translation2d translation = new Translation2d(
            Math.cos(angle) * magnitude * swerveDrive.getMaximumChassisVelocity(),
            Math.sin(angle) * magnitude * swerveDrive.getMaximumChassisVelocity()
        );

        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            translation = translation.rotateBy(Rotation2d.fromDegrees(180));
        }

        swerveDrive.drive(
            translation,
            angularMagnitude * swerveDrive.getMaximumChassisAngularVelocity(),
            true,
            false
        );
    }

    /**
     * Provides the pose of the drivetrain on the field from the odometry.
     * 
     * @return The odometry pose.
     */
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    /**
     * Provides the field relative robot velocity.
     * 
     * @return The field relative velocity.
     */
    public ChassisSpeeds getVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    /**
     * Provides the overall linear speed, of field relative robot velocity.
     * 
     * @return The field relative translational velocity in meters per second.
     */
    public LinearVelocity getTranslationalVelocity() {
        ChassisSpeeds velocities = swerveDrive.getFieldVelocity();
        return MetersPerSecond.of(
            Math.hypot(velocities.vxMetersPerSecond, velocities.vyMetersPerSecond)
        );
    }

    /**
     * Provides the rotational velocity of the chassis.
     * 
     * @return The rotational velocity.
     */
    public AngularVelocity getRotationalVelocity() {
        return RadiansPerSecond.of(swerveDrive.getRobotVelocity().omegaRadiansPerSecond);
    }

    /**
     * Points the wheels on all the swerve modules on the chassis inwards,
     * making the robot more difficult to push. 
     **/
    public void lock() {
        swerveDrive.lockPose();
    }

    /**
     * Provides a command to drive the robot using field relative translative values and heading as angular velocity.
     *
     * @param xAxis     The input axis that corresponds to movement along to the X axis.
     * @param yAxis     The input axis that corresponds to movement along the Y axis.
     * @param rotationAxis The input axis that corresponds to rotational movement.
     * @return Drive command.
     */
    public Command driveCommand(DoubleSupplier xAxis, DoubleSupplier yAxis, DoubleSupplier rotationAxis) {
        return run(() -> {
            drive(
                xAxis.getAsDouble(),
                yAxis.getAsDouble(),
                Math.pow(rotationAxis.getAsDouble(), SwerveConstants.SMOOTHING_EXPONENT)
            );
        });
    }

    /**
     * Provies a command that resets the gyro to the current heading for field relative driving purposes.
     * 
     * @return The command that zeros the gyro.
     */
    public Command zeroGyroCommand() {
        return runOnce(() -> {
            Optional<Alliance> alliance = DriverStation.getAlliance();
            Translation2d translation = getPose().getTranslation();
            Pose2d pose;
            
            if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                pose = new Pose2d(translation, Rotation2d.fromDegrees(180.0));
            } else {
                pose = new Pose2d(translation, Rotation2d.fromDegrees(0.0));
            }

            swerveDrive.resetOdometry(pose);
        });
    }

    @Override
    public void periodic() {
        if (!SmartDashboard.getBoolean("Swerve/Vision Enabled", true)) {
            return;
        }

        var estimates = visionSubsystem.getEstimates();

        for (var estimate: estimates) {
            swerveDrive.addVisionMeasurement(estimate.pose, estimate.timestamp, estimate.stdDevs);
        }
    }

    @Override
    public void simulationPeriodic() {
        visionSubsystem.updateSimulation(swerveDrive.getSimulationDriveTrainPose().get());
    }
}
