package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AimingConstants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import static edu.wpi.first.units.Units.*;

public class AimPoseCommand extends Command {
    private final PIDController thetaController = new PIDController(
        AimingConstants.ROTATIONAL_PID_KP,
        AimingConstants.ROTATIONAL_PID_KI,
        AimingConstants.ROTATIONAL_PID_KD
    );

    private final SwerveSubsystem swerveSubsystem;
    private final LEDSubsystem ledSubsystem;
    private final Supplier<Pose2d> target;
    private final DoubleSupplier xAxis;
    private final DoubleSupplier yAxis;

    /**
     * Constructs a new AimPoseCommand.
     * 
     * @param swerveSubsystem The swerve subsystem.
     * @param ledSubsystem The LED subsystem.
     * @param target The target to point.
     * @param xAxis Supplier of the X axis value on the driving joystick.
     * @param yAxis Supplier of the Y axis value on the driving joystick.
     */
    public AimPoseCommand(
        SwerveSubsystem swerveSubsystem,
        LEDSubsystem ledSubsystem,
        Supplier<Pose2d> target,
        DoubleSupplier xAxis,
        DoubleSupplier yAxis
    ) {
        this.swerveSubsystem = swerveSubsystem;
        this.ledSubsystem = ledSubsystem;
        this.target = target;
        this.xAxis = xAxis;
        this.yAxis = yAxis;
        addRequirements(swerveSubsystem);

        thetaController.setTolerance(AimingConstants.ROTATIONAL_TOLERANCE.in(Radians));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        thetaController.reset();
    }

    @Override
    public void execute() {
        Pose2d pose = swerveSubsystem.getPose();

        Translation2d difference = target.get().getTranslation().minus(pose.getTranslation());

        double currentHeading = pose.getRotation().getRadians();
        double desiredHeading = difference.getAngle().getRadians();
        double thetaVelocity = thetaController.calculate(currentHeading, desiredHeading);

        if (thetaController.atSetpoint() && xAxis.getAsDouble() == 0 && yAxis.getAsDouble() == 0) {
            swerveSubsystem.lock();
        } else {
            swerveSubsystem.drive(
                xAxis.getAsDouble(),
                yAxis.getAsDouble(),
                thetaVelocity
            );
        }

        ledSubsystem.setAligning(!thetaController.atSetpoint());
    }

    @Override
    public void end(boolean interrupted) {
        ledSubsystem.setAligning(false);
    }
}
