// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.constants.DriverStationConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
    @Logged(name = "LED Subsystem")
    private final LEDSubsystem ledSubsystem = new LEDSubsystem();

    @Logged(name = "Vision Subsystem")
    private final VisionSubsystem visionSubsystem = new VisionSubsystem();

    @Logged(name = "Swerve Subsystem")
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(ledSubsystem, visionSubsystem);

    private final CommandXboxController driverController = new CommandXboxController(DriverStationConstants.DRIVER_CONTROLLER_PORT);

    private final SendableChooser<Command> sendableChooser = new SendableChooser<>();

    public RobotContainer() {
        configureBindings();
        configureAutonomous();
    }

    private void configureBindings() {
        swerveSubsystem.setDefaultCommand(swerveSubsystem.driveCommand(
            () -> -MathUtil.applyDeadband(driverController.getLeftY(), DriverStationConstants.DRIVER_CONTROLLER_LEFT_DEADBAND),
            () -> -MathUtil.applyDeadband(driverController.getLeftX(), DriverStationConstants.DRIVER_CONTROLLER_LEFT_DEADBAND),
            () -> -MathUtil.applyDeadband(driverController.getRightX(), DriverStationConstants.DRIVER_CONTROLLER_RIGHT_DEADBAND)
        ));

        driverController.start().onTrue(swerveSubsystem.zeroGyroCommand());
        driverController.a().whileTrue(swerveSubsystem.alignCommand(VisionConstants.OUTPOST_POSE_SUPPLIER));
        driverController.rightTrigger().whileTrue(swerveSubsystem.aimCommand(
            VisionConstants.HUB_POSE_SUPPLIER,
            () -> -MathUtil.applyDeadband(driverController.getLeftY(), DriverStationConstants.DRIVER_CONTROLLER_LEFT_DEADBAND),
            () -> -MathUtil.applyDeadband(driverController.getLeftX(), DriverStationConstants.DRIVER_CONTROLLER_LEFT_DEADBAND)
        ));
    }

    private void configureAutonomous() {
        sendableChooser.setDefaultOption("Default", new PathPlannerAuto("DEFAULT"));
        SmartDashboard.putData("Auto Chooser", sendableChooser);
    }

    public Command getAutonomousCommand() {
        return sendableChooser.getSelected();
    }
}
