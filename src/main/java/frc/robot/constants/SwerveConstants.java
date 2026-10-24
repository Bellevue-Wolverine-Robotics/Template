package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.pathplanner.lib.config.PIDConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

public class SwerveConstants {
    private static final LinearVelocity VELOCITY_AT_12_VOLTS = MetersPerSecond.of(4.54);

    public static final LinearVelocity MAXIMUM_TRANSLATIONAL_VELOCITY = VELOCITY_AT_12_VOLTS;
    public static final AngularVelocity MAXIMUM_ANGULAR_VELOCITY = RotationsPerSecond.of(1.0);

    public static final Matrix<N3, N1> STATE_STD_DEVS = VecBuilder.fill(0.1, 0.1, 0.1);

    public static final PIDConstants PATHPLANNER_TRANSLATIONAL_PID = new PIDConstants(10.0, 0.0, 0.0);
    public static final PIDConstants PATHPLANNER_ROTATIONAL_PID = new PIDConstants(7.0, 0.0, 0.0);

    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs STEER_GAINS = new Slot0Configs()
        .withKP(100.0).withKI(0.0).withKD(0.5)
        .withKS(0.1).withKV(1.91).withKA(0.0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs DRIVE_GAINS = new Slot0Configs()
        .withKP(0.1).withKI(0.0).withKD(0.0)
        .withKS(0.0).withKV(0.124);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;

    // The type of motor used for the drive motor
    private static final DriveMotorArrangement DRIVE_MOTOR_TYPE = DriveMotorArrangement.TalonFX_Integrated;
    // The type of motor used for the steer motor
    private static final SteerMotorArrangement STEER_MOTOR_TYPE = SteerMotorArrangement.TalonFX_Integrated;

    // The remote sensor feedback type to use for the steer motors;
    // When not Pro-licensed, Fused*/Sync* automatically fall back to Remote*
    private static final SteerFeedbackType STEER_FEEDBACK_TYPE = SteerFeedbackType.FusedCANcoder;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final Current SLIP_CURRENT = Amps.of(120.0);

    // Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
    // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
    private static final TalonFXConfiguration DRIVE_INITIAL_CONFIGS = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                // Default supply current limit is 70 A, but it can be lowered to avoid brownouts.
                // Supply current limits can be larger than the breaker current rating.
                .withSupplyCurrentLimit(Amps.of(70.0))
                .withSupplyCurrentLimitEnable(true)
        );
    private static final TalonFXConfiguration STEER_INITIAL_CONFIGS = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                // Swerve azimuth does not require much torque output, so we can set a relatively low
                // stator current limit to help avoid brownouts without impacting performance.
                .withStatorCurrentLimit(Amps.of(60.0))
                .withStatorCurrentLimitEnable(true)
        );
    private static final CANcoderConfiguration ENCODER_INITIAL_CONFIGS = new CANcoderConfiguration();
    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    private static final Pigeon2Configuration PIGEON_CONFIGS = null;

    // CAN bus that the devices are located on;
    // All swerve devices must share the same CAN bus
    public static final CANBus kCANBus = new CANBus("canivore", "./logs/example.hoot");

    // Every 1 rotation of the azimuth results in COUPLE_RATIO drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double COUPLE_RATIO = 3.8181818181818183;

    private static final double DRIVE_GEAR_RATIO = 7.363636363636365;
    private static final double STEER_GEAR_RATIO = 15.42857142857143;
    private static final Distance WHEEL_RADIUS = Inches.of(2.167);

    private static final boolean INVERT_LEFT_SIDE = false;
    private static final boolean INVERT_RIGHT_SIDE = true;

    private static final int PIGEON_ID = 1;

    // These are only used for simulation
    private static final MomentOfInertia STEER_INERTIA = KilogramSquareMeters.of(0.01);
    private static final MomentOfInertia DRIVE_INERTIA = KilogramSquareMeters.of(0.035);
    // Simulated voltage necessary to overcome friction
    private static final Voltage STEER_FRICTION_VOLTAGE = Volts.of(0.2);
    private static final Voltage DRIVE_FRICTION_VOLTAGE = Volts.of(0.2);

    public static final SwerveDrivetrainConstants DRIVETRAIN_CONSTANTS = new SwerveDrivetrainConstants()
            .withCANBusName(kCANBus.getName())
            .withPigeon2Id(PIGEON_ID)
            .withPigeon2Configs(PIGEON_CONFIGS);

    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> CONSTANT_CREATOR =
        new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
            .withSteerMotorGearRatio(STEER_GEAR_RATIO)
            .withCouplingGearRatio(COUPLE_RATIO)
            .withWheelRadius(WHEEL_RADIUS)
            .withSteerMotorGains(STEER_GAINS)
            .withDriveMotorGains(DRIVE_GAINS)
            .withSteerMotorClosedLoopOutput(STEER_CLOSED_LOOP_OUTPUT)
            .withDriveMotorClosedLoopOutput(DRIVE_CLOSED_LOOP_OUTPUT)
            .withSlipCurrent(SLIP_CURRENT)
            .withSpeedAt12Volts(VELOCITY_AT_12_VOLTS)
            .withDriveMotorType(DRIVE_MOTOR_TYPE)
            .withSteerMotorType(STEER_MOTOR_TYPE)
            .withFeedbackSource(STEER_FEEDBACK_TYPE)
            .withDriveMotorInitialConfigs(DRIVE_INITIAL_CONFIGS)
            .withSteerMotorInitialConfigs(STEER_INITIAL_CONFIGS)
            .withEncoderInitialConfigs(ENCODER_INITIAL_CONFIGS)
            .withSteerInertia(STEER_INERTIA)
            .withDriveInertia(DRIVE_INERTIA)
            .withSteerFrictionVoltage(STEER_FRICTION_VOLTAGE)
            .withDriveFrictionVoltage(DRIVE_FRICTION_VOLTAGE);


    // Front Left
    private static final int FRONT_LEFT_DRIVE_MOTOR_ID = 3;
    private static final int FRONT_LEFT_STEER_MOTOR_ID = 2;
    private static final int FRONT_LEFT_ENCODER_ID = 1;
    private static final Angle FRONT_LEFT_ENCODER_OFFSET = Rotations.of(0.15234375);
    private static final boolean FRONT_LEFT_STEER_MOTOR_INVERTED = true;
    private static final boolean FRONT_LEFT_ENCODER_INVERTED = false;

    private static final Distance FRONT_LEFT_X_POS = Inches.of(10.0);
    private static final Distance FRONT_LEFT_Y_POS = Inches.of(10.0);

    // Front Right
    private static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 1;
    private static final int FRONT_RIGHT_STEER_MOTOR_ID = 0;
    private static final int FRONT_RIGHT_ENCODER_ID = 0;
    private static final Angle FRONT_RIGHT_ENCODER_OFFSET = Rotations.of(-0.4873046875);
    private static final boolean FRONT_RIGHT_STEER_MOTOR_INVERTED = true;
    private static final boolean FRONT_RIGHT_ENCODER_INVERTED = false;

    private static final Distance FRONT_RIGHT_X_POS = Inches.of(10.0);
    private static final Distance FRONT_RIGHT_Y_POS = Inches.of(-10.0);

    // Back Left
    private static final int BACK_LEFT_DRIVE_MOTOR_ID = 7;
    private static final int BACK_LEFT_STEER_MOTOR_ID = 6;
    private static final int BACK_LEFT_ENCODER_ID = 3;
    private static final Angle BACK_LEFT_ENCODER_OFFSET = Rotations.of(-0.219482421875);
    private static final boolean BACK_LEFT_STEER_MOTOR_INVERTED = true;
    private static final boolean BACK_LEFT_ENCODER_INVERTED = false;

    private static final Distance BACK_LEFT_X_POS = Inches.of(-10.0);
    private static final Distance BACK_LEFT_Y_POS = Inches.of(10.0);

    // Back Right
    private static final int BACK_RIGHT_DRIVE_MOTOR_ID = 5;
    private static final int BACK_RIGHT_STEER_MOTOR_ID = 4;
    private static final int BACK_RIGHT_ENCODER_ID = 2;
    private static final Angle BACK_RIGHT_ENCODER_OFFSET = Rotations.of(0.17236328125);
    private static final boolean BACK_RIGHT_STEER_MOTOR_INVERTED = true;
    private static final boolean BACK_RIGHT_ENCODER_INVERTED = false;

    private static final Distance BACK_RIGHT_X_POS = Inches.of(-10.0);
    private static final Distance BACK_RIGHT_Y_POS = Inches.of(-10.0);


    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FRONT_LEFT_MODULE =
        CONSTANT_CREATOR.createModuleConstants(
            FRONT_LEFT_STEER_MOTOR_ID, FRONT_LEFT_DRIVE_MOTOR_ID, FRONT_LEFT_ENCODER_ID, FRONT_LEFT_ENCODER_OFFSET,
            FRONT_LEFT_X_POS, FRONT_LEFT_Y_POS, INVERT_LEFT_SIDE, FRONT_LEFT_STEER_MOTOR_INVERTED, FRONT_LEFT_ENCODER_INVERTED
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FRONT_RIGHT_MODULE =
        CONSTANT_CREATOR.createModuleConstants(
            FRONT_RIGHT_STEER_MOTOR_ID, FRONT_RIGHT_DRIVE_MOTOR_ID, FRONT_RIGHT_ENCODER_ID, FRONT_RIGHT_ENCODER_OFFSET,
            FRONT_RIGHT_X_POS, FRONT_RIGHT_Y_POS, INVERT_RIGHT_SIDE, FRONT_RIGHT_STEER_MOTOR_INVERTED, FRONT_RIGHT_ENCODER_INVERTED
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BACK_LEFT_MODULE =
        CONSTANT_CREATOR.createModuleConstants(
            BACK_LEFT_STEER_MOTOR_ID, BACK_LEFT_DRIVE_MOTOR_ID, BACK_LEFT_ENCODER_ID, BACK_LEFT_ENCODER_OFFSET,
            BACK_LEFT_X_POS, BACK_LEFT_Y_POS, INVERT_LEFT_SIDE, BACK_LEFT_STEER_MOTOR_INVERTED, BACK_LEFT_ENCODER_INVERTED
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BACK_RIGHT_MODULE =
        CONSTANT_CREATOR.createModuleConstants(
            BACK_RIGHT_STEER_MOTOR_ID, BACK_RIGHT_DRIVE_MOTOR_ID, BACK_RIGHT_ENCODER_ID, BACK_RIGHT_ENCODER_OFFSET,
            BACK_RIGHT_X_POS, BACK_RIGHT_Y_POS, INVERT_RIGHT_SIDE, BACK_RIGHT_STEER_MOTOR_INVERTED, BACK_RIGHT_ENCODER_INVERTED
        );
}
