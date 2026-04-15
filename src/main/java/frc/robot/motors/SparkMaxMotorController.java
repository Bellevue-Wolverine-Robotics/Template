package frc.robot.motors;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

public class SparkMaxMotorController implements MotorController {
    private final SparkMax motor;
    private final SparkMaxConfig config = new SparkMaxConfig();
    private final SparkClosedLoopController controller;

    public SparkMaxMotorController(int id) {
        motor = new SparkMax(id, MotorType.kBrushless);
        controller = motor.getClosedLoopController();
    }

    @Override
    public int getId() {
        return motor.getDeviceId();
    }

    @Override
    public void follow(MotorController motor, boolean inverted) {
        config.follow(motor.getId(), inverted);
    }

    @Override
    public void configurePid(double kP, double kI, double kD) {
        config.closedLoop.p(kP).i(kI).d(kD);
    }

    @Override
    public void configureFeedForward(double kS, double kV, double kA, double kG, GravityType gravityType) {

        if (gravityType == GravityType.Arm) {
            config.closedLoop.feedForward.kS(kS).kV(kV).kA(kA).kCos(kG);
        } else {
            config.closedLoop.feedForward.kS(kS).kV(kV).kA(kA).kG(kG);
        }
    }

    @Override
    public void configureFeedForward(double kS, double kV, double kA) {
        config.closedLoop.feedForward.kS(kS).kV(kV).kA(kA);
    }

    @Override
    public void setConversionFactor(double factor) {
        config.encoder.positionConversionFactor(factor);
        config.encoder.velocityConversionFactor(factor);
    }

    @Override
    public void setCurrentLimit(Current current) {
        config.smartCurrentLimit((int) Math.round(current.in(Amps)));
    }

    @Override
    public void setBrakeMode(boolean brake) {
        config.idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void setInverted(boolean inverted) {
        config.inverted(inverted);
    }

    @Override
    public void apply() {
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setDutyCycle(Dimensionless percentage) {
        controller.setSetpoint(percentage.in(Value), ControlType.kDutyCycle);
    }

    @Override
    public void setVoltage(Voltage voltage) {
        controller.setSetpoint(voltage.in(Volts), ControlType.kVoltage);
    }

    @Override
    public void setCurrent(Current current) {
        controller.setSetpoint(current.in(Amps), ControlType.kCurrent);
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        controller.setSetpoint(velocity.in(RPM), ControlType.kVelocity);
    }

    @Override
    public void setPosition(Angle position) {
        controller.setSetpoint(position.in(Rotations), ControlType.kPosition);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public Angle getPosition() {
        return Rotations.of(motor.getEncoder().getPosition());
    }

    @Override
    public AngularVelocity getVelocity() {
        return RPM.of(motor.getEncoder().getVelocity());
    }

    @Override
    public Dimensionless getOutput() {
        return Value.of(motor.getAppliedOutput());
    }

    @Override
    public Current getOutputCurrent() {
        return Amps.of(motor.getOutputCurrent());
    }

    @Override
    public Voltage getBusVoltage() {
        return Volts.of(motor.getBusVoltage());
    }

    @Override
    public Temperature getTemperature() {
        return Celsius.of(motor.getMotorTemperature());
    }

    @Override
    public void resetEncoder() {
        motor.getEncoder().setPosition(0.0);
    }

    @Override
    public void resetEncoder(Angle position) {
        motor.getEncoder().setPosition(position.in(Rotations));
    }
}
