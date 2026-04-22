package frc.robot.motors;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

public class TalonFXMotorController implements MotorController {
    private final TalonFX motor;
    private final TalonFXConfiguration config = new TalonFXConfiguration();

    public TalonFXMotorController(int id, String bus) {
        motor = new TalonFX(id, new CANBus(bus));
    }

    @Override
    public int getId() {
        return motor.getDeviceID();
    }

    @Override
    public void follow(MotorController motor, boolean inverted) {
        this.motor.setControl(new Follower(
            motor.getId(),
            inverted ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned
        ));
    }

    @Override
    public void configurePid(double kP, double kI, double kD) {
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
    }

    @Override
    public void configureFeedForward(double kS, double kV, double kA, double kG, GravityType gravityType) {
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;
        config.Slot0.kG = kG;

        if (gravityType == GravityType.Arm) {
            config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        } else if (gravityType == GravityType.Elevator) {
            config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        }
    }

    @Override
    public void configureFeedForward(double kS, double kV, double kA) {
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;
    }

    @Override
    public void setConversionFactor(double factor) {
        config.Feedback.SensorToMechanismRatio = 1.0 / factor;
    }

    @Override
    public void setCurrentLimit(Current current) {
        config.CurrentLimits.SupplyCurrentLimit = current.in(Amps);
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
    }

    @Override
    public void setBrakeMode(boolean brake) {
        config.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    }

    @Override
    public void setInverted(boolean inverted) {
        config.MotorOutput.Inverted = inverted ?
            InvertedValue.Clockwise_Positive :
            InvertedValue.CounterClockwise_Positive;
    }

    @Override
    public void apply() {
        motor.getConfigurator().apply(config);
    }

    @Override
    public void setDutyCycle(Dimensionless percentage) {
        motor.setControl(new DutyCycleOut(percentage.in(Value)));
    }

    @Override
    public void setVoltage(Voltage voltage) {
        motor.setControl(new VoltageOut(voltage));
    }

    @Override
    public void setCurrent(Current current) {
        motor.setControl(new TorqueCurrentFOC(current));
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        motor.setControl(new VelocityVoltage(velocity));
    }

    @Override
    public void setPosition(Angle position) {
        motor.setControl(new PositionVoltage(position));
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public Angle getPosition() {
        return motor.getPosition().getValue();
    }

    @Override
    public AngularVelocity getVelocity() {
        return motor.getVelocity().getValue();
    }

    @Override
    public Dimensionless getOutput() {
        return Value.of(motor.getDutyCycle().getValue());
    }

    @Override
    public Current getOutputCurrent() {
        return motor.getStatorCurrent().getValue();
    }

    @Override
    public Voltage getBusVoltage() {
        return motor.getSupplyVoltage().getValue();
    }

    @Override
    public Temperature getTemperature() {
        return motor.getDeviceTemp().getValue();
    }

    @Override
    public void resetEncoder() {
        motor.setPosition(0.0);
    }

    @Override
    public void resetEncoder(Angle position) {
        motor.setPosition(position); 
    }
}
