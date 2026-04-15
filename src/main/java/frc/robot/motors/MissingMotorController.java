package frc.robot.motors;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

public class MissingMotorController implements MotorController {
    public int getId() { return -1; }
    public void configurePid(double kP, double kI, double kD) {}
    public void configureFeedForward(double kS, double kV, double kA, double kG, GravityType gravityType) {}
    public void configureFeedForward(double kS, double kV, double kA) {}
    public void setConversionFactor(double factor) {}
    public void setCurrentLimit(Current current) {}
    public void setBrakeMode(boolean brake) {}
    public void setInverted(boolean inverted) {}
    public void apply() {}
    public void follow(MotorController motor, boolean inverted) {}
    public void setDutyCycle(Dimensionless percentage) {}
    public void setVoltage(Voltage voltage) {}
    public void setCurrent(Current current) {}
    public void setVelocity(AngularVelocity velocity) {}
    public void setPosition(Angle position) {}
    public void stop() {}
    public Angle getPosition() { return Rotations.of(0); }
    public AngularVelocity getVelocity() { return RotationsPerSecond.of(0); }
    public Dimensionless getOutput() { return Value.of(0); }
    public Current getOutputCurrent() { return Amps.of(0); }
    public Voltage getBusVoltage() { return Volts.of(0); }
    public Temperature getTemperature() { return Celsius.of(0); }
    public void resetEncoder() {}
    public void resetEncoder(Angle position) {}
}
