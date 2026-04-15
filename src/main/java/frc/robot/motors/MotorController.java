package frc.robot.motors;

import edu.wpi.first.units.measure.*;

public interface MotorController {
    /** Represents a gravity type for a mechanism with feedforward. */
    enum GravityType {
        /** Applies the gravity feedforward as a constant force. */
        Elevator,
        /** Applies the gravity feedforward proportional to the cosine of a mechanism's angle. */
        Arm
    }

    /**
     * Provides the CAN ID of the motor controller.
     * 
     * @return The device's CAN ID.
     */
    int getId();

    /**
     * Configures the PId contorller for closed loop contorl.
     * 
     * @param kP The proportional constant.
     * @param kI The integral constant.
     * @param kD The derivative constant.
    */
    void configurePid(double kP, double kI, double kD);

    /**
     * Configures the feedforward controller for closed loop control.
     * 
     * @param kS The volts needed to overcome static friction.
     * @param kV The volts needed per rotation per second.
     * @param kA The volts needed per rotation per second per second.
     * @param kG The volts needed to overcome gravity.
     * @param gravityType The type of gravity to apply for the system.
     */
    void configureFeedForward(double kS, double kV, double kA, double kG, GravityType gravityType);

    /** Configures the  feedforward controller for closed loop control.
     *
     * @param kS The volts needed to overcome static friction.
     * @param kV The volts needed per rotation per second.
     * @param kA The volts needed per rotation per rotation per second.
     */
    void configureFeedForward(double kS, double kV, double kA);

    /**
     * Sets the conversion factor for mechanism rotations, relative to the shaft's rotations.
     * 
     * @param factor The factor to apply to shaft rotations.
     */
    void setConversionFactor(double factor);

    /**
     * Sets the current limit on the motor.
     * Requires {@link #apply()} to be called for the changes to take effect.
     * 
     * @param current The maximum allowable current.
     */
    void setCurrentLimit(Current current);

    /**
     * Sets the idle breaking behavior of the motor.
     * Requires {@link #apply()} to be called for the changes to take effect.
     * 
     * @param brake Whether to use brake mode. Otherwise, coast mode is used.
     */
    void setBrakeMode(boolean brake);

    /** Whether to invert the motor's direction. An inverted motor's shaft rotates clockwise.
     * Requires {@link #apply()} to be called for the changes to take effect.
     * 
     * @param inverted Whether to invert the motor.
    */
    void setInverted(boolean inverted);

    /**
     * Applies all configuration changes the motor controller.
     */
    void apply();

    /**
     * Sets the motor's output to match a leader motor's output.
     * For {@link TalonFXMotorController}, the output immediately follows the leader,
     * while {@link SparkMaxMotorController} requires {@link #apply()} to be explicitly called.
     * <p>The leader must be of the same type as the follower.
     * For example, a {@link TalonFXMotorController} cannot follow a {@link SparkMaxMotorController}.
     * <p><strong>Warning:</strong> Many motor controllers only follow voltage, so it is recommended to
     * explictly configure coast mode on both the leader and follower.
     *
     * @param motor The leading motor controller to follow.
     * @param inverted Whether to invert relative to the leader motor controller.
     */
    void follow(MotorController motor, boolean inverted);

    /**
     * Sets the output of the motor as a percentage of maximum voltage.
     * 
     * @param percentage The percentage of maximum available voltage to use, from -1 to 1.
     */
    void setDutyCycle(Dimensionless percentage);

    /**
     * Sets the output of the motor in terms of voltage.
     * 
     * @param voltage The output voltage.
     */
    void setVoltage(Voltage voltage);

    /**
     * Sets the output of the motor in terms of current.
     * <p>For {@link SparkMaxMotorController}, the current is rounded to the nearest whole number of amps.
     * 
     * @param current The output current.
     */
    void setCurrent(Current current);

    /**
     * Sets the target rotational velocity for closed loop control.
     * 
     * @param velocity The target velocity.
     */
    void setVelocity(AngularVelocity velocity);

    /**
     * Sets the target rotational position for closed loop control,
     * relative to the intial position of the encoder.
     * 
     * @param position The relative target position.
     */
    void setPosition(Angle position);

    /**
     * Stops all output to the motor.
     */
    void stop();

    /**
     * Gives the position reading from the encoder,
     * relative to the initial position of the encoder.
     * 
     * @return The relative position reading.
     */
    Angle getPosition();

    /**
     * Gives the velocity reading from the encoder.
     * 
     * @return The velocity reading.
     */
    AngularVelocity getVelocity();

    /**
     * Gives the duty cycle voltage output of the motor controller.
     * 
     * @return The duty cycle output, from -1 to 1.
     */
    Dimensionless getOutput();

    /**
     * Gives the current output of the motor controller in amps.
     * 
     * @return The current output.
     */
    Current getOutputCurrent();

    /**
     * Gives the available voltage being supplied the motor controller from the battery.
     * 
     * @return The available voltage.
     */
    Voltage getBusVoltage();

    /**
     * Gives the temperature reading of the motor.
     * 
     * @return The motor's temperature.
     */
    Temperature getTemperature();

    /**
     * Resets the encoder reading to zero.
     */
    void resetEncoder();

    /**
     * Sets the encoder reading to a specific position.
     * 
     * @param position The position.
     */
    void resetEncoder(Angle position);
}
