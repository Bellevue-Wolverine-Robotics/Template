package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LEDConstants;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.epilogue.Logged;

public class LEDSubsystem extends SubsystemBase {
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;

    @Logged(name="Aligning")
    private boolean aligning = false;

    public LEDSubsystem() {
        led = new AddressableLED(LEDConstants.PWM_PORT);
        ledBuffer = new AddressableLEDBuffer(LEDConstants.LENGTH);

        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();
    }

    public void setAligning(boolean aligning) {
        this.aligning = aligning;
    }

    private void setPattern(LEDPattern pattern) {
        pattern.applyTo(ledBuffer);
        led.setData(ledBuffer);
    }

    private void setBlueYellow() {
        LEDPattern gradient = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kYellow, Color.kBlue);
        gradient.applyTo(ledBuffer);
        led.setData(ledBuffer);
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            setBlueYellow();
            return;
        }

        LEDPattern basePattern = LEDPattern.solid(Color.kGreen);

        if (aligning) {
            setPattern(basePattern.blink(Seconds.of(0.5)));
        } else {
            setPattern(basePattern);
        }
    }
}
