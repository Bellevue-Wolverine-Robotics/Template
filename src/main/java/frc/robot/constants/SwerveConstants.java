package frc.robot.constants;

import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;

import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

public class SwerveConstants {
    public static final SimulatedArena SIMULATED_ARENA = new Arena2026Rebuilt(false);
    public static final LinearVelocity MAXIMUM_VELOCITY = MetersPerSecond.of(5.0);
    public static final double SMOOTHING_EXPONENT = 1.0;

    public static final double PATHPLANNER_TRANSLATIONAL_PID_KP = 5.0;
    public static final double PATHPLANNER_TRANSLATIONAL_PID_KI = 0.0;
    public static final double PATHPLANNER_TRANSLATIONAL_PID_KD = 0.0;
    public static final double PATHPLANNER_ROTATIONAL_PID_KP = 5.0;
    public static final double PATHPLANNER_ROTATIONAL_PID_KI = 0.0;
    public static final double PATHPLANNER_ROTATIONAL_PID_KD = 0.0;
}
