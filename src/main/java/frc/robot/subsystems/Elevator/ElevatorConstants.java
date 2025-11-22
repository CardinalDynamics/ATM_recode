package frc.robot.subsystems.Elevator;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;

public final class ElevatorConstants {

    // Elevator CAN IDs
    public static final int kElevator1ID = 13;
    public static final int kElevator2ID = 12;

    // Encoder ports
    public static final int kChannelA = 0;
    public static final int kChannelB = 1;

    // Setpoints
    public static final double kHOMESETPOINT = 0;
    public static final double kL2SETPOINT = 2475.0;
    public static final double kL3SETPOINT = 5250.0;
    public static final double kL4SETPOINT = 9600.0;

    // Controler values
    public static final double kP = 0.019;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kG = 0.15;
    public static final double kMaxVelocity = 40000.0;
    public static final double kMaxAcceleration = 80000.0;
    public static final double kPositionTolerance = 30.0;

    // Elevator ff
    public static final ElevatorFeedforward elevatorFeedforward
        = new ElevatorFeedforward(0, 0.15, 0);
}
