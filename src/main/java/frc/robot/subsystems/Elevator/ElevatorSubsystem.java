package frc.robot.subsystems.Elevator;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class ElevatorSubsystem extends SubsystemBase {
    SparkMax elevator1;
    SparkMax elevator2;
    ProfiledPIDController controller;
    Encoder encoder;
    SparkMaxConfig elevator1Config = new SparkMaxConfig();
    SparkMaxConfig elevator2Config = new SparkMaxConfig();
    ElevatorFeedforward feedforward = new ElevatorFeedforward(0, ElevatorConstants.kG, 0);
    double targetPosition;
    
    public ElevatorSubsystem() {
        elevator1 = new SparkMax(ElevatorConstants.kElevator1ID, MotorType.kBrushless);
        elevator2 = new SparkMax(ElevatorConstants.kElevator2ID, MotorType.kBrushless);

        elevator1Config.inverted(true).idleMode(IdleMode.kBrake);
        elevator2Config.inverted(false).idleMode(IdleMode.kBrake);

        elevator1.configure(elevator1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevator2.configure(elevator2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        controller = new ProfiledPIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, new Constraints(ElevatorConstants.kMaxVelocity, ElevatorConstants.kMaxAcceleration));

        encoder = new Encoder(ElevatorConstants.kChannelA, ElevatorConstants.kChannelB, false);
        encoder.reset();

        targetPosition = ElevatorConstants.kHOMESETPOINT;

        controller.setGoal(targetPosition);
        controller.setTolerance(ElevatorConstants.kPositionTolerance);
    }

    @Logged(name = "Position")
    public double getElevatorPosition() {
        return encoder.getDistance();
    }

    public double getElevatorRate() {
        return encoder.getRate();
    }

    // Manually sets power of elevator motors and sets the setpoint of the 
    // controller to the current position so motion profiles don't break.
    public void setElevatorVolts(double volts) {
        elevator1.setVoltage(volts);
        elevator2.setVoltage(volts);
        controller.getSetpoint().position = getElevatorPosition();
    }

    // Sets goal position of elevator
    public void setElevatorPosition(double position) {
        targetPosition = position;
        controller.setGoal(targetPosition);
    }

    public double getElevatorGoal() {
        return controller.getGoal().position;
    }

    // Sets motor outputs to move elevator to target position
    public void usePIDOutput() {
        elevator1.setVoltage(controller.calculate(getElevatorPosition()) + feedforward.calculate(0));
        elevator2.setVoltage(controller.calculate(getElevatorPosition()) + feedforward.calculate(0));
    }

    // Returns true if the elevator is at the correct position
    public boolean atGoal() {
        return controller.atGoal();
    }
}
