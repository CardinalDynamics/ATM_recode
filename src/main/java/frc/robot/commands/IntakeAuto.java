package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Manipulator.ManipulatorSubsystem;


public class IntakeAuto extends Command {
    ManipulatorSubsystem manipulator;
    IntakeSubsystem intake;
    boolean passed = false;
    
    public IntakeAuto(ManipulatorSubsystem manipulator, IntakeSubsystem intake) {
        this.manipulator = manipulator;
        this.intake = intake;
        addRequirements(manipulator, intake);
    }

    public void initialize() {
        passed = false;
        intake.setIntakeVolts(-10);
        manipulator.setManipulatorVoltage(4);
    }

    public void execute() {
        if (intake.isFunnelTripped()) {
            passed = true;
            intake.setIntakeVolts(-8);
        }
        if (passed && !intake.isFunnelTripped()) {
            intake.setIntakeVolts(0);
            manipulator.setManipulatorVoltage(2);
        }
    }

    public void end() {
        intake.setIntakeVolts(0);
        manipulator.setManipulatorVoltage(0);
        passed = false;
    }

    public boolean isFinished() {
        return manipulator.isManipulatorTripped();
    }
}
