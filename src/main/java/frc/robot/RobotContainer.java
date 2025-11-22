// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.AlignLocations.BranchSide;
import frc.robot.commands.IntakeAuto;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@Logged
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  ElevatorSubsystem elevator = new ElevatorSubsystem();
  IntakeSubsystem intake = new IntakeSubsystem();
  ManipulatorSubsystem manipulator = new ManipulatorSubsystem();
  SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  AutoAlign alignmentCommandFactory = new AutoAlign(swerve);

  // Controllers
  private final CommandXboxController driverController =
      new CommandXboxController(Constants.kDriverControllerPort);
  private final CommandXboxController operatorController =
      new CommandXboxController(Constants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // All subsystems should default to not moving so they stop when buttons are released
    elevator.setDefaultCommand(Commands.run(() -> elevator.setElevatorVolts(0), elevator));
    manipulator.setDefaultCommand(Commands.run(() -> manipulator.setManipulatorVoltage(0), manipulator));
    intake.setDefaultCommand(Commands.run(() -> intake.setIntakeVolts(0), intake));

    
    // Swap controls based on alliance
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      swerve.setDefaultCommand(Commands.run(() -> swerve.drive(
        () -> MathUtil.applyDeadband(driverController.getLeftY(), .1),
        () -> MathUtil.applyDeadband(driverController.getLeftX(), .1),
        () -> -.9 * MathUtil.applyDeadband(driverController.getRightX(), .15)), swerve));
    } else {
          swerve.setDefaultCommand(Commands.run(() -> swerve.drive(
            () -> -MathUtil.applyDeadband(driverController.getLeftY(), .1),
            () -> -MathUtil.applyDeadband(driverController.getLeftX(), .1),
            () -> -.9 * MathUtil.applyDeadband(driverController.getRightX(), .15)), swerve));
    }

    driverController.a().onTrue(Commands.runOnce(() -> swerve.zeroGyro()));

    // Auto Align commands
    driverController.x().whileTrue(alignmentCommandFactory.generateCommand(BranchSide.LEFT));
    driverController.b().whileTrue(alignmentCommandFactory.generateCommand(BranchSide.RIGHT));
    driverController.y().whileTrue(alignmentCommandFactory.generateCommand(BranchSide.CENTER));

    // Manual elevator controls
    operatorController.rightBumper().whileTrue(Commands.run(() -> elevator.setElevatorVolts(2), elevator));
    operatorController.leftBumper().whileTrue(Commands.run(() -> elevator.setElevatorVolts(-2), elevator));

    // shoot controls
    operatorController.rightTrigger()
        .whileTrue(Commands.run(() -> manipulator.setManipulatorVoltage(4), manipulator));
    operatorController.rightTrigger().whileTrue(Commands.run(() -> intake.setIntakeVolts(-4.0), intake));

    // reverse shoot
    operatorController.leftTrigger()
        .whileTrue(Commands.run(() -> manipulator.setManipulatorVoltage(-4), manipulator));
    operatorController.leftTrigger().whileTrue(Commands.run(() -> intake.setIntakeVolts(4), intake));

    // Elevator setpoints
    operatorController.y().onTrue(Commands.runOnce(() -> elevator.setElevatorPosition(ElevatorConstants.kL3SETPOINT)));
    operatorController.y().whileTrue(Commands.run(() -> elevator.usePIDOutput(), elevator));

    operatorController.x().onTrue(Commands.runOnce(() -> elevator.setElevatorPosition(ElevatorConstants.kL2SETPOINT)));
    operatorController.x().whileTrue(Commands.run(() -> elevator.usePIDOutput(), elevator));

    operatorController.b().onTrue(Commands.runOnce(() -> elevator.setElevatorPosition(ElevatorConstants.kL4SETPOINT)));
    operatorController.b().whileTrue(Commands.run(() -> elevator.usePIDOutput(), elevator));

    operatorController.a().onTrue(Commands.runOnce(() -> elevator.setElevatorPosition(ElevatorConstants.kHOMESETPOINT)));
    operatorController.a().whileTrue(Commands.run(() -> elevator.usePIDOutput(), elevator));

    // Algae intake position
    operatorController.povLeft().onTrue(Commands.runOnce(() -> elevator.setElevatorPosition(1200.0)));
    operatorController.povLeft().whileTrue(Commands.run(() -> elevator.usePIDOutput(), elevator));

    // Auto intake command
    operatorController.povDown().onTrue(new IntakeAuto(manipulator, intake));

    // Fast shoot
    operatorController.povUp().whileTrue(Commands.run(() -> manipulator.setManipulatorVoltage(10), manipulator));

    // Driver controller score
    driverController.rightTrigger()
    .whileTrue(Commands.run(() -> manipulator.setManipulatorVoltage(4), manipulator));
    driverController.rightTrigger().whileTrue(Commands.run(() -> intake.setIntakeVolts(-4.0), intake));

    driverController.rightBumper().whileTrue(AutoBuilder.followPath(new PathPlannerPath(PathPlannerPath.waypointsFromPoses(swerve.getPose(), new Pose2d(8, 3, new Rotation2d())),
    new PathConstraints(3.0, 1.0, 540.0, 220.0),
            new IdealStartingState(MetersPerSecond.of(new Translation2d(swerve.getFieldRelativeSpeeds().vxMetersPerSecond, swerve.getFieldRelativeSpeeds().vyMetersPerSecond).getNorm()), swerve.getPose().getRotation()), 
            new GoalEndState(0.0, new Rotation2d()))));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return AutoBuilder.buildAuto("Start0-test");
  }
}
