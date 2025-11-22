package frc.robot.subsystems.Manipulator;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class ManipulatorSubsystem extends SubsystemBase{
    LaserCan manipulatorSensor;
    SparkMax manipulator;
    SparkMaxConfig manipulatorConfig = new SparkMaxConfig();

    public ManipulatorSubsystem() {
        manipulatorSensor = new LaserCan(ManipulatorConstants.kManipulatorSensorID);
        manipulatorConfig.inverted(false).idleMode(IdleMode.kBrake);
        manipulator = new SparkMax(ManipulatorConstants.kManipulatorID, MotorType.kBrushless);
        manipulator.configure(manipulatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public double getSensorMeasurementManipulator() {
        // null check
        var sensorMeasurementManipulator = manipulatorSensor.getMeasurement();
        if (sensorMeasurementManipulator != null) {
            return sensorMeasurementManipulator.distance_mm;
        }
        return 0;
    }

    public void setManipulatorVoltage(double volts) {
        manipulator.setVoltage(volts);
    }

    public boolean isManipulatorTripped() {
        return getSensorMeasurementManipulator() < ManipulatorConstants.kManipulatorID;
    }
}