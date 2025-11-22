package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class IntakeSubsystem extends SubsystemBase {
    LaserCan funnelSensor;
    SparkMax roller;
    SparkMaxConfig rollerConfig = new SparkMaxConfig();

    public IntakeSubsystem() {
        funnelSensor = new LaserCan(IntakeConstants.kFunnelSensorID);
        roller = new SparkMax(IntakeConstants.kRollerID, MotorType.kBrushless);
        rollerConfig.inverted(true).idleMode(IdleMode.kCoast);
        roller.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public double getSensorMeasurementFunnel() {
        // null check
        var sensorMeasurementFunnel = funnelSensor.getMeasurement();
        if (sensorMeasurementFunnel != null) {
            return sensorMeasurementFunnel.distance_mm;
        }
        return 0;
    }

    public void setIntakeVolts(double volts) {
        roller.setVoltage(volts);
    }
    // returns true if funnel sensor senses coral
    public boolean isFunnelTripped() {
        return getSensorMeasurementFunnel() < IntakeConstants.kSensorTolerance;
    }
}
