/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakePivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

// spark implementation of real mechanism

@Logged
public class AlgaeIntakePivotIOSpark implements AlgaeIntakePivotIO {

  public static final AlgaeIntakePivotConfig config = new AlgaeIntakePivotConfig(0.34, 0, 0, 0.21);

  // device ids are plcaeholders
  private SparkMax pivotMotorLeft =
      new SparkMax(
          AlgaeIntakePivotConstants.kPivotMotorLeftId,AlgaeIntakePivotConstants.kPivotBusId,
          MotorType.kBrushless); // corresponds to left and right motors for pivot
  private SparkMax pivotMotorRight =
      new SparkMax(
        AlgaeIntakePivotConstants.kPivotMotorRightId,AlgaeIntakePivotConstants.kPivotBusId,
        MotorType.kBrushless);

  public AlgaeIntakePivotIOSpark() {
    configureMotors(); // configures motors once algae spark object
  }

  public void configureMotors() {
    pivotMotorLeft.configure( // configures two spark motors
        new SparkMaxConfig()
            .inverted(AlgaeIntakePivotConstants.kLeftInverted)
            .voltageCompensation(AlgaeIntakePivotConstants.kNominalVoltage.in(Volts))
            .smartCurrentLimit(AlgaeIntakePivotConstants.kSmartCurrentLimit)
            .apply(
                new EncoderConfig()
                    .velocityConversionFactor(
                        AlgaeIntakePivotConstants.kPivotVelocityConversionFactor)
                    .positionConversionFactor(
                        AlgaeIntakePivotConstants.kPivotPositionConversionFactor)),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    pivotMotorRight.configure(
        new SparkMaxConfig()
            .voltageCompensation(AlgaeIntakePivotConstants.kNominalVoltage.in(Volts))
            .smartCurrentLimit(AlgaeIntakePivotConstants.kSmartCurrentLimit)
            .apply(
                new EncoderConfig()
                    .velocityConversionFactor(
                        AlgaeIntakePivotConstants.kPivotVelocityConversionFactor)
                    .positionConversionFactor(
                        AlgaeIntakePivotConstants.kPivotPositionConversionFactor))
            .follow(
                AlgaeIntakePivotConstants.kPivotMotorLeftId,
                AlgaeIntakePivotConstants.kRightInverted),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void setPivotVoltage(Voltage volts) {
    pivotMotorLeft.setVoltage(volts);
  }

  public void updateInputs(AlgaeIntakePivotInputs inputs) {
    inputs.pivotAngle = Degrees.of(pivotMotorLeft.getEncoder().getPosition());
    inputs.pivotVelocity = DegreesPerSecond.of(pivotMotorLeft.getEncoder().getVelocity());
    inputs.pivotCurrent = Amps.of(pivotMotorLeft.getOutputCurrent());
  }

  public void resetEncoder(Angle angle) {
    pivotMotorLeft.getEncoder().setPosition(angle.in(Degrees));
  }
}
