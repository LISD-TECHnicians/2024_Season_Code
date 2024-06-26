package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.PivotConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

public class PivotSubsystem extends SubsystemBase {
  private final CANSparkMax pivotLeft = new CANSparkMax(PivotConstants.PIVOT_LEFT_ID, MotorType.kBrushless);
  private final CANSparkMax pivotRight = new CANSparkMax(PivotConstants.PIVOT_RIGHT_ID, MotorType.kBrushless);

  private final PIDController pivotPID = new PIDController(PivotConstants.PIVOT_P, PivotConstants.PIVOT_I, PivotConstants.PIVOT_D);

  // Encoder in Right SparkMax, Limits in Left SparkMax

  public PivotSubsystem() {
    pivotLeft.restoreFactoryDefaults();
    pivotRight.restoreFactoryDefaults();

    pivotLeft.enableVoltageCompensation(ControllerConstants.NOMINAL_VOLTAGE);
    pivotRight.enableVoltageCompensation(ControllerConstants.NOMINAL_VOLTAGE);

    pivotLeft.setSmartCurrentLimit(PivotConstants.PIVOT_CURRENT_LIMIT);
    pivotRight.setSmartCurrentLimit(PivotConstants.PIVOT_CURRENT_LIMIT);

    pivotLeft.setIdleMode(IdleMode.kBrake);
    pivotRight.setIdleMode(IdleMode.kBrake);

    pivotLeft.setSoftLimit(SoftLimitDirection.kForward, PivotConstants.PIVOT_FORWARD_LIMIT);
    pivotLeft.setSoftLimit(SoftLimitDirection.kReverse, PivotConstants.PIVOT_REVERSE_LIMIT);

    pivotLeft.enableSoftLimit(SoftLimitDirection.kForward, true);
    pivotLeft.enableSoftLimit(SoftLimitDirection.kReverse, true);

    pivotRight.follow(pivotLeft, true);

    pivotLeft.burnFlash();
    pivotRight.burnFlash();
  }

  public void setPivotAngle(double angle) {
    pivotLeft.set(-MathUtil.clamp(pivotPID.calculate(getPivotAngle(), angle), -PivotConstants.PIVOT_MAX_OUTPUT, PivotConstants.PIVOT_MAX_OUTPUT));
  }

  public double getPivotRawAngle(){
    return pivotLeft.getEncoder().getPosition();
  }

  public double getPivotAngle() {
    return -pivotLeft.getEncoder().getPosition() * PivotConstants.PIVOT_GEAR_RATIO * 360 + PivotConstants.PIVOT_INITIAL_ANGLE; //  degrees
  }

  public boolean getShooterReadiness(double angle) {
    return Math.abs(getPivotAngle() - angle) < PivotConstants.PIVOT_VARIABILITY;
  }

  public boolean getIntakeReadiness() {
    return Math.abs(getPivotAngle() - PivotConstants.INTAKE_ANGLE) < PivotConstants.PIVOT_VARIABILITY;
  }

  public boolean getTravelReadiness() {
    return Math.abs(getPivotAngle() - PivotConstants.TRAVEL_ANGLE) < PivotConstants.PIVOT_VARIABILITY;
  }

  public boolean getClimberReadiness() {
    return Math.abs(getPivotAngle() - PivotConstants.CLIMBER_ANGLE) < PivotConstants.PIVOT_VARIABILITY;
  }

  public boolean getAmpReadiness() {
    return Math.abs(getPivotAngle() - PivotConstants.AMP_ANGLE) < PivotConstants.PIVOT_VARIABILITY;
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
