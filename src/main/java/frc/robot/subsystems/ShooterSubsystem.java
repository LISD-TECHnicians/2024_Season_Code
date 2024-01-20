package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX pivotLeft = new TalonFX(ShooterConstants.PIVOT_LEFT_ID, "rio");
  private final TalonFX pivotRight = new TalonFX(ShooterConstants.PIVOT_RIGHT_ID, "rio");

  private final CANSparkMax shooterLeft = new CANSparkMax(ShooterConstants.SHOOTER_LEFT_ID, MotorType.kBrushless);
  private final CANSparkMax shooterRight = new CANSparkMax(ShooterConstants.SHOOTER_RIGHT_ID, MotorType.kBrushless);

  private final DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(ShooterConstants.PIVOT_ENCODER_PORT);

  private final PIDController pivotPID = new PIDController(ShooterConstants.PIVOT_P, ShooterConstants.PIVOT_I, ShooterConstants.PIVOT_D);

  public ShooterSubsystem() {
    pivotRight.setControl(new Follower(pivotLeft.getDeviceID(), true));
    shooterRight.follow(shooterLeft, true);
  }

  public void setShooterSpeed() {
    shooterLeft.set(1.0);
  }

  public void setShooterSpeed(double speed) {
    shooterLeft.set(speed);
  }

  public void setPivotAngle(double angle) {
    pivotLeft.set(MathUtil.clamp(pivotPID.calculate(getPivotAngle(), angle), -1.0, 1.0));
  }

  public double getPivotAngle() {
    return Units.rotationsToRadians(pivotEncoder.getAbsolutePosition()); //  convert to radians
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
