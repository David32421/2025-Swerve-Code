// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
// import com.revrobotics.AbsoluteEncoder;
//import com.revrobotics.CANEncoder;
import com.revrobotics.spark.SparkFlex;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
//import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Constants.DriveConstants;
import frc.Constants.ModuleConstants;
import frc.robot.RobotContainer;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */
  //public SwerveModule() {

private final SparkFlex driveMotor;
private final SparkFlex turningMotor;

// private final SparkAbsoluteEncoder driveEncoder;
// private final SparkAbsoluteEncoder turningEncoder;

//private final CANcoder driveEncoder;
private final CANcoder turningEncoder;

private final PIDController turningPidController;

// private final AnalogInput absoluteEncoder;
private final boolean absoluteEncoderReversed;
private final double absoluteEncoderOffsetRad;
  
public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed, 
int turningEncoderId, double absoluteEncoderOffsetRad, boolean absoluteEncoderReversed) {

  this.absoluteEncoderOffsetRad=absoluteEncoderOffsetRad;
  this.absoluteEncoderReversed=absoluteEncoderReversed;
  //absoluteEncoder = new AnalogInput(absoluteEncoderId);

  driveMotor = new SparkFlex(driveMotorId, MotorType.kBrushless);
  turningMotor = new SparkFlex(turningMotorId, MotorType.kBrushless);

  // driveMotor.setInverted(driveMotorReversed);
  // turningMotor.setInverted(turningMotorReversed);

  // driveEncoder = driveMotor.getAbsoluteEncoder();
  //driveEncoder = new CANcoder(11);
  turningEncoder = new CANcoder(turningEncoderId);

  //driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
 // driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
  //turningEncoder.setPositionConversionFactor(2*Math.PI);
  //turningEncoder.setVelocityConversionFactor(2*Math.PI/60);

  turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
  turningPidController.enableContinuousInput(-Math.PI, Math.PI);

resetEncoders();

  }

  public double getTurningPosition()  {
    return turningEncoder.getPosition().getValueAsDouble();
  }

  public double getTurningVelocity() {
    return turningEncoder.getVelocity().getValueAsDouble();
  }

  public double getDriveVelocity() {
    return driveMotor.getEncoder().getVelocity();
  }

  public double getDrivePosition() {
    return driveMotor.getEncoder().getPosition();
  }

public double  getAbsoluteEncoderRad() {
    double angle = turningEncoder.getAbsolutePosition().getValueAsDouble();
    angle *= 2.0 * Math.PI;
    angle -= absoluteEncoderOffsetRad;
    return angle*(absoluteEncoderReversed ? -1.0 : 1.0);
    
}

public void resetEncoders() {

 // driveEncoder.setPositionConversionFactor(0);
  turningEncoder.setPosition(getAbsoluteEncoderRad());

}

public SwerveModuleState getState() {
  return new SwerveModuleState(driveMotor.getEncoder().getVelocity(), new Rotation2d());
}

public void setDesiredState(SwerveModuleState state) {

  if (Math.abs(state.speedMetersPerSecond)<0.001){
    stop();
    return;
  }

  state = SwerveModuleState.optimize(state, getState().angle);
  driveMotor.set(state.speedMetersPerSecond/DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
  turningMotor.set(turningPidController.calculate(getTurningPosition(),state.angle.getRadians()));
  SmartDashboard.putString("Swerve["+ turningEncoder.getDeviceID()+"] state", state.toString());
}

public void stop() {
  driveMotor.set(0);
  turningMotor.set(0);
}

public void writeModuleToDashboard(String tag){
  SmartDashboard.putNumber(tag + "_absSteer", getAbsoluteEncoderRad());
  SmartDashboard.putNumber(tag + "_steer", getTurningPosition());
  SmartDashboard.putNumber(tag + "_drive", getDrivePosition());
}

 // @Override
 // public void periodic() {
    // This method will be called once per scheduler run
 // }
}
