// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax.IdleMode;
import frc.robot.Constants.DrivebaseConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.MotorIDConstants;

public class Drivebase extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public Drivebase() {
    configMotors();  //
  }


  CANSparkMax m_leftSlave = new CANSparkMax(MotorIDConstants.kLeftSlave, MotorType.kBrushless);  //brushless > brush
  CANSparkMax m_leftMaster = new CANSparkMax(MotorIDConstants.kLeftMaster, MotorType.kBrushless);
  CANSparkMax m_rightSlave = new CANSparkMax(MotorIDConstants.kRightSlave, MotorType.kBrushless);
  CANSparkMax m_rightMaster = new CANSparkMax(MotorIDConstants.kRightMaster, MotorType.kBrushless);

  RelativeEncoder m_leftEncoder  = m_leftMaster.getEncoder(Type.kQuadrature,DrivebaseConstants.kCountsPerRev);
  RelativeEncoder m_rightEncoder = m_rightMaster.getEncoder(Type.kQuadrature,DrivebaseConstants.kCountsPerRev);

  DifferentialDrive m_differentialDrive = new DifferentialDrive(m_leftMaster,m_rightMaster);

  public void configMotors(){  
    m_leftSlave.follow(m_leftMaster);
    m_rightSlave.follow(m_rightMaster);

    m_leftMaster.setInverted(true);
    m_leftSlave.setInverted(m_leftMaster.getInverted());

    m_leftMaster.restoreFactoryDefaults();   
    m_leftSlave.restoreFactoryDefaults();
    m_rightMaster.restoreFactoryDefaults();   
    m_rightSlave.restoreFactoryDefaults();
    

    m_leftMaster.setIdleMode(IdleMode.kBrake);
    m_leftSlave.setIdleMode(m_leftMaster.getIdleMode());
    m_rightMaster.setIdleMode(m_leftMaster.getIdleMode());
    m_rightSlave.setIdleMode(m_leftMaster.getIdleMode());

    m_leftMaster.setSmartCurrentLimit(DrivebaseConstants.kStallLimit,DrivebaseConstants.kFreeLimit);
    m_leftSlave.setSmartCurrentLimit(DrivebaseConstants.kStallLimit,DrivebaseConstants.kFreeLimit)
    m_rightMaster.setSmartCurrentLimit(DrivebaseConstants.kStallLimit,DrivebaseConstants.kFreeLimit);
    m_rightSlave.setSmartCurrentLimit(DrivebaseConstants.kStallLimit,DrivebaseConstants.kFreeLimit);


    //stallimit - minimum amount of torque that the motor needs to "break the stall" 
    //free - maximum amount of power distributed into the motor

  }


  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }
  public CommandBase die() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          m_DifferentialDrive.arcadeDrive(1, 0);
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   * 
   * 
   *
   */

   public limitedArcadeDrive(double speed, double rotation){   //runs arcade drive
      m_differentialDrive.arcadeDrive(speed*0.5, rotation*0.5); //half power so motors don't burn out
   }

    public arcadeDrive(double speed, double rotation){
      m_differentialDrive.arcadeDrive(speed, rotation); //full power when u need more power
   }

   public double getAverageDistance(){ // the left distance + right distance divided by two to get the average distance of both encoders
      return (getLeftDistance() + getRightDistance())/2;
   }

  public double getLeftDistance(){ 
      double circumference = Math.PI * DrivebaseConstants.kWheelDiameterInches; //stores the circumference variable; circumference = 
      double revolutions = (m_leftEncoder.getPosition() / m_leftEncoder.getCountsPerRevolution());
      //distance = circumference * revolutions  (ARC LENGTH)
      return (circumference * revolution);
  }

  public double getRightDistance(){
    double circumference = MATH.PI * DrivebaseConstants.kWheelDiameterInches;
    double revolutions = m_rightEncoder.getPosition() / m_rightEncoder.getCountsPerRevolution();
    return (circumference * revolution);
  }



  
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("left distance", getLeftDistance());
    SmartDashboard.putNumber("right distance", getRightDistance());
    SmartDashboard.putNumber("average distance", getAverageDistance());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
