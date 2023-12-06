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

public class Drivebase extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public Drivebase() {
    configMotors();
  }

  CANSparkMax m_leftSlave = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax m_leftMaster = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax m_rightSlave = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax m_rightMaster = new CANSparkMax(4, MotorType.kBrushless);

  DifferentialDrive m_DifferentialDrive = new DifferentialDrive(m_leftMaster,m_rightMaster);

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

    //set smartcurrentlimit to 40 
    //write in a comment the difference between stalllimit and freelimit
    //create a simple arcade drive method that limits the speed by half
    //teach emma 

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

  
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
