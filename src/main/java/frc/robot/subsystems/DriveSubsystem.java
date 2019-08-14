/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.commands.DriveManually;

/**
 * Add your docs here.
 */
public class DriveSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // instantiate new motor controller objects
  public TalonSRX leftMaster = new TalonSRX(RobotMap.leftMasterPort);
  public TalonSRX leftSlave = new TalonSRX(RobotMap.leftSlavePort);
  public TalonSRX rightMaster = new TalonSRX(RobotMap.rightMasterPort);
  public TalonSRX rightSlave = new TalonSRX(RobotMap.rightSlavePort);

// instantiate a new DifferentialDrive object and assign motor controllers to differential drive
public DifferentialDrive diffDrive = new DifferentialDrive(leftMaster,rightMaster);

// create constructor function
public DriveSubsystem() {
// point slaves to masters
leftSlave.follow(leftMaster);
rightSlave.follow(rightMaster);
}

// add manualDrive() method
public void manualDrive(double move, double turn){

 /*
  limits max speed to half at all times 
  if(move > .5) move = .5;
*/
 
  if (Math.abs(move) < 0.10) {
    move = 0;
  }
  if (Math.abs(turn) < 0.10) {
    turn = 0;
  }
  diffDrive.arcadeDrive(move, turn);
}




  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DriveManually());
  }
}
