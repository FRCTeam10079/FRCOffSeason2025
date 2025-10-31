// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  
  // MASTER STATE MACHINE - Controls EVERYTHING
  private final RobotStateMachine m_stateMachine;

  public Robot() {
    m_robotContainer = new RobotContainer();
    m_stateMachine = RobotStateMachine.getInstance();
  }

  @Override
  public void robotPeriodic() {
    // Update master state machine
    m_stateMachine.periodic();
    
    // Run command scheduler
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {
    // State machine transition: Robot disabled
    m_stateMachine.setMatchState(RobotStateMachine.MatchState.DISABLED);
  }

  @Override
  public void disabledPeriodic() {
    // Stay in disabled state
  }

  @Override
  public void disabledExit() {
    // Leaving disabled state
    System.out.println("Exiting disabled mode...");
  }

  @Override
  public void autonomousInit() {
    // State machine transition: Autonomous starting
    m_stateMachine.setMatchState(RobotStateMachine.MatchState.AUTO_INIT);
    
    // Get and schedule autonomous command
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
      // Transition to running state
      m_stateMachine.setMatchState(RobotStateMachine.MatchState.AUTO_RUNNING);
    }
  }

  @Override
  public void autonomousPeriodic() {
    // Autonomous is running - state machine tracks this
  }

  @Override
  public void autonomousExit() {
    // Autonomous ending
    System.out.println("Autonomous period ended");
  }

  @Override
  public void teleopInit() {
    // State machine transition: Teleop starting
    m_stateMachine.setMatchState(RobotStateMachine.MatchState.TELEOP_INIT);
    
    // Cancel autonomous command
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
    // Transition to running state
    m_stateMachine.setMatchState(RobotStateMachine.MatchState.TELEOP_RUNNING);
  }

  @Override
  public void teleopPeriodic() {
    // Teleop is running - state machine tracks endgame automatically
  }

  @Override
  public void teleopExit() {
    // Teleop ending
    System.out.println("Teleop period ended");
  }

  @Override
  public void testInit() {
    // State machine transition: Test mode starting
    m_stateMachine.setMatchState(RobotStateMachine.MatchState.TEST_INIT);
    
    CommandScheduler.getInstance().cancelAll();
    
    // Transition to running state
    m_stateMachine.setMatchState(RobotStateMachine.MatchState.TEST_RUNNING);
  }

  @Override
  public void testPeriodic() {
    // Test mode is running
  }

  @Override
  public void testExit() {
    // Test mode ending
    System.out.println("Test mode ended");
  }

  @Override
  public void simulationPeriodic() {
    // Simulation running
  }
}
