/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class autoShootingMidOfField extends SequentialCommandGroup {

  /**
   * Creates a new autoShootingAlignedWithTarget.
   */
  
  public autoShootingMidOfField() {
    super();
    new autoShootingCommandGroup(0.762, 1.6764, 0, 0, 0);
  }
}
