package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

// import frc.robot.commands.AutoTurn;
//import frc.robot.commands.BallIntake;
// import frc.robot.commands.BallLiftDown;
// import frc.robot.commands.BallLiftUp;
//import frc.robot.commands.BallOuttake;
//import frc.robot.commands.HatchMotorForward;
//import frc.robot.commands.HatchMotorReverse;
//import frc.robot.commands.SolenoidForward;
//import frc.robot.commands.SolenoidReverse;

import frc.robot.commands.UltraDrive;



public class OI {

    public Joystick driveJoystick = new Joystick(Constants.DRIVE_JOYSTICK);
    
    private Joystick mechJoystick = new Joystick(Constants.JOYSTICK);

    //Joystick (Parke)
    JoystickButton intakeIn = new JoystickButton(mechJoystick, Constants.JOYB_B);
    JoystickButton cargoLiftUp = new JoystickButton(mechJoystick, Constants.JOYB_X);

    JoystickButton hubAlignVisionLeft = new JoystickButton(driveJoystick, Constants.JOYB_LB);
    JoystickButton hubAlignVisionRight = new JoystickButton(driveJoystick, Constants.JOYB_RB);

    
    JoystickButton caitlinButton = new JoystickButton(mechJoystick, Constants.JOYB_LB);

    JoystickButton reverseCargo = new JoystickButton(mechJoystick, Constants.JOYB_RJ);



    //driveJoystick (Eva)
    //JoystickButton LLDrive = new JoystickButton(driveJoystick, Constants.JOYB_B);

    JoystickButton climberUp = new JoystickButton(mechJoystick, Constants.JOYB_Y);
    JoystickButton climberDown = new JoystickButton(mechJoystick, Constants.JOYB_A);

    edu.wpi.first.wpilibj2.command.button.POVButton solenoidDumpForward = new POVButton(mechJoystick, Constants.POVB_N);
    POVButton solenoidDumpReverse = new POVButton(mechJoystick, Constants.POVB_S);

    POVButton autoLeft = new POVButton(driveJoystick, Constants.POVB_W);
    POVButton autoRight = new POVButton(driveJoystick, Constants.POVB_E);
    POVButton autoForward = new POVButton(driveJoystick, Constants.POVB_N);
    POVButton autoBackward = new POVButton(driveJoystick, Constants.POVB_S);
    // JoystickButton driveForward = new JoystickButton(driveJoystick, Constants.JOYB_A);


    public Joystick getDriveJoystick() {
      return driveJoystick;
    }

    public Joystick getJoystick(){
       return mechJoystick;
    }

    public OI(){


      autoForward.whileTrue(new UltraDrive(autoForward, -0.3)); // (1) = direction, -1 for backward movement and 1 for forward movement
      autoBackward.whileTrue(new UltraDrive(autoBackward, 0.3));

    }

    

}