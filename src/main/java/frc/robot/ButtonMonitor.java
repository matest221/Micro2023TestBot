
package frc.robot;

import edu.wpi.first.wpilibj2.command.button.*;

/// Mahesh 1/12/2023 moving to new architecture with Trigger use instead of command Button ! 

public class ButtonMonitor {

    public enum ButtonState {
        Active,
        Inactive,

        NeverActive,
    }
    private Trigger button;
    private ButtonState state;

    public ButtonMonitor(Trigger buttonToMonitor) {
        button = buttonToMonitor;
        state = ButtonState.NeverActive;
    }
    
    public ButtonState checkButtonState(){

        if(button.getAsBoolean()){
            state = ButtonState.Active;
        }
        else if (!button.getAsBoolean() && state == ButtonState.Active) {
            state = ButtonState.Inactive;
        }
        
        return state;
    }
    public Trigger getButton(){
        return button;
    }
}
