package frc.robot.Devices;
import java.util.HashMap;
import edu.wpi.first.wpilibj.XboxController;
public class Controller extends XboxController {
    private HashMap<Button, Boolean> debounceButtons = new HashMap<Button, Boolean>();
    public Controller(int port) {
        super(port);
    }
    public boolean getDebouncedButton(Button button) {
        if (!debounceButtons.containsKey(button)) {
            debounceButtons.put(button, false);
        }
        if (this.getRawButton(button.value) && debounceButtons.get(button))
        {
            debounceButtons.put(button, false);
            return false;
        }
        else if (this.getRawButtonPressed(button.value))
        {
            debounceButtons.put(button, true);
            return true;
        }
        else
        {
            return debounceButtons.get(button);
        }
    }
}
