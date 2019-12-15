package frc.team832.robot;

import frc.team832.lib.driverstation.controllers.Attack3;
import frc.team832.lib.driverstation.controllers.Extreme3DPro;

public class OI {
    public final Attack3 leftStick;
    public final Extreme3DPro rightStick;

    public OI() {
        leftStick = new Attack3(0);
        rightStick = new Extreme3DPro(1);

        // do commands here
    }
}