package frc.team832.robot.subsystems;

import frc.team832.lib.vision.ChameleonVisionSubsystem;
import frc.team832.lib.vision.VisionTarget;

public class Vision extends ChameleonVisionSubsystem {

	public Vision() { super("FrontCam", 0.01); }

	@Override
	public void consumeTarget (VisionTarget target) {}
}