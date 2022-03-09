/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
// public class ButtonMap {
public enum XboxButtons {
	toggleFOD(5), 		// Left Bumper
	kBumperRight(6), 	// Right Bumper
	kStickLeft(9), 		// Left Joystick
	kStickRight(10), 	// Right Joysick
	kA(1), 				// A Button
	kB(2), 				// B Button
	kX(3), 				// X Button
	lockOn(4), 			// Y Button
	restLimelight(7), 	// Back Button
	resetNavx(8); 		// Start Button

	private int value;

	XboxButtons(int value) {
		this.value = value;
	}

}