package frc.robot;

import edu.wpi.first.wpilibj2.command.button.Button;

import java.util.function.BooleanSupplier;

public class CustomButton extends Button {
	public CustomButton(BooleanSupplier supplier) {
		super(supplier);
	}
}