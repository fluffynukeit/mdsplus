/*
		A basic implementation of the DeviceSetup class.
*/

import java.awt.*;

public class RFXGClockSetup extends DeviceSetup
{
	public RFXGClockSetup(Frame parent)
	{
		super(parent);
		
		// This code is automatically generated by Visual Cafe when you add
		// components to the visual environment. It instantiates and initializes
		// the components. To modify the code, only use code syntax that matches
		// what Visual Cafe can generate, or Visual Cafe may be unable to back
		// parse your Java file into its visual environment.
		//{{INIT_CONTROLS
		setDeviceProvider("150.178.3.112");
		setDeviceTitle("RFX Gated Clock Generator");
		setDeviceType("RFXGClock");
		getContentPane().setLayout(null);
		setSize(573,320);
		deviceField1.setNumCols(25);
		deviceField1.setTextOnly(true);
		deviceField1.setOffsetNid(1);
		deviceField1.setLabelString("Comment: ");
		getContentPane().add(deviceField1);
		deviceField1.setBounds(0,12,372,40);
		getContentPane().add(deviceDispatch1);
		deviceDispatch1.setBounds(396,12,131,40);
		deviceField2.setNumCols(20);
		deviceField2.setOffsetNid(2);
		deviceField2.setLabelString("Decoder: ");
		getContentPane().add(deviceField2);
		deviceField2.setBounds(0,48,312,40);
		deviceField3.setNumCols(4);
		deviceField3.setOffsetNid(3);
		deviceField3.setLabelString("Gate chan.:");
		getContentPane().add(deviceField3);
		deviceField3.setBounds(312,48,130,40);
		deviceField4.setNumCols(4);
		deviceField4.setOffsetNid(4);
		deviceField4.setLabelString("Clock chan.:");
		getContentPane().add(deviceField4);
		deviceField4.setBounds(444,48,130,40);
		deviceChoice1.setOffsetNid(5);
		{
			String[] tempString = new String[4];
			tempString[0] = "EVENT";
			tempString[1] = "TRIGGER RISING";
			tempString[2] = "TRIGGER FALLING";
			tempString[3] = "SOFTWARE";
			deviceChoice1.setChoiceItems(tempString);
		}
		deviceChoice1.setLabelString("Trigger mode: ");
		getContentPane().add(deviceChoice1);
		deviceChoice1.setBounds(12,96,252,40);
		deviceField5.setNumCols(20);
		deviceField5.setTextOnly(true);
		deviceField5.setOffsetNid(6);
		deviceField5.setLabelString("Event: ");
		getContentPane().add(deviceField5);
		deviceField5.setBounds(252,96,312,40);
		deviceField6.setNumCols(15);
		deviceField6.setOffsetNid(8);
		deviceField6.setLabelString("Delay: ");
		getContentPane().add(deviceField6);
		deviceField6.setBounds(12,144,264,40);
		deviceField7.setNumCols(15);
		deviceField7.setOffsetNid(10);
		deviceField7.setLabelString("Duration: ");
		getContentPane().add(deviceField7);
		deviceField7.setBounds(276,144,300,40);
		deviceField8.setNumCols(15);
		deviceField8.setOffsetNid(9);
		deviceField8.setLabelString("Frequency : ");
		getContentPane().add(deviceField8);
		deviceField8.setBounds(0,180,276,40);
		deviceChoice2.setOffsetNid(11);
		{
			String[] tempString = new String[6];
			tempString[0] = "SINGLE SWITCH: TOGGLE";
			tempString[1] = "SINGLE SWITCH: HIGH PULSES";
			tempString[2] = "SINGLE SWITCH: LOW PULSES";
			tempString[3] = "DOUBLE SWITCH: TOGGLE";
			tempString[4] = "DOUBLE SWITCH: HIGH PULSES";
			tempString[5] = "DOUBLE SWITCH: LOW PULSES";
			deviceChoice2.setChoiceItems(tempString);
		}
		deviceChoice2.setLabelString("Output mode: ");
		getContentPane().add(deviceChoice2);
		deviceChoice2.setBounds(0,228,324,40);
		deviceField10.setOffsetNid(7);
		deviceField10.setLabelString("Ext. trigger: ");
		getContentPane().add(deviceField10);
		deviceField10.setBounds(276,180,240,40);
		getContentPane().add(deviceButtons1);
		deviceButtons1.setBounds(132,276,324,40);
		//}}
	}

	public RFXGClockSetup()
	{
		this((Frame)null);
	}

	public RFXGClockSetup(String sTitle)
	{
		this();
		setTitle(sTitle);
	}

	public void setVisible(boolean b)
	{
		if (b)
			setLocation(50, 50);
		super.setVisible(b);
	}

	static public void main(String args[])
	{
		(new RFXGClockSetup()).setVisible(true);
	}

	public void addNotify()
	{
		// Record the size of the window prior to calling parents addNotify.
		Dimension size = getSize();

		super.addNotify();

		if (frameSizeAdjusted)
			return;
		frameSizeAdjusted = true;

		// Adjust size of frame according to the insets
		Insets insets = getInsets();
		setSize(insets.left + insets.right + size.width, insets.top + insets.bottom + size.height);
	}

	// Used by addNotify
	boolean frameSizeAdjusted = false;

	//{{DECLARE_CONTROLS
	DeviceField deviceField1 = new DeviceField();
	DeviceDispatch deviceDispatch1 = new DeviceDispatch();
	DeviceField deviceField2 = new DeviceField();
	DeviceField deviceField3 = new DeviceField();
	DeviceField deviceField4 = new DeviceField();
	DeviceChoice deviceChoice1 = new DeviceChoice();
	DeviceField deviceField5 = new DeviceField();
	DeviceField deviceField6 = new DeviceField();
	DeviceField deviceField7 = new DeviceField();
	DeviceField deviceField8 = new DeviceField();
	DeviceChoice deviceChoice2 = new DeviceChoice();
	DeviceField deviceField10 = new DeviceField();
	DeviceButtons deviceButtons1 = new DeviceButtons();
	//}}

}