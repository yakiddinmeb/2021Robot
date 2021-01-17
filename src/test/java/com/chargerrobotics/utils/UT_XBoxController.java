package com.chargerrobotics.utils;

// Import testing libraries
import static org.mockito.Mockito.*;
import static org.testng.Assert.assertEquals;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import org.testng.annotations.*;

// Import the unit under test

public class UT_XBoxController {

  private com.chargerrobotics.utils.XboxController controller;

  private edu.wpi.first.wpilibj.XboxController internalController;

  @BeforeClass
  public void setUp() {
    // Init unit under test
    controller = new com.chargerrobotics.utils.XboxController();

    internalController = spy(controller.controller);
  }

  /**
   * Tests that when a particular X value is returned from the controller
   * that the deadzone is triggered, resulting in a 0.0 reading
   *
   * @param value The value returned by the physical controller
   * @param hand The stick (left or right) to test against
   */
  @Test(dataProvider = "deadzoneValuesProvider")
  public void testStickXDeadzone(double value, Hand hand) {
    when(internalController.getX(hand)).thenReturn(value);

    double joystickReading = 2.0; // Utilize 2.0 as an invalid value as it is out of the joystick range
    switch (hand) {
      case kLeft:
        joystickReading = controller.getLeftStickX();
      case kRight:
        joystickReading = controller.getRightStickX();
    }

    assertEquals(joystickReading, 0.0);
  }

  /**
   * Tests that when a particular Y value is returned from the controller
   * that the deadzone is triggered, resulting in a 0.0 reading
   *
   * @param value The value returned by the physical controller
   * @param hand The stick (left or right) to test against
   */
  @Test(dataProvider = "deadzoneValuesProvider")
  public void testStickYDeadzone(double value, Hand hand) {
    when(internalController.getY(hand)).thenReturn(value);

    double joystickReading = 2.0; // Utilize 2.0 as an invalid value as it is out of the joystick range
    switch (hand) {
      case kLeft:
        joystickReading = controller.getLeftStickY();
      case kRight:
        joystickReading = controller.getRightStickY();
    }

    assertEquals(joystickReading, 0.0);
  }

  /**
   * Tests that when a particular X value is returned from the controller
   * that the deadzone is not triggered
   *
   * @param value The value returned by the physical controller
   * @param hand The stick (left or right) to test against
   */
  @Test(dataProvider = "nonDeadzoneValuesProvider")
  public void testStickXNoDeadzone(double value, Hand hand) {
    when(internalController.getX(hand)).thenReturn(value);

    assertEquals(controller.getLeftStickX(), value);

    double joystickReading = 2.0; // Utilize 2.0 as an invalid value as it is out of the joystick range
    switch (hand) {
      case kLeft:
        joystickReading = controller.getLeftStickX();
      case kRight:
        joystickReading = controller.getRightStickX();
    }

    assertEquals(joystickReading, value);
  }

  /**
   * Tests that when a particular Y value is returned from the controller
   * that the deadzone is not triggered
   *
   * @param value The value returned by the physical controller
   * @param hand The stick (left or right) to test against
   */
  @Test(dataProvider = "nonDeadzoneValuesProvider")
  public void testStickYNoDeadzone(double value, Hand hand) {
    when(internalController.getY(hand)).thenReturn(value);

    double joystickReading = 2.0; // Utilize 2.0 as an invalid value as it is out of the joystick range
    switch (hand) {
      case kLeft:
        joystickReading = controller.getLeftStickY();
      case kRight:
        joystickReading = controller.getRightStickY();
    }

    assertEquals(joystickReading, value);
  }

  @DataProvider
  public Object[][] deadzoneValuesProvider() {
    return new Object[][] {
      {0.0, Hand.kLeft},
      {0.0, Hand.kRight},
      {0.05, Hand.kLeft},
      {0.05, Hand.kRight},
      {0.10, Hand.kLeft},
      {0.10, Hand.kRight},
      {0.13, Hand.kLeft},
      {0.13, Hand.kRight},
      {0.111111111111, Hand.kLeft},
      {0.111111111111, Hand.kRight},
      {-0.0, Hand.kLeft},
      {-0.0, Hand.kRight},
      {-0.05, Hand.kLeft},
      {-0.05, Hand.kRight},
      {-0.10, Hand.kLeft},
      {-0.10, Hand.kRight},
      {-0.13, Hand.kLeft},
      {-0.13, Hand.kRight},
      {-0.111111111111, Hand.kLeft},
      {-0.111111111111, Hand.kRight},
    };
  }

  @DataProvider
  public Object[][] nonDeadzoneValuesProvider() {
    return new Object[][] {
      {1.0, Hand.kLeft},
      {1.0, Hand.kRight},
      {0.5, Hand.kLeft},
      {0.5, Hand.kRight},
      {0.25, Hand.kLeft},
      {0.25, Hand.kRight},
      {0.2, Hand.kLeft},
      {0.2, Hand.kRight},
      {0.13000000001, Hand.kLeft},
      {0.13000000001, Hand.kRight},
      {-1.0, Hand.kLeft},
      {-1.0, Hand.kRight},
      {-0.5, Hand.kLeft},
      {-0.5, Hand.kRight},
      {-0.25, Hand.kLeft},
      {-0.25, Hand.kRight},
      {-0.2, Hand.kLeft},
      {-0.2, Hand.kRight},
      {-0.13000000001, Hand.kLeft},
      {-0.13000000001, Hand.kRight},
    };
  }
}
