#include <Pololu3pi.h>
#include <PololuQTRSensors.h>
#include <OrangutanMotors.h>
#include <OrangutanAnalog.h>
#include <OrangutanLCD.h>
#include <OrangutanPushbuttons.h>

Pololu3pi robot;
unsigned int sensors[5]; // an array to hold sensor values
OrangutanAnalog analog;
OrangutanLCD lcd;
OrangutanPushbuttons buttons;
OrangutanMotors motors;

// Initializes the 3pi, displays a welcome message, calibrates, and
// plays the initial music.  This function is automatically called
// by the Arduino framework at the start of program execution.
void setup()
{
  unsigned int counter; // used as a simple timer

  // This must be called at the beginning of 3pi code, to set up the
  // sensors.  We use a value of 2000 for the timeout, which
  // corresponds to 2000*0.4 us = 0.8 ms on our 20 MHz processor.
  robot.init(2000);


  // Display battery voltage and wait for button press
  while (!buttons.isPressed(BUTTON_B))
  {
    int bat = analog.readBatteryMillivolts();

    lcd.clear();
    lcd.print(bat);
    lcd.print("mV");
    lcd.gotoXY(0, 1);
    lcd.print("Press B");

    delay(100);
  }

  // Always wait for the button to be released so that 3pi doesn't
  // start moving until your hand is away from it.
  buttons.waitForRelease(BUTTON_B);
  delay(1000);

  // Auto-calibration: turn right and left while calibrating the
  // sensors.
  for (counter=0; counter<80; counter++)
  {
    if (counter < 20 || counter >= 60)
      motors.setSpeeds(40, -40);
    else
      motors.setSpeeds(-40, 40);

    // This function records a set of sensor readings and keeps
    // track of the minimum and maximum values encountered.  The
    // IR_EMITTERS_ON argument means that the IR LEDs will be
    // turned on during the reading, which is usually what you
    // want.
    robot.calibrateLineSensors(IR_EMITTERS_ON);

    // Since our counter runs to 80, the total delay will be
    // 80*20 = 1600 ms.
    delay(20);
  }
  
  lcd.clear();
  lcd.print("Press B");
  
  motors.setSpeeds(0, 0);
  buttons.waitForPress(BUTTON_B);
  buttons.waitForRelease(BUTTON_B);
  lcd.clear();
  lcd.print("Go!");
}

// The main function.  This function is repeatedly called by
// the Arduino framework.
void loop()
{
  // Get the position of the line.  Note that we *must* provide
  // the "sensors" argument to read_line() here, even though we
  // are not interested in the individual sensor readings.
  unsigned int position = robot.readLine(sensors, IR_EMITTERS_ON);
  
  lcd.clear();
  lcd.print(position);
  int base_position = 2000;
  int error = base_position - position;
  int base_speed = 100;
  
  error = error * 0.05; // error is proportional
  int Lspd = base_speed-error;
  int Rspd = base_speed+error;
  motors.setSpeeds(Lspd, Rspd);
  delay(10);
  int prev_error = error;  
  position = robot.readLine(sensors, IR_EMITTERS_ON);
  error = base_position - position; 
  error = error * 0.05;
  int derivative = prev_error - error;
  Lspd = base_speed-error-derivative;
  Rspd = base_speed+error+derivative;
  motors.setSpeeds(Lspd, Rspd);
  
  // speed = base + kp* error (+/-) kd*prev_error
  
  //error has a max of ±2000
  //2000 * 0.05 = 100, therefore
  //the maximum speed possible
  //is 250, and 250 < 255,
  //so we use a scalar of 0.


}
