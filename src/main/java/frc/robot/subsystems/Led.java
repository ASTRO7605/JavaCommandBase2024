// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.AddressableLED;
// import edu.wpi.first.wpilibj.AddressableLEDBuffer;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.LedConstants;

// public class Led extends SubsystemBase {
//     private final AddressableLED m_led = new AddressableLED(LedConstants.kLedChannel);
//     private final AddressableLEDBuffer m_buffer = new AddressableLEDBuffer(LedConstants.kNumLeds);
//     private LedConstants.Animation m_currentAnim = LedConstants.Animation.ALLIANCE;

//     private boolean m_isNoteInIntake = false;
//     private boolean m_isNoteSeen = false;
//     private boolean m_isRobotInRange = true;
//     private boolean m_isRobotAlignedToShoot = false;

//     private int m_alternatePrescaleCounter = 0;
//     private boolean m_alternateIsFirstColor = true;

//     private double m_colorSweepPrescaleCounter = 0.0;
//     private int m_colorSweepAnimCounter = 0;

//     private double m_splitWithBlinkPrescaleCounter = 0.0;
//     private boolean m_splitWithBlinkClock = true;

//     public Led() {
//         m_led.setLength(LedConstants.kNumLeds);
//         m_led.setData(m_buffer);
//         m_led.start();
//     }

//     @Override
//     public void periodic() {
//         switch (m_currentAnim) {
//             case ALLIANCE: {
//                 var alli = DriverStation.getAlliance();
//                 if (alli.isPresent() && alli.get() == DriverStation.Alliance.Red) {
//                     colorSweep(LedConstants.Colors.redAlliance);
//                 } else if (alli.isPresent() && alli.get() == DriverStation.Alliance.Blue) {
//                     colorSweep(LedConstants.Colors.blueAlliance);
//                 } else {
//                     alternate(LedConstants.Colors.redAlliance, LedConstants.Colors.blueAlliance);
//                 }
//                 break;
//             }

//             case SPLIT: {
//                 LedConstants.Color top = LedConstants.Colors.off;
//                 if (m_isNoteInIntake) {
//                     top = LedConstants.Colors.noteInIntake;
//                 } else if (m_isNoteSeen) {
//                     top = LedConstants.Colors.noteSeen;
//                 }
//                 LedConstants.Color bottom = m_isRobotInRange ? LedConstants.Colors.robotInRange
//                         : LedConstants.Colors.off;
//                 splitWithBlink(top, m_isNoteInIntake, bottom, m_isRobotAlignedToShoot);

//                 break;
//             }
//             default:
//                 break;
//         }
//     }

//     public void setAnimation(LedConstants.Animation animation) {
//         m_currentAnim = animation;
//     }

//     public LedConstants.Animation getAnimation() {
//         return m_currentAnim;
//     }

//     public void setIsNoteInIntake(boolean isNoteInIntake) {
//         m_isNoteInIntake = isNoteInIntake;
//     }

//     public void setIsNoteSeen(boolean isNoteSeen) {
//         m_isNoteSeen = isNoteSeen;
//     }

//     public void setIsRobotInRange(boolean isRobotInRange) {
//         m_isRobotInRange = isRobotInRange;
//     }

//     public void setIsRobotAlignedToShoot(boolean isRobotAlignedToShoot) {
//         m_isRobotAlignedToShoot = isRobotAlignedToShoot;
//     }

//     private void alternate(LedConstants.Color color1, LedConstants.Color color2) {
//         ++m_alternatePrescaleCounter;

//         if (m_alternatePrescaleCounter < LedConstants.kAlternatePrescale) {
//             // prescale counter not reached yet
//             return;
//         }

//         m_alternatePrescaleCounter = 0;
//         m_alternateIsFirstColor = !m_alternateIsFirstColor;

//         LedConstants.Color current = m_alternateIsFirstColor ? color1 : color2;

//         for (int led = 0; led < LedConstants.kNumLeds; ++led) {
//             m_buffer.setRGB(led, current.red(), current.green(), current.blue());
//         }

//         m_led.setData(m_buffer);
//     }

//     private void colorSweep(LedConstants.Color color) {
//         m_colorSweepPrescaleCounter += (1.0 / LedConstants.kSweepPrescale);

//         // calculate value to compare against as being prescale_counter after
//         // kRequestedPrescale increments (may not be equal to 1)
//         if (m_colorSweepPrescaleCounter >= (LedConstants.kSweepPrescale * (1.0 / LedConstants.kSweepPrescale))) {
//             // reset prescale and increment animation
//             m_colorSweepPrescaleCounter = 0.0;
//             ++m_colorSweepAnimCounter;

//             if (m_colorSweepAnimCounter >= LedConstants.kNumLeds)
//                 m_colorSweepAnimCounter = 0;
//         }

//         // sequence comprised of 1 'fade out' LED, 8 'full on' LEDs, 1 'fade in' LED

//         // current LED to update, incremented after setting a LED
//         int led = m_colorSweepAnimCounter;

//         // 'fade out' LED at beginning of sequence
//         // fade out as prescale increases
//         m_buffer.setRGB(led, (int) (color.red() * (1.0 - m_colorSweepPrescaleCounter)),
//                 (int) (color.green() * (1.0 - m_colorSweepPrescaleCounter)),
//                 (int) (color.blue() * (1.0 - m_colorSweepPrescaleCounter)));

//         // next LED
//         ++led;
//         if (led >= LedConstants.kNumLeds)
//             led = 0;

//         // turn on LEDs in middle of sequence
//         for (int count = 0; count < LedConstants.kNumSweepFullOnLeds; ++count) {

//             m_buffer.setRGB(led, color.red(), color.green(), color.blue());

//             // next LED
//             ++led;
//             if (led >= LedConstants.kNumLeds)
//                 led = 0;
//         }

//         // 'fade in' LED at end of sequence
//         // fade in as prescale increases
//         m_buffer.setRGB(led, color.red() * (int) m_colorSweepPrescaleCounter,
//                 color.green() * (int) m_colorSweepPrescaleCounter,
//                 color.blue() * (int) m_colorSweepPrescaleCounter);

//         // next LED
//         ++led;
//         if (led >= LedConstants.kNumLeds)
//             led = 0;

//         // turn off LEDs past sequence
//         for (int count = 0; count < (LedConstants.kNumLeds - LedConstants.kNumSweepFullOnLeds - 2); ++count) {
//             m_buffer.setRGB(led, LedConstants.Colors.off.red(), LedConstants.Colors.off.green(),
//                     LedConstants.Colors.off.blue());

//             // next LED
//             ++led;
//             if (led >= LedConstants.kNumLeds)
//                 led = 0;
//         }

//         m_led.setData(m_buffer);
//     }

//     private void splitWithBlink(LedConstants.Color top, boolean topBlink, LedConstants.Color bottom,
//             boolean bottomBlink) {
//         ++m_splitWithBlinkPrescaleCounter;

//         if (m_splitWithBlinkPrescaleCounter >= LedConstants.kFlashPrescale) {
//             m_splitWithBlinkPrescaleCounter = 0;
//             m_splitWithBlinkClock = !m_splitWithBlinkClock;
//         }

//         if (bottomBlink && !m_splitWithBlinkClock) {
//             bottom = LedConstants.Colors.off;
//         }

//         if (topBlink && !m_splitWithBlinkClock) {
//             top = LedConstants.Colors.off;
//         }

//         for (int led = 0; led < (LedConstants.kNumLeds / 2); ++led) {
//             m_buffer.setRGB(led, bottom.red(), bottom.green(), bottom.blue());
//         }

//         for (int led = (LedConstants.kNumLeds / 2); led < (LedConstants.kNumLeds); ++led) {
//             m_buffer.setRGB(led, top.red(), top.green(), top.blue());
//         }

//         m_led.setData(m_buffer);
//     }
// }
