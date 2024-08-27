package org.firstinspires.ftc.teamcode.subsystems.ledcontroller;

/**
 * <h1>LEDState</h1>
 * <br>
 * <p>
 *     Represents the different states that an LED can be in.
 *
 *     <ul>
 *         <li>RED   - The LED is red (channel one is active)</li>
 *         <li>GREEN - The LED is green (channel two is active)</li>
 *         <li>AMBER - The LED is amber (both channel one and two are active)</li>
 *         <li>OFF   - The LED is off (Neither channel one or two is active)</li>
 *     </ul>
 * </p>
 */
public enum LEDState {
    RED,
    GREEN,
    AMBER,
    OFF
}
