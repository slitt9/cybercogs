# Slow object-edge wall follower

Open **ObjectWallFollower.ino** in Arduino IDE, board **Arduino UNO R4 WiFi**. Complete single file; no manual commands. Starts three seconds after reset. Uses your reference motor wiring and polarity, and front D5/D6 plus right D7/D8.

## New behavior

- Right-wall PID target remains15 cm. Cruise75 PWM, alignment70, turn80.
- **Right-side relevance cap:25 cm.** Two consecutive readings beyond25 cm, or two missing right echoes, mean the close wall/object has ended. A distant background wall does not count as the wall being followed.
- With a usable front reading and no close front obstacle, loss of the right wall starts **right_edge_turn**: left wheel forward at80 PWM, right wheel stopped. This is a definite right turn rather than the old almost-straight75/65 command.
- The next side is reacquired after **three fresh stable readings between8 and22 cm**, changing by at most1 cm per reading, with a clear front. Then both wheels creep forward using PID. The tighter22 cm reacquisition boundary prevents repeated switching at the25 cm loss threshold.
- **No stationary confirm_side state.** Stable readings are checked while moving. Front clearance ends the left pivot immediately into moving PID alignment; it no longer stops diagonally to gather readings.
- After a left turn, at least600 ms of gentle alignment is allowed before a missing side triggers the next right turn. This short clearance interval helps get past the first corner. During normal following, two lost-side readings trigger the turn directly.
- Entering and leaving either turn cancels the old wheel commands before ramping the new command, avoiding extra turning during the transition.

## Distance caps and front protection

Front readings beyond60 cm are treated as60 cm internally: simply clear space. Raw distances are still shown in telemetry. Front obstacles within30 cm trigger a left pivot;34 cm is required for clear-front confirmation. A reading12 cm or less stops both motors until at least16 cm clearance returns.

**A missing front echo is not converted into a clear reading.** It still pauses motion until valid readings return. The right-side no-echo case does trigger a right turn when the front sensor is working. If the robot pauses with front_cm=nan, that is sensor_wait, not wall-loss steering.

## Watch on the first test

Serial Monitor9600 baud:

`ms,front_cm,right_cm,state,left_pwm,right_pwm,stable_side_samples`

At the box edge, right_cm should rise above25 or become nan; state should become **right_edge_turn**, with positive left_pwm and right_pwm=0. Once three side readings reacquire the next surface, state becomes **pid_align**, then **pid_follow**, with both wheel commands positive.

Keep the motor battery connected for motor power. Lower PWM can stall a wheel: raise the relevant speed slightly if needed. Two ultrasonic sensors cannot measure a precise90-degree heading or prove whole-chassis clearance, so verify one corner at a time on the floor. If the inside wheel coasts, actual turning radius also depends on grip and chassis geometry.

Sensor-fault stops and bounded failed-maneuver retries remain. Reset/power-cycle after a final blocked stop. Use the power switch to stop an autonomous run.
