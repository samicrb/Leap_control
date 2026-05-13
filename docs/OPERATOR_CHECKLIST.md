# Operator Checklist — Doosan Gesture Demo

Print this page and keep it next to the supervisor PC during the event.

---

## A. Cold-start sequence

- [ ] Robot controller (A0509, V3.5) powered, emergency stop released.
- [ ] Leap Motion sensor plugged in (USB), LED visible.
- [ ] Ultraleap service (Gemini 6.2.0) running on the host PC.
- [ ] Demo object positioned on the pick spot.
- [ ] PC console window visible to the operator.
- [ ] `doosan_gesture_demo.exe` launched. Header reads **DEMO INACTIVE**.
- [ ] Robot moved to the safe pose (visible end-of-boot status).

## B. Pre-visitor smoke test (run every morning)

Go through each step. Expected result in parentheses.

1. **Button off, hands present.** Nothing moves. State stays `IDLE`.
2. **Press SPACE.** State becomes `READY`. Prompt asks for both hands.
3. **Show only left hand.** Still `READY`. Prompt: show right hand.
4. **Show only right hand.** Still `READY`. Prompt: show left hand.
5. **Show both hands, left open, right closed.** State becomes
   `POSITION`. Moving the right hand up makes the tool go up
   (if not, flip `motion.sign_z`).
6. **Open right hand.** State becomes `RECENTER`, robot freezes
   immediately.
7. **Close right hand again.** State returns to `POSITION`, no jump.
8. **Switch left to closed fist, keep right closed.** State becomes
   `ORIENTATION`. Right-hand rotation drives the tool orientation.
9. **Remove the left hand while in position mode.** State jumps to
   `FAULT` with reason "left hand lost". Robot freezes.
10. **Remove both hands, wait, then show them again.** State returns to
    `READY`. No automatic motion.
11. **Place both palms facing each other > 170 mm apart.** After the
    gesture hold time the UI reads `GRIPPER ARMED`. Closing the hands
    through the neutral band and past 50 mm triggers **CLOSE**. State
    returns to `READY` immediately after the impulse.
12. **Repeat the gripper gesture the other way.** Expected: **OPEN**
    impulse.
13. **Workspace envelope test** (only if `workspace.enabled = true`):
    push the hand toward a workspace boundary. The UI shows `[LIMIT]`;
    the robot slides along the limit but does not cross it. If the
    envelope is disabled (default), skip this step.

## C. Full pick-and-place cycle

- [ ] Engage position mode.
- [ ] Guide the tool over the demo object.
- [ ] Use the gripper gesture (OPEN) to pre-open the gripper.
- [ ] Re-engage position mode. Lower the tool.
- [ ] Use the gripper gesture (CLOSE) to grasp the object.
- [ ] Re-engage. Move to the drop spot.
- [ ] Use the gripper gesture (OPEN) to release.
- [ ] Re-engage and step the tool back to safe area.

## D. Mid-day / between visitors

- [ ] Quick recenter test (steps 6–7 of section B).
- [ ] One pick-and-place dry cycle without a visitor.
- [ ] Verify the workspace limit still behaves (step 13) — only if enabled.

## E. Shutdown

- [ ] Press SPACE → state returns to `IDLE`.
- [ ] Press `Q` in the console window → clean shutdown.
- [ ] Robot stops, servo disengages, connection closes (log confirms).
- [ ] Turn off controller, leave safe pose on teach pendant.

## F. Degraded-mode recipes

| Symptom                                       | Action                                                              |
|-----------------------------------------------|---------------------------------------------------------------------|
| Sensor LED off                                | Reseat USB. Restart Ultraleap service.                              |
| Hand detection flickers                       | Increase `gesture.min_confidence` by 0.1. Check lighting.           |
| Robot feels sluggish                          | Increase `motion.position_scale` by 0.1. Decrease smoothing by 0.05.|
| Robot feels nervous                           | Increase smoothing by 0.05. Increase dead-zone by 1 mm.             |
| Gripper double-triggers                       | Raise `gripper.cooldown_s` to 1.2.                                   |
| Gripper never triggers                        | Check two-hand posture: palms must face each other. Relax `facing_dot_max` to `-0.3`. |
| Workspace limit hit too often                 | Widen envelope carefully; test reach first. Or set `workspace.enabled = false` for free motion. |
| `FAULT` keeps returning                       | Check log for the reason. Usually: hand loss, sensor stutter.       |
| Robot disconnects during the run              | Verify cable. Restart app. Verify `robot.ip` / port.                |

## G. End-of-event

- [ ] Reset config to safe defaults (commit to git).
- [ ] Archive `doosan_gesture_demo.log` with the date of the event.
- [ ] Note anything you changed during the day in the lab notebook.
