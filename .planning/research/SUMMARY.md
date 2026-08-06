# Research Summary

**Date:** 2026-08-06

## O2 flow

PDF specifies 240 seconds, randomized shelf cargo positions, four block types mapped to Samsung/Foxconn/Amkor/Hana Micron Vina, pallet lifting, 250 x 250 mm containment, level pallet placement, and task 2 only after task 1 completion.

## Vision design

Camera geometry is fixed to the fork: it points straight at two pallets on the elevator's current shelf level. Camera FOV covers one block. `MULTI_TARGET` first classifies the right and left blocks during scan passes. Then `SINGLE_TARGET` tracks the left pallet/template candidate, preserves raw `TM_CCOEFF_NORMED` confidence, calculates template center and `dxPx`/`dyPx`, rejects weak matches, and applies lightweight temporal stabilization while the robot strafes to center the camera on the left pallet. Do not normalize the result matrix after matching because that destroys absolute confidence.

`MULTI_TARGET` should combine two sequential observations: robot strafes right and reads right block, then strafes left and reads left block. Each one-block scan evaluates block templates/class candidates, then the controller stores both labels, centers, and confidence before calculating the lateral midpoint for the shared fork. Return ranked detections with class label, center, box, and raw confidence. This prevents `minMaxLoc` from hiding other blocks.

## FTC constraints

FTC `HardwareMap` names are case-sensitive. Hardware should initialize during OpMode init before start. Blocking drive/elevator/camera actions need timeout and stop handling. Endstop is physical travel protection; homed step count is logical elevator reference.

## Recommended flow

`INIT → HOME_ELEVATOR → NAVIGATE_SOURCE → MULTI_SCAN_RIGHT → MULTI_SCAN_LEFT → SELECT_LEFT_TARGET → SINGLE_TRACK_LEFT → STRAFE_TO_LEFT_CENTER → DRIVE_FORWARD → CONFIRM_LEFT_IR_AND_RIGHT_IR → DUAL_PICKUP → NAVIGATE_FACTORY → PLACE → VERIFY → NEXT_OR_TASK2 → SAFE_STOP`.

## References

- Supplied PDF: `C:\Users\phamd\Downloads\1779357526197-200637899-20-30.pdf`
- [OpenCV Java template matching](https://github.com/opencv/opencv/blob/4.x/samples/java/tutorial_code/ImgProc/tutorial_template_matching/MatchTemplateDemo.java)
- [Template matching and NMS](https://inference.roboflow.com/workflows/blocks/template_matching/)
- [FTC OpMode docs](https://ftc-docs.firstinspires.org/en/latest/programming_resources/tutorial_specific/android_studio/creating_op_modes/Creating-and-Running-an-Op-Mode-%28Android-Studio%29.html)
- [FTC Robot Controller SDK](https://github.com/FIRST-Tech-Challenge/FTCRobotController)
