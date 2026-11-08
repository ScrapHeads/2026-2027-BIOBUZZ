# Blue Power localization reference

Source: https://github.com/BluePowerRobotics/Team19725_BIOBUZZ

This is a reduced reference-only copy of the supplied Team19725_BIOBUZZ-master archive for ScrapHeads FTC #21485's BIOBUZZ AprilTag localization review. It is outside the Android source roots and is not a standalone buildable project.

Retained under `Team19725_BIOBUZZ-master/`:

- `LICENSE` and upstream `README.md`.
- `fmaps/BIOBUZZ_HiveFlat_Blue.fmap` and `fmaps/BIOBUZZ_HiveFlat_Red.fmap`.
- Under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`:
  - `Parameter/HypParams.java`.
  - `Processors/VisionLocalizer/MT1Localizer.java`, `MT1Localizer.md`, and `OneMap_MT1Localizer.md`.
  - `Processors/FusionLocalizer/AdaptiveEKFLocalizer.java` and `Theory.md`.
  - `Processors/RobotPosition/RobotPosition.java` and `RobotPosition.md`.

The retained files preserve the IDE-visible contents of the extracted copy at cleanup time. The earlier IDE move changed package declarations and imports in that copy; these files are not claimed to be pristine upstream source. Their dependencies were intentionally omitted because they are for study, not compilation. The original ZIP remains the recovery source for omitted files and upstream declarations.

Reference map dimensions, coordinate frames, tag geometry, camera settings, and calibration values must be verified before use. No localization implementation or runtime map uploads are enabled by this reference folder.
