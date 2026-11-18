# Reusable ATLAS rover

`model.sdf` owns chassis/wheel mechanics, drive control and simulation truth.
`definition.json` exports its base frame, truth/command interfaces and binds
keyboard-control parameters to the SDF controller values. `control/` and
`meshes/` belong to this rover implementation. No sensor instances or world
selection belong here; the assembler reads those separately.

The truth TF is for simulation visualization, not an estimator output.
