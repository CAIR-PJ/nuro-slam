# Nuro-SLAM Single-Robot Event-Based Benchmark

## Scope

This directory contains a SHA256-verified copy of the reviewed
Nuro-SLAM benchmark framework and its historical configurations.

The benchmark supports the evaluation of event-based SLAM
pipelines involving CMax-SLAM, EROAM, EMBA and EPBA.

## Environment

Original development environment:

- macOS Apple Silicon ARM64
- ROS 2 Jazzy (Homebrew)
- Gazebo Harmonic
- Homebrew OpenCV
- Python and native C++ SLAM executables

The copied configurations retain historical absolute paths.
They are not portable without environment-specific adaptation.

## Benchmark execution

The benchmark runners support explicit execution options.

The historical manifest uses:

    execution_enabled: false

This is a required configuration invariant in the original
framework, not an unconditional prohibition on execution.

Do not invoke benchmark execution without reviewing commands,
input datasets, output directories and resource limits.

No benchmark is executed by this integration procedure.

## Phase II command manifest

The included phase2_command_manifest.json contains 18 planned
jobs with unconfigured argv arrays.

It is retained as historical planning evidence and is NOT
a complete executable reproduction specification.

## Scientific evidence

Prior Phase II Playroom SO(3) validation reported 18 successful
trials across six configurations and three repetitions.

These results support internal numerical consistency of the
tested configurations.

They do not establish ground-truth trajectory accuracy,
ATE/RPE, mapping accuracy or cross-algorithm superiority.

The scientific compatibility matrix is provisional and
contains historical readiness assessments.

## Source integration

The benchmark source files were copied without modification
and verified against the development workspace using SHA256.

The original benchmark workspace and generated outputs
remain outside the Git repository.

## Limitations

- Dataset provenance requires independent verification.
- Ground-truth alignment remains unresolved.
- Historical absolute paths require adaptation.
- Energy measurement methodology needs validation.
- Full end-to-end reproducibility is not yet established.
- ROS launch and native executable availability depend on
  the target installation.

## Repository policy

Do not commit raw datasets, full benchmark logs, trajectories,
large plots, generated archives or temporary build outputs.

Changes to benchmark execution or scientific conclusions
require separate validation.
