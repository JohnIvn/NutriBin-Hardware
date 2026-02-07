# Contributing to NutriBin-Hardware

Thank you for your interest in contributing! We welcome improvements, bug reports, and new hardware/firmware ideas — your help makes this project better.

## How to contribute

- Discuss big changes or feature ideas by opening an issue first.
- For bug reports, please include:
  - Hardware version (e.g., ESP32, Arduino R3).
  - Steps to reproduce the issue.
  - Relevant serial logs or error messages.
- When you're ready to contribute code, open a Pull Request (PR) with a clear title and description.

## Development setup

This project consists of Arduino and ESP32 firmware.

### Prerequisites

1.  **Arduino IDE** (or Arduino CLI / VS Code with Arduino Extension).
2.  **ESP32 Board Support**:
    - Add this URL to "Additional Boards Manager URLs": `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
    - Install the `esp32` platform via the Boards Manager.

### Compiling

Select the appropriate board (e.g., `ESP32 Dev Module`) and target the specific sketch in the `Hardware/` directory:

- `Hardware/ESP_AP/`
- `Hardware/ESP_CAM/`
- etc.

## Branching & PR guidelines

- Create a feature branch from `main`: `feature/short-description`.
- Keep commits focused and use clear commit messages.
- Ensure your changes compile successfully before submitting.

## Code style

- Follow standard C++/Arduino coding conventions.
- Keep the code readable and well-commented, especially for hardware interactions (Pin definitions, sensor communication).
- Use descriptive variable names.

## Licensing and CLA

By contributing you agree that your contributions will be licensed under the project's existing license.

## Code of conduct

Please follow the project's [Code of Conduct](CODE_OF_CONDUCT.md). Respectful, inclusive behavior is expected.

Thanks again — contributions are appreciated!

## Pull Request checklist & template

Before opening a Pull Request (PR), please make sure your changes meet the checklist below:

- [ ] I opened an issue describing the change (for non-trivial fixes).
- [ ] My branch is up-to-date with `main`.
- [ ] My code compiles without errors for the target board.
- [ ] I updated the README if hardware pins or dependencies changed.

Suggested PR description template:

```
Title: [Fix/Feature] Short description

Description:
- What changed and why.
- Which hardware boards were tested (e.g. ESP32, Arduino Uno).

Testing:
- Summary of serial output or physical test results.
```

Thanks — a clear PR description and checklist speeds up review and improves project quality.
