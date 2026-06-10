# CARKit Vendor Note

This folder contains the F1TENTH control stack previously run by CARKit through the separate `ariiees/ada:foxy-f1tenth` Docker container.

Source:

- Repository: `https://github.com/thecarlab/ada_system`
- Imported path: `src/f1tenth_system`
- Source basis: `main` at `8f724985dd8517f44870b5348cca10a878935bea`

CARKit vendors this code so the joystick, Ackermann mux, and VESC driver stack build inside the single `ariiees/carkit:latest` Docker environment.

Keep upstream license files in place when changing this code.
