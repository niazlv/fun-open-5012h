# Todolist. Changelog(?)

Need to write something here.

## Functional

Needs to finish with priority

- [ ] make menu bar
- [ ] when enter into programming mode, show warning and fill screen with text aka "DFU Mode.\n Please reboot to exit."
- [ ] make a classic view to 200ms and more(dont lag).
- [ ] fix auto mode.
- [ ] make reasignble buttons into settings.
- [ ] AM detection: `src/dsp/am.c` is written and host-tested, but is NOT in
      the firmware build - its buffers are 2984 bytes more TCM than the image
      has (there are 224 bytes of slack before it). See the header of
      `src/dsp/am.h` for the measurements and for what would have to free up.
      When it lands it should go through the signal classifier rather than
      take a measurements cell of its own.

## Chore

Something need to others

- [ ] normal build and run config(vscode) on all platforms

## Docs

Documentation this project.

- [ ] add more pictures with work interface and something.
- [ ] more docs for programming mode.

- [x] Make todolist/changelog
