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

### Refactoring backlog (2026-08 tech-debt pass; deliberately deferred, with why)

- [ ] `src/acq/trigger.c`: six ~75%-identical hand-written asm scanners
      (edge x channels). Needs a host-test harness first - unverifiable today,
      and hot inline asm is not a thing to dedupe blind.
- [ ] `config.c` FMC block: the register driver shares its state machine with
      the store's health bookkeeping (busy flags, retries, coredump report).
      Extracting `src/hal/fmc.c` means redesigning that boundary - do it only
      with the store's failure modes in front of you.
- [ ] `config.c` v1->v2 migration: shares `g_old_layouts` with the always-on
      calibration adoption path; not a clean cut either.
- [ ] `capture.c` sinc/nl math into `src/dsp/`: sinc_between() rides on
      BufferInfo's ring+window clamp; flattening the signature trades 4 args
      for 8. Worth it only together with a host test for the resampler.
- [ ] `src/apps/` game_common: the init/record_best/frame-tick copies have
      drifted behaviorally (2048 writes config through, snake/flappy defer,
      tetris never persists). Unify only with per-game decisions in hand;
      tetris's missing record_best is a behavior change to make on purpose.
- [ ] 3D types: `camera_t` in engine3d.h and raytrace_test.c are same-name
      DIFFERENT structs; only vec3_t/ray_t match. Unifying means renaming one
      camera - small API break, decide when touching those apps anyway.
- [ ] `logic_decode.h` split into per-protocol headers (26 includers rebuild
      on any struct change).
- [ ] `scope_` prefix pass for the generic internal names now exported by
      scope_internal.h (update_display, toast_show, text_width, ...).
- [ ] `utils.h` assumes CMSIS: drops the 38k-line vendor header into 10 of 11
      apps; fixing it frees every game from gd32f4xx.h.
- [ ] tests/host_test.c (9.7k lines) split per protocol, like the game tests.
- [ ] pd_decode trim loop keeps the LAST pass, not the best one - res.count
      flaps frame to frame on marginal captures (diagnosed 2026-08-08 during
      the decode-marks fix; keep-best is a decoder behavior change, verify
      against a real PD capture).

## Docs

Documentation this project.

- [ ] add more pictures with work interface and something.
- [ ] more docs for programming mode.

- [x] Make todolist/changelog
