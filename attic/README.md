# attic

Code that is in the repository but in no build.

Everything here was found by checking each `.c` in the old flat root against
the `SRCS` list in `make/Makefile`: these are the files that were not in it,
and that nothing else includes either. They compiled the day they were written
and have been carried along since, so it is not obvious from a directory
listing which files are live. That is the whole reason for this directory —
`src/` is what ships, `attic/` is what does not.

Nothing here has been modified; the files were moved with `git mv`, so
`git log --follow` still reaches their history.

| file | superseded by |
| --- | --- |
| `context_menu.c/.h` | `src/ui/menu_widget.c` |
| `menu_system.c/.h` | `src/ui/menu_widget.c` — only `context_menu.c` used it |
| `doom_complete_port.c/.h` | `src/apps/doom_port.c` |
| `doom_embedded.c/.h` | `src/apps/doom_port.c` |
| `doom_full_port.c/.h` | `src/apps/doom_port.c` |
| `doom_stub.c` | `src/apps/doom_port.c` |
| `test_coredump.c/.h` | `tests/coredump_test.c`, which runs on the host |

Four DOOM ports are here because the port was arrived at by attempts, and the
attempts were kept. `src/apps/doom_port.c` is the one that runs.

If none of this is worth keeping, `git rm -r attic` — the history keeps it
either way. It was quarantined rather than deleted because deciding that a
file is finished with is a judgement call, and the reorganisation that moved
it was not the moment to make it.
