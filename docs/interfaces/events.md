# Event Identifiers

The cFE event (EVS) identifiers of the OrthoStream applications
(ADR-020 section 3).

* Each application defines its identifiers in
  `apps/<app>/fsw/inc/<app>_eventids.h`.
* Identifiers are never reused or renumbered.
* `tests/interfaces/check_interfaces.py` checks that the identifiers of each
  application are unique, lie in their range and are listed here.

## Ranges

| Range | Use |
| --- | --- |
| 1–19 | life cycle and commands: initialisation, no-op, reset, command errors, pipe errors, table load and validation |
| 20–59 | data path: state changes, dropped items with their reason, rejected poses or batches |
| 60–79 | fault changes, raised by the fault set of `ost_common`: identifier 60 + index of the fault in the application's list |
| 80–99 | library diagnostics: 80 debug, 81 warning, 82 error |

## Types

* `CRITICAL`: the application cannot continue its function.
* `ERROR`: an item or a request failed.
* `INFORMATION`: state changes and command acknowledgements.
* `DEBUG`: diagnostics, disabled by default.

Events are raised on changes of state, not per item. Per-item counts go to
housekeeping. Events that can repeat are limited with EVS binary filters.

## Applications

The sections below are added with the applications. Each lists the
identifiers of `<app>_eventids.h` in a table with the columns Id, Name, Type
and Meaning; the fault events list the application's faults in order.
