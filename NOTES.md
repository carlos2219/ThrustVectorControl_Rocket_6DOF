# Working notes

## cwd dependency: matl.m / rocket_upwork InitFcn (not fixed, just recorded)

`rocket_upwork.slx`'s InitFcn is `matl;` (bare script call, no path). This only
resolves if MATLAB's current working directory is the PROJECT root at
load/update time, since that's where `matl.m` now lives (and `matl.m` itself
calls `lqr_gain_design;` the same bare way, same constraint). If cwd is
anything else when the model is opened/updated, InitFcn silently fails to
find `matl`/`lqr_gain_design` and the `rocket` struct never gets built, which
then cascades into `Invalid setting for parameter Value` errors on every
Constant block that reads `rocket.*`.

**Update (2026-08-26):** `UMUT/` was removed and its files (`matl.m`,
`lqr_gain_design.m`, `rocket_upwork.slx`, the thrust CSVs) moved up to the
PROJECT root, per client preference for a single project folder. This moves
the required cwd from `UMUT/` to the PROJECT root but does NOT fix the
underlying fragility - InitFcn is still a bare, path-less `matl;` call.
Verified working from the new root location (`open_system` + `Update
Diagram` + a 40s `sim()` all succeeded with cwd = PROJECT root).

Still not fixed. Option for later: make InitFcn resolve `matl.m`'s path
explicitly (e.g. relative to the model file's own folder via
`fileparts(get_param(bdroot,'FileName'))`), so it stops depending on
whatever MATLAB's cwd happens to be at load time.
