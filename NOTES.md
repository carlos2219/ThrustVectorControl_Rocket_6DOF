# Working notes

## cwd dependency: matl.m / rocket_upwork InitFcn (not fixed, just recorded)

`rocket_upwork.slx`'s InitFcn is `matl;` (bare script call, no path). This only
resolves if MATLAB's current working directory is `UMUT/` at load/update time,
since that's where `matl.m` lives (and `matl.m` itself calls `lqr_gain_design;`
the same bare way, same constraint). Confirmed live: running
`UMUT/matl.m` first (which sets cwd to `UMUT/`), then `Update Diagram` on
`rocket_upwork`, succeeds cleanly. If cwd is anything else when the model is
opened/updated, InitFcn silently fails to find `matl`/`lqr_gain_design` and the
`rocket` struct never gets built, which then cascades into `Invalid setting for
parameter Value` errors on every Constant block that reads `rocket.*`.

Not fixed yet. Options for later: make InitFcn resolve `matl.m`'s path
explicitly (e.g. relative to the model file's own folder), or move `matl.m`
up to the project root. Deferred until after the x_cg/rocket_forces_moments
integration is settled, since it touches `rocket_upwork.slx` itself.
