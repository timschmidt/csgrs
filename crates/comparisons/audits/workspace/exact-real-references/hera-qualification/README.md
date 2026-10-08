# HERA qualification artifacts

These files are audit probes and result logs, not part of the 14-file original
HERA source inventory. See ../HERA_FILE_NOTES.md for scope, compatibility
changes, exact oracle contracts, benchmark interpretation and transfer closure.

The original archive is unchanged. GHC 9.6.7 and MPFR 4.2.2 were used. Historical
compiler warnings remain. Build commands, the compatibility copy and frozen
executables are under /tmp/hera-audit.UOikl9; hera-compat-source.diff records
the numerical-code-preserving build changes. The cache variant changes only
the original last-request equality test to >=. The full-radius benchmark is
explicitly a policy reconstruction, not an additional native donor API.

The timed binaries were built -O2 -fno-full-laziness -fno-cse -rtsopts and run
with +RTS -T. Both shell drivers pin CPU 6 and alternate variant order. Raw
allocation columns are cumulative managed bytes, not peak RSS. Read the result
logs as numerical findings: an exit-zero probe may report failed assertions as
data. finite-sum*.log are empty because both commands reached their three-second
timeout (exit 124), not because they passed.
