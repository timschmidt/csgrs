set pagination off
set confirm off
set debuginfod enabled off
set auto-load python-scripts off
set print thread-events off
set startup-with-shell off
break hypersolve_diagonal_pilot::curve_resultant::exact_polynomial_square_root
commands
silent
printf "AUDIT_SQUARE_ROOT_HIT\n"
continue
end
run probe
