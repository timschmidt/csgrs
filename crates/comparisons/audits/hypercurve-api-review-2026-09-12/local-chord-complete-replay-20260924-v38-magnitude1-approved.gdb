set pagination off
set confirm off
set debuginfod enabled off
set print frame-arguments none
set print entry-values no
set $chamfer_calls = 0
break _RNvMs1j_NtCs211r7KXB5U9_10hypercurve13bezier_offsetNtB6_43BezierRecursiveQuadraticParallelExpression228squared_magnitude_difference
commands 1
silent
set $chamfer_calls = $chamfer_calls + 1
if $chamfer_calls <= 12
printf "magnitude-call=%u\n", $chamfer_calls
bt 5
end
continue
end
run
printf "total-magnitude-calls=%u\n", $chamfer_calls
thread apply all bt 12
kill
quit
