set pagination off
set confirm off
set debuginfod enabled off
set print frame-arguments none
set print entry-values no
set $chamfer_calls = 0
break _RNvMsj_NtCs3xD80Wu1NFK_10hypercurve13bezier_regionNtB5_12CurveRegion231chamfer_loop_vertex_by_setbacks
commands 1
silent
set $chamfer_calls = $chamfer_calls + 1
printf "chamfer-call=%u\n", $chamfer_calls
continue
end
run
thread apply all bt 12
kill
quit
