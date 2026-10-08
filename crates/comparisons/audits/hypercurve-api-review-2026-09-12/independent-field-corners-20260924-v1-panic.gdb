set pagination off
set confirm off
set print frame-arguments none
set print entry-values no
break rust_panic
run
bt 30
kill
quit
