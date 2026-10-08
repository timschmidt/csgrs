set pagination off
set confirm off
set print frame-arguments none
set print entry-values no
set debuginfod enabled off
break __rustc::rust_panic
run
bt 30
kill
quit
