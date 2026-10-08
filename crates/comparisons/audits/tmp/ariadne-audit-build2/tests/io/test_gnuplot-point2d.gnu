unset key
set terminal pngcairo nocrop size 800, 800
set output "test_gnuplot-point2d.png"
set multiplot
set xlabel 'x0'
set ylabel 'x1'
set xrange [0:5] 
set yrange [0:5] 
set style fill transparent solid 1
plot "<echo '2 4'" w points pt 7 ps 2
quit
