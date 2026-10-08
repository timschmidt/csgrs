unset key
set terminal pngcairo nocrop size 800, 800
set output "test_gnuplot-box.png"
set multiplot
set xlabel 'x0'
set ylabel 'x1'
set xrange [0:5] 
set yrange [1:4] 
set style fill transparent solid 1
plot '-' w filledcurves fc rgb "#FFBF80" fs solid 1 border lc rgb "#000000"
1 2
4 2
4 3
1 3
1 2
e
quit
