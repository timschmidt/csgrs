unset key
set terminal pngcairo nocrop size 800, 800
set output "test_gnuplot-curve.png"
set multiplot
set xlabel 'x0'
set ylabel 'x1'
set xrange [0:1] 
set yrange [0:1] 
set style fill transparent solid 1
plot '-' w lines lw 1 lc rgb "#000000"
0 0
0.1 0.01
0.2 0.04
0.3 0.09
0.4 0.16
0.5 0.25
0.6 0.36
0.7 0.49
0.8 0.64
0.9 0.81
1 1
e
quit
