unset key
set terminal pngcairo nocrop size 800, 800
set output "test_gnuplot-LabelledFigure-Gauss3DProjXZ.png"
set multiplot
set xlabel 'x'
set ylabel 'z'
set xrange [0:19] 
set yrange [0:1] 
set style fill transparent solid 1
quit
