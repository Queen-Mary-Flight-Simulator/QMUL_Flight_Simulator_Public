#gnuplot script for gold.dat 

set title "frame timing"
set xlabel "frame no."
set ylabel "t usec"
set xr [0.0:11500.0]
set yr [0.0:50000.0]
set terminal png
set output "timing.png"
plot "timing.dat" using 1:2 title 'y(t)' with lines
reset
set output
