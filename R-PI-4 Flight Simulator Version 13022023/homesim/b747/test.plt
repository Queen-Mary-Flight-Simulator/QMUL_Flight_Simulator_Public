#gnuplot script for timer1.c

set title "timing"
set xlabel "test"
set ylabel "msec"
set xr [0.0:1000.0]
set yr [0.0:20.0]
set ytics 1.0
set terminal png
set output "test.png"
plot 'test.dat' using 1:2 with lines, \
     'test.dat' using 1:3 with lines
