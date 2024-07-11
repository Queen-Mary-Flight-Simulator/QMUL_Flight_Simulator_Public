set terminal png truecolor font arial 8 size 600,800
set output "ex3.png"
set size 1,1
set origin 0,0
set lmargin 10
set multiplot
set grid
set format y "%5g"
set size 1.0, 0.250000
set style line 1 lw 1 linecolor rgb "blue"
set origin 0, 0.750000
set ylabel "YAW DEGS"
set xr[0.0:60.000000]
set yr[0.000000:50.000000]
plot 'ex3.dat' using 1:2 title 'YAW' with lines
set origin 0, 0.500000
set ylabel "YAW RATE DEG/S"
set xr[0.0:60.000000]
set yr[-2.000000:4.000000]
plot 'ex3.dat' using 1:3 title 'YAW RATE' with lines
set origin 0, 0.250000
set ylabel "BETA DEGS"
set xr[0.0:60.000000]
set yr[-5.000000:5.000000]
plot 'ex3.dat' using 1:4 title 'BETA' with lines
set origin 0, 0.000000
set ylabel "RUDDER DEGS"
set xr[0.0:60.000000]
set yr[-20.000000:20.000000]
plot 'ex3.dat' using 1:5 title 'RUDDER' with lines
unset multiplot
reset
set output
