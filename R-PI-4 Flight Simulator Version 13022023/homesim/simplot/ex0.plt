set terminal png truecolor font arial 8 size 600,800
set output "ex0.png"
set size 1,1
set origin 0,0
set lmargin 10
set multiplot
set grid
set format y "%5g"
set size 1.0, 0.200000
set style line 1 lw 1 linecolor rgb "blue"
set origin 0, 0.800000
set ylabel "PITCH DEGS"
set xr[0.0:120.000000]
set yr[-15.000000:15.000000]
plot 'ex0.dat' using 1:2 title 'PITCH' with lines
set origin 0, 0.600000
set ylabel "PITCH RATE DEG/S"
set xr[0.0:120.000000]
set yr[-10.000000:10.000000]
plot 'ex0.dat' using 1:3 title 'PITCH RATE' with lines
set origin 0, 0.400000
set ylabel "ALTITUDE FT"
set xr[0.0:120.000000]
set yr[2000.000000:4000.000000]
plot 'ex0.dat' using 1:4 title 'ALTITUDE' with lines
set origin 0, 0.200000
set ylabel "TAS KTS"
set xr[0.0:120.000000]
set yr[160.000000:240.000000]
plot 'ex0.dat' using 1:5 title 'TAS' with lines
set origin 0, 0.000000
set ylabel "ELEVATOR DEGS"
set xr[0.0:120.000000]
set yr[-20.000000:20.000000]
plot 'ex0.dat' using 1:6 title 'ELEVATOR' with lines
unset multiplot
reset
set output
