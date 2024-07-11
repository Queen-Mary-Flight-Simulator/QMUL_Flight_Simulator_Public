set terminal png truecolor font arial 8 size 600,800
set output "ex2.png"
set size 1,1
set origin 0,0
set lmargin 10
set multiplot
set grid
set format y "%5g"
set size 1.0, 0.250000
set style line 1 lw 1 linecolor rgb "blue"
set origin 0, 0.750000
set ylabel "PITCH DEGS"
set xr[0.0:10.000000]
set yr[0.000000:20.000000]
plot 'ex2.dat' using 1:2 title 'PITCH' with lines
set origin 0, 0.500000
set ylabel "PITCH RATE DEG/S"
set xr[0.0:10.000000]
set yr[-8.000000:8.000000]
plot 'ex2.dat' using 1:3 title 'PITCH RATE' with lines
set origin 0, 0.250000
set ylabel "ALPHA DEGS"
set xr[0.0:10.000000]
set yr[-4.000000:12.000000]
plot 'ex2.dat' using 1:4 title 'ALPHA' with lines
set origin 0, 0.000000
set ylabel "ELEVATOR DEGS"
set xr[0.0:10.000000]
set yr[-20.000000:20.000000]
plot 'ex2.dat' using 1:5 title 'ELEVATOR' with lines
unset multiplot
reset
set output
