t=0.000001:0.00000001:0.00005;
s = 157./(t*6400);
# plot(t,s);
# ylabel("Speed(mm/s)");
# xlabel("Time(s)");
# hold on;

S = 0:613;
n = (1218.75./S) - 2;
plot(n,S);
ylabel("Speed(mm/s)");
xlabel("Number(n)");
