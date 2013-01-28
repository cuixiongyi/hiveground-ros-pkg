function cubic_plot(a, t)

hold off
plot(t, a(1) + a(2)*t + a(3)*(t.^2) + a(4)*(t.^3));
hold on
plot(t, a(2) + 2*a(3)*(t) + 3*a(4)*(t.^2));
plot(t, 2*a(3) + 6*a(4)*(t));
hold off


endfunction
