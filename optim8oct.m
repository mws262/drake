dims = 2;
pts = 64 - 7;
pinned_spacing = 8;
segs = pts - 1;

syms g h i j m x t_p;
a = sym('a', [segs, 4]); % Trajectory polynomial coefficients. Cubic.
b = sym('b', [segs, 4]); % Trajectory polynomial coefficients. Cubic.

k = sym('k', [pts, 1]);
tst = [0:pts - 2]'./5;%sym('t', [pts, 1]);
te = [1:pts - 1]'./5;%sym('t', [pts, 1]);

t_cubic = [t.^3, t.^2, t, ones(size(t))];

poly_eval_fcn = @(t, coeff)(coeff(:,1) .* t.^3 + coeff(:,2) .* t.^2 + coeff(:,3) .* t + coeff(:,4));
poly_dt_fcn = @(t, coeff)(3 * coeff(:,1) .* t.^2 + 2 * coeff(:,2) .* t + coeff(:,3));
poly_ddt_fcn = @(t, coeff)(6 * coeff(:,1) .* t + 2 * coeff(:,2));

xst = poly_eval_fcn(tst, a);
xe = poly_eval_fcn(te, a);
vxst = poly_dt_fcn(tst, a);
vxe = poly_dt_fcn(te, a);
axst = poly_ddt_fcn(tst, a);
axe = poly_ddt_fcn(te, a);

yst = poly_eval_fcn(tst, b);
ye = poly_eval_fcn(te, b);
vyst = poly_dt_fcn(tst, b);
vye = poly_dt_fcn(te, b);
ayst = poly_ddt_fcn(tst, b);
aye = poly_ddt_fcn(te, b);

start_x = [xst(1) == 0; vxst(1) == 0; axst(1) == 0];
end_x = [xe(end) == 0; vxe(end) == 0; axe(end) == 0];

start_y = [yst(1) == 0; vyst(1) == 0; ayst(1) == 0];
end_y = [ye(end) == 0; vye(end) == 0; aye(end) == 0];

accel_generic_x = poly_ddt_fcn(t_p, a);
accel_generic_y = poly_ddt_fcn(t_p, b);
cost_integral = 0;
for i = 1:size(tst,1)
  cost_integral = cost_integral + int(accel_generic_x(i), t_p, tst(i) ,te(i));
  cost_integral = cost_integral + int(accel_generic_y(i), t_p, tst(i) ,te(i));
end

x_constraints = [
start_x;
end_x;
xe(1:end - 1) == xst(2:end);
vxe(1:end - 1) == vxst(2:end);
axe(1:end - 1) == axst(2:end)
];

y_constraints = [
start_y;
end_y;
ye(1:end - 1) == yst(2:end);
vye(1:end - 1) == vyst(2:end);
aye(1:end - 1) == ayst(2:end)
];

x_coeffs = reshape(a.',numel(a),1);
y_coeffs = reshape(b.',numel(b),1);

all_constraints = [x_constraints; y_constraints];
all_coeffs = [x_coeffs; y_coeffs];

[A_cost, B_cost] = equationsToMatrix(cost_integral);
A_cost_num = eval(A_cost);

[A,B] = equationsToMatrix(all_constraints, all_coeffs);
A_num = eval(A);
B_num = eval(B);


[sol, fval] = linprog(A_cost', [],[],A_num, B_num)

x_sol = sol(1:length(sol)/2);
y_sol = sol(length(sol)/2 + 1:end);

figure;
plot(0,0);
hold on;
segnum = 1;
for i = 1:4:length(x_sol)
    tspan = linspace(tst(i), te(i), 10);
    xseg = x_sol(i).*tspan.^3 + x_sol(i + 1).*tspan.^2 + x_sol(i + 2).*tspan + x_sol(i + 3)*ones(size(tspan));
    yseg = y_sol(i).*tspan.^3 + y_sol(i + 1).*tspan.^2 + y_sol(i + 2).*tspan + y_sol(i + 3)*ones(size(tspan));
    plot(xseg, yseg);
    segnum = segnum + 1;
end
