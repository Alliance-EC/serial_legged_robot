% VMC解算优化版本，大幅减少计算量
clear;
tic;
syms phi1_t(t) phi2_t(t) phi_dot_1 phi_dot_2;
syms l1 l2 ;
syms L0 phi0;

%1进行几何计算
x_B = l1 * cos(phi1_t);
y_B = l1 * sin(phi1_t);
x_C = x_B + l2 * cos(phi2_t);
y_C = y_B + l2 * sin(phi2_t);

x_dot_B = diff(x_B, t);
y_dot_B = diff(y_B, t);
x_dot_C = diff(x_C, t);
y_dot_C = diff(y_C, t);

% 将表达式中 diff(X,t) 类型的符号变量替换为记作 X_dot 的基本符号变量
x_dot_C = subs(x_dot_C, diff(phi2_t, t), phi_dot_2);
x_dot_C = subs(x_dot_C, diff(phi1_t, t),phi_dot_1);

y_dot_C = subs(y_dot_C, diff(phi2_t, t), phi_dot_2);
y_dot_C = subs(y_dot_C, ...
    [diff(phi1_t, t)], ...
    [phi_dot_1]);
% 求速度雅可比矩阵
x_dot = [x_dot_C; y_dot_C];
q_dot = [phi_dot_1; phi_dot_2];
x_dot = simplify(collect(x_dot, q_dot));
J = simplify(jacobian(x_dot, q_dot));

R = [cos(phi0 - pi / 2) -sin(phi0 - pi / 2);
     sin(phi0 - pi / 2) cos(phi0 - pi / 2)];
M = [0 -1 / L0;
     1 0];
M_v = [1 0;
     0 -1 / L0];
% 动力雅可比矩阵
Tf = J.' * R * M;
Tf = simplify(Tf);

% 逆向动力雅可比矩阵
% % % 用T1,T2反解F、Tp，T_tr=[F,Tp]
syms T1 T2;
T_tr = Tf \ [T1; T2];
T_tr = simplify(T_tr);
disp('几何计算完毕');
syms phi1 phi2;
L0_=sqrt(x_C^2+y_C^2);
phi0_=atan2(y_C, x_C);
theta = simplify(pi / 2 - phi0_);
% 上述是https://zhuanlan.zhihu.com/p/563048952 中1.2.2VMC部分

% 求得腿部姿态
pos_t = [L0_; theta];
pos = formula(pos_t);
pos = subs(pos, [phi1_t(t) phi2_t(t)], [phi1 phi2]);
disp('5%');
% 位置雅可比矩阵
L0_=subs(L0_, [phi1_t(t) phi2_t(t)], [phi1 phi2]);
phi0_=subs(phi0_, [phi1_t(t) phi2_t(t)], [phi1 phi2]);

J11=diff(L0_,phi1);
J12=diff(L0_,phi2);
J21=diff(phi0_,phi1);
J22=diff(phi0_,phi2);
J_p=[J11 J12; J21 J22];
% 求解末端速度
spd=J_p*[phi_dot_1;phi_dot_2];
spd=simplify(spd);
spd=formula(spd);
% 求得VMC转换矩阵
syms F Tp;
Tf = subs(Tf, [L0 phi0], [L0_ phi0_]);
Tf = simplify(Tf);
T_t = Tf * [F; Tp];
T = formula(T_t);
T = subs(T, [phi1_t(t) phi2_t(t)], [phi1 phi2]);

% 反解VMC
T_tr = subs(T_tr, [L0 phi0], [L0_ phi0_]);
T_tr = simplify(T_tr);
T_r = formula(T_tr);
T_r = subs(T_r, [phi1_t(t) phi2_t(t)], [phi1 phi2]);
disp('带入角度计算完毕，储存结果');
save("data/leg_calc.mat");

%% 赋值计算
toc;
clear;
tic;
load("data/leg_calc.mat");
l1_=0.18;l2_=0.18;  %误差0.5mm
pos =subs (pos,[l1 l2],[l1_ l2_]);
spd = subs(spd, [l1 l2], [l1_ l2_]);
T = subs(T, [l1 l2], [l1_ l2_]);
T_r = subs(T_r, [l1 l2], [l1_ l2_]);
matlabFunction(spd, 'File', 'function/spd_calc');
matlabFunction(pos, 'File', 'function/leg_pos');
matlabFunction(T, 'File', 'function/leg_conv');
matlabFunction(T_r, 'File', 'function/leg_conv_reverse');
disp('函数已生成');