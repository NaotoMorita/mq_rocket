%-----初期姿勢
phi0=0;
theta0=80*pi/180;
psi0=0*pi/180;
q=euler2quat(phi0,theta0,psi0);

Ox0=0;
Oy0=0;
Oz0=0;
P=[Ox0;Oy0;Oz0];

%-----初速度と初期位置
U=[0;0;0];
r=[0;0;0];

dcm=quat2dcm(q);
inv_q=quat_inv(q);
inv_dcm=quat2dcm(inv_q);
v=dcm*U;
w_i=dcm*P;
dt=0.005;
t=0;

%-----風設定
wind=[0;2;0];

%-----推力設定
Total_impulse=2.7;
Tmax=10
tmax=0.1;

%-----慣性モーメントと質量
m=0.0714;
Ix=0.0000198;
Iy=0.001293;
Iz=0.001293;

%-----出力オプション
blen=1.5;
step=50;

make_rand=rand;

%-----ランチャーセッティング
Launcher_length=2;
L_l=dcm*[Launcher_length;0;0];
L_h=L_l(3);

%-----空力系パラメータ
%-----揚力傾斜
CLab=2;
%-----CGtoCP
lp=0.06
%-----正面面積
Sf=(0.023)^2/4*pi;
%-----側面積
Ss=(2.5*42.5)*10^(-4);
%-----正面積抗力係数
CD=0.8
%-----抗力傾斜
CDab=0.008
%-----パラシュート展開時刻
parashoot_time=4
%-----パラシュート面積
Sp=(0.25)^2*pi/4;

i=0;


