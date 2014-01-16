clear all
quaternion2


%-----パラメータ読込
rocket_parameter

do 
i++;
t+=dt;

%-----ロケットにかかる力を求める
rocket_Force_Moment

%-----機体座標系から慣性座標系へ
dcm=quat2dcm(q);
F_i=dcm*F_e;
M_i_I=dcm*(M_e./[Ix;Iy;Iz]);
%-----重力加速度を足す
F_i+=[0;0;9.8*m];

%-----ランチャーから離れる前
if -r(3)<=-L_h 
	F_e=inv_dcm*F_i;
	F_e(2:3,1)=0;
	F_i=dcm*F_e;
	if F_i(3,1)>0
		F_i(:,1)=0;
	end
end
	
%-----慣性座標系で積分
if i==1
	v+=F_i./m.*dt;
	r+=v.*dt;
	sub_F_i=F_i;
	sub_v=v;
else
	v+=(F_i+sub_F_i)./2./m.*dt;
	r+=(v+sub_v)./2.*dt;
	sub_F_i=F_i;
	sub_v=v;
end
%-----慣性座標系でのモーメント
w_i+=M_i_I.*dt;

inv_q=quat_inv(q);
inv_q=quat_normalizer(inv_q);
inv_dcm=quat2dcm(inv_q);

P=inv_dcm*w_i;

%-----風の影響を足し、機体座標系へ
U=inv_dcm*(v+wind);

if -r(3)>=-L_h 
	q+=deq_quat(q,P(1),P(2),P(3)).*dt;
	q=quat_normalizer(q);
end
	
%ログ系
log_T(:,i)=T;
log_D(:,i)=D;
log_L(:,i)=L;
log_F_i(:,i)=F_i;
log_F_e(:,i)=F_e;
log_F_v(:,i)=F_v;
log_t(:,i)=t;
log_r(:,i)=r;
log_Uv(:,i)=Uv;
log_q(:,i)=q;
log_v(:,i)=v;
log_eu(:,i)=quat2euler(q);
log_dcm{i}=dcm;


if sum(isnan(M_e))!=0 
	break;
end
until(-r(3)<=-L_h&&v(3)>0&&i>=10)

rocket_graph
