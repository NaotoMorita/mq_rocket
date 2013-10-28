1;
%-----推力
function T=thrust(t,Tmax,tmax,grad)
	if t<=tmax
		T=Tmax/tmax*t;
	else
		T=Tmax*exp(-grad*(t-tmax));
	end
end

if U(1)!=0;	
	alpha=-asin(U(3)/sqrt(U(1)^2+U(2)^2+U(3)^2))*U(1)/abs(U(1));
	beta=-atan(U(2)/U(1))*U(1)/abs(U(1));
	
else
	alpha=0;
	beta=0;
end	

%-----速度座標系へ
if sqrt(U(1)^2+U(2)^2+U(3)^2)!=0
	xd=U./sqrt(U(1)^2+U(2)^2+U(3)^2);
	yd=cross(xd+[0;0;10],xd);
	yd=yd./sqrt(yd(1)^2+yd(2)^2+yd(3)^2);
	zd=cross(xd,yd);
	zd=zd./sqrt(zd(1)^2+zd(2)^2+zd(3)^2);
	inv_dcm_b_v=[xd,yd,zd];
	dcm_b_v=inv([xd,yd,zd]);
else
	dcm_b_v=eye(3);
	inv_dcm_b_v=eye(3);
end


Uv=dcm_b_v*U;
%-----揚力
function L=lift(Uv,S,CLab,alpha,beta,L_h,r)
	rho=1.184;
	if Uv(1)==0
		L(1:3,1)=0;
	else
		L(2,1)=1/2*rho*Uv(1)^2*S*CLab*beta;
		L(3,1)=1/2*rho*Uv(1)^2*S*CLab*alpha;
	end
	if -r(3)<=-L_h 
		L(:,1)=0;
	end
end

	
%-----抗力
function D=drag(Uv,Sf,Ss,Sp,CD,CDab,parashoot_time,t,alpha,beta,L_h,r)
	rho=1.184;
	if Uv(1)==0
		D(1,1)=0;
		D(3,1)=0;
	else
		D(1,1)=1/2*rho*Uv(1)^2*Sf*CD;
		if -r(3)>=-L_h 
		D(1,1)+=(1/2*rho*Uv(1)^2*Ss*CDab*abs(beta)+1/2*rho*Uv(1)^2*Ss*CDab*abs(alpha));
		end
		D(3,1)=0;
	end
	
	if t>=parashoot_time
		D(1,1)=1/2*rho*Uv(1)^2*1.3*Sp;
		D(2,1)=0;
		D(3,1)=0;
	end
end


if i==1;
%-----力積から推力減衰の係数を求める
	function res=ti_grad(Tmax,tmax,grad,Total_impulse)
		res=quad(@(t)thrust(t,Tmax,tmax,grad),0,20)-Total_impulse;
	end
grad=fsolve(@(grad)ti_grad(Tmax,tmax,grad,Total_impulse),50);
end
	
	
Traw=thrust(t,Tmax,tmax,grad);
T=[Traw;0;0];
%T=[Traw+0.01*(rand-0.5)*Traw;0.01*(rand-0.5)*Traw;0.01*(rand-0.5)*Traw];
L=lift(Uv,Ss,CLab,alpha,beta,L_h,r);
D=drag(Uv,Sf,Ss,Sp,CD,CDab,parashoot_time,t,alpha,beta,L_h,r);
%M=moment(L,D,lp);
%M_PID=roll_PID(0,P(1),Uv,Sfin,CLafin,Kp,Kd,Ki,save_parameter);
%M+=[+0.01*lp*(rand-0.5)*Traw;0.01*(rand-0.5)*Traw*lp;0.01*lp*(rand-0.5)*Traw];


	
%-----速度座標系の力から機体座標系へ
F_v=[-D(1)+L(1);-D(2)+L(2);-D(3)+L(3)];
F_e=inv_dcm_b_v*F_v;
F_e+=T;

M_e=[0;
	F_e(3)*lp;
	-F_e(2)*lp];
	


