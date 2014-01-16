1;

%-----quaternion��
function qp=quat_product(q, p)
	q_matrix=[q(1),-q(2),-q(3),-q(4);
			q(2),q(1),-q(4),q(3);
			q(3),q(4),q(1),-q(2);
			q(4),-q(3),q(2),q(1)];
			
	qp=q_matrix*p;
end

%-----quaternion�m�[�}���C�Y
function q_normalized=quat_normalizer(q)
	norm=sqrt(q(1)^2+q(2)^2+q(3)^2+q(4)^2);
	q_normalized=q./norm;
end

%-----�tquaternion
function inv_q=quat_inv(q)
	norm=sqrt(q(1)^2+q(2)^2+q(3)^2+q(4)^2);
	quatd=[q(1);-q(2);-q(3);-q(4)];
	inv_q=quatd./norm;
end

%-----quaternion�����
function q=quat_making(theta,x,y,z)
q(1,1)=cos(theta/2);
q(2:4,1)=sin(theta/2).*[x;y;z];
end

%-----quaternion�̎��Ԑ���
function dquat_dt=deq_quat(quat,ox,oy,oz)
	dquat_dt=1./2.*(quat_product(quat,[0;ox;oy;oz]));
end

%-----quaternion����I�C���[�p�ւ̕ϊ�
function [ptp]=quat2euler(q)
	phi=atan((2*(q(1)*q(2)+q(3)*q(4)))/(1-2*(q(2)^2+q(3)^2)));
	theta=asin(2*(q(1)*q(3)-q(4)*q(2)));
	psi=atan((2*(q(1)*q(4)+q(2)*q(3)))/(1-2*(q(3)^2+q(4)^2)));
	ptp=[phi;theta;psi];
end	

%----quaternion��������]���s��ւ̕ϊ�
function E=quat2dcm(q)
	E(1,1)=q(1)^2+q(2)^2-q(3)^2-q(4)^2; E(1,2)=2*(q(2)*q(3)-q(1)*q(4)); E(1,3)=2*(q(1)*q(3)+q(2)*q(4));
	E(2,1)=2*(q(2)*q(3)+q(1)*q(4)); E(2,2)=q(1)^2-q(2)^2+q(3)^2-q(4)^2; E(2,3)=2*(q(3)*q(4)-q(1)*q(2));
	E(3,1)=2*(q(2)*q(4)-q(1)*q(3)); E(3,2)=2*(q(1)*q(2)+q(3)*q(4)); E(3,3)=q(1)^2-q(2)^2-q(3)^2+q(4)^2;
end

%----�I�C���[�p����quaternion�ւ̕ϊ�
function quat=euler2quat(phi,theta,psi)
	q(1)=cos(phi/2)*cos(theta/2)*cos(psi/2)+sin(phi/2)*sin(theta/2)*sin(psi/2);
	q(2)=sin(phi/2)*cos(theta/2)*cos(psi/2)-cos(phi/2)*sin(theta/2)*sin(psi/2);
	q(3)=cos(phi/2)*sin(theta/2)*cos(psi/2)+sin(phi/2)*cos(theta/2)*sin(psi/2);
	q(4)=cos(phi/2)*cos(theta/2)*sin(psi/2)-sin(phi/2)*sin(theta/2)*cos(psi/2);
	
	quat=q';
end

%-----�����]���s�񂩂�quaternion�ւ̕ϊ�


p=quat_making(90/180*pi,0,0,1);
p=quat_normalizer(p);

