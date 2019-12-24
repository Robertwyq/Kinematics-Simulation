%% ������ѧ����ҵ ��е���˶��滮 3160103829 ������
% 1.�����ؽڵı�׼DH������
clear all
clear;
clc;
%DH������     th    d     a    alpha   type
L(1) = Link([ 0     13    0    -pi/2    0]);%��ת�ؽ�
L(2) = Link([ 0     0     8    0        0]);%��ת�ؽ�
L(3) = Link([ pi/2  -5    0    -pi/2    0]);%��ת�ؽ�
L(4) = Link([ 0     8     0    pi/2     0]);%��ת�ؽ�
L(5) = Link([ pi/2  0     0    -pi/2    0]);%��ת�ؽ�
L(6) = Link([ 0     6     0    0        0]);%��ת�ؽ�
%������������
L(1).qlim = [0 330]*pi/180;
L(2).qlim = [0 310]*pi/180;
L(3).qlim = [0 360]*pi/180;
L(4).qlim = [0 360]*pi/180;
L(5).qlim = [0 360]*pi/180;
L(6).qlim = [0 360]*pi/180;
robot = SerialLink(L, 'name', 'Robot arm');%��е�۵Ĺ���
% �����ʼ��״̬
Q0=[0 0 -pi/2 pi pi/2 0];%��ʼ״̬����
Q1=[0 0 -pi/3 pi pi/3 0];%��ʼ״̬����
w=[-10 50 -30 40 -10 30];% �����ռ�
q=[];%�����ռ�
e=[];%���ռ�
% Qinit=[0 0 0 0 0 0];%��ʼ�ؽڽǶ�
% T=robot.fkine(Qinit);%������ĩ�˵���α仯����
% q=robot.ikine(T);%����ĩ����̬��ȡ�ؽڽ�
% չʾ��ʼλ�õĻ�е��
%robot.plot(Q1,'workspace',w);
% ʾ�̹۲��е�۵�λ��
% teach(robot)

% 2.��е���˶�ѧ���
% 2.1�����󲿹̶�ʱ�����ǰ�����ؽڱ���
syms a1 a2 a3 a4 a5 a6 t r1 r2 r3 r4 r5 r6
syms a b c d e f theta1 theta2 theta3
%���� theta1 theta2 theta3 Ϊĩ����̬��ŷ����
%����ĩ�˵�λ�ã�
final_x=15;
final_y=-8;
final_z=15;
%����ĩ�˵���̬  ����Z-Y-X��ŷ���Ǳ�ʾ
for t=0.2:0.001:pi*3/4
theta1=0.5;%��Z����ת
theta2=0;%��Y����ת
theta3=t;%��X����ת
RZ=[cos(theta1) -sin(theta1) 0;
    sin(theta1) cos(theta1)  0;
    0           0            1];
RY=[cos(theta2) 0 sin(theta2);
    0           1          0 ;
    -sin(theta2) 0  cos(theta2)];
RX=[1  0  0;
    0  cos(theta3)  -sin(theta3);
    0  sin(theta3)  cos(theta3)];
Final_R=RZ*RY*RX;%��ȡ�����յ���ת����
%����ĩ�˵���α任����ΪB
B=[Final_R(1,:) final_x;Final_R(2,:) final_y;Final_R(3,:) final_z;0 0 0 1];
T=B;
px=T(1,4)-6*T(1,3);%��px
py=T(2,4)-6*T(2,3);%��py
pz=T(3,4)-6*T(3,3);%��pz
L=px^2+py^2+pz^2;%�󲿾����׼��ľ���ƽ��
r1=atan2(py,px)-atan2(-5,sqrt(px^2+py^2-25));%���r1
K=(26*pz-16-L)/128;%�й����Ƿ���̬�ɴ�Ĳ���
% display(K);
if abs((26*pz-16-L)/128)>1
    r3=-1.57;
else
    r3=real(asin((26*pz-16-L)/128));%���r3
end
a=real(8-8*sin(r3));
b=real(8*cos(r3));
c=real(-8*cos(r1)*cos(r3));
d=real(8*cos(r1)-8*cos(r1)*sin(r3));
e=real(13-pz);
f=real(px-5*sin(r1));
if abs(b)<0.001 || abs(c)<0.001
    r2=0;
else
    r2=atan2((e*d-f*b)/(a*d-b*c),(e*c-f*a)/(b*c-a*d));%���r2
end
% 2.2 ������õ�ǰ�������������������ؽڱ���
T11=[cos(r1) 0 -sin(r1) 0;
    sin(r1) 0 cos(r1)  0;
    0      -1  0      13;
    0      0    0      1];
T22=[cos(r2) -sin(r2) 0  8*cos(r2);
    sin(r2) cos(r2)  0  8*sin(r2);
    0      0    1      0;
    0      0    0      1];
T33=[cos(r3) 0 -sin(r3) 0;
    sin(r3) 0 cos(r3)  0;
    0      -1   0      -5;
    0      0    0      1];
R3=(T11*T22*T33)^-1;% �����
R=R3*B;
r5=atan2(-sqrt(R(1,3)^2+R(2,3)^2),R(3,3));%���r5
r4=atan2(-R(2,3)/sin(r5),-R(1,3)/sin(r5));%���r4
r6=atan2(R(3,2)/-sin(r5),R(3,1)/sin(r5));%���r6
% 2.3 ���ļ������ı���
T44=[cos(r4) 0 sin(r4)   0;
    sin(r4) 0 -cos(r4)  0;
    0      1    0       8;
    0      0    0       1];
T55=[cos(r5) 0 -sin(r5)   0;
    sin(r5) 0  cos(r5)   0;
    0      -1    0       0;
    0      0    0       1];
T66=[cos(r6) -sin(r6) 0 0;
    sin(r6) cos(r6)  0 0;
    0      0    1      6;
    0      0    0      1];
% robot.plot(QQ,'workspace',w,'jaxes','jointdiam',1.3)
QQ=[r1 r2 r3 r4 r5 r6];
M=T11*T22*T33*T44*T55*T66;
N=M(1:3,4)-B(1:3,4);
e=N'*N;%����ƽ�����
display(e);%��ʾ��ÿ�ε����
    if e<=1 && r3~=-1.57%�ų�����������µĽ�
        q=[q;QQ];%������õ��ĹؽڽⱣ������
    end
end

%���˶�ѧ���
% a1=0.16;
% a2=-0.26;
% a3=-0.58;
% a4=0.7;
% a5=-0.59;
% a6=0.39;
% T1=[cos(a1) 0 -sin(a1) 0;
%     sin(a1) 0 cos(a1)  0;
%     0      -1  0      13;
%     0      0    0      1];
% T2=[cos(a2) -sin(a2) 0  8*cos(a2);
%     sin(a2) cos(a2)  0  8*sin(a2);
%     0      0    1      0;
%     0      0    0      1];
% T3=[cos(a3) 0 -sin(a3) 0;
%     sin(a3) 0 cos(a3)  0;
%     0      -1   0      -5;
%     0      0    0      1];
% T4=[cos(a4) 0 sin(a4)   0;
%     sin(a4) 0 -cos(a4)  0;
%     0      1    0       8;
%     0      0    0       1];
% T5=[cos(a5) 0 -sin(a5)   0;
%     sin(a5) 0  cos(a5)   0;
%     0      -1    0       0;
%     0      0    0       1];
% T6=[cos(a6) -sin(a6) 0 0;
%     sin(a6) cos(a6)  0 0;
%     0      0    1      6;
%     0      0    0      1];
% A=T1*T2*T3*T4;
% display(A);
% B=T1*T2*T3*T4*T5*T6;
% display(B);
% C=T5*T6;
% display(C);
% init=[0.19 -0.26 -0.58 0.22 -0.59 0.29];
% robot.plot(init,'workspace',w);
% T=robot.fkine(init);%���ÿ⺯������α任����
% for t=0:0.1:3.14
%     q=robot.ikine(T);
%     robot.plot(q,'workspace',w);
% end
% px=T.t(1,1)-6*T.a(1,1);
% py=T.t(2,1)-6*T.a(2,1);
% pz=T.t(3,1)-6*T.a(3,1);


