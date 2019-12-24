%% 轨迹规划
% 计算保存原始数据的速度量
temp=q(1,:);
v=[];%创建v矩阵来存储
for i=2:size(q)
    v=[v;q(i,:)-temp];
    temp=q(i,:);
end
% 调节个别角度相差了一个周期的问题
for i=1:size(q)
    if q(i,6)<0
        q(i,6)=q(i,6)+2*pi;
    end
end

% 1.参数输入
vmax=0.01;%输入关节的最大速度
amax=0.1;%输入关节的最大加速度
% 主导关节的选择
max=size(q);
q1=q(max(1,1),1)-q(1,1);%计算关节1变化的范围
q2=q(max(1,1),2)-q(1,2);%计算关节2变化的范围
q3=q(max(1,1),3)-q(1,3);%计算关节3变化的范围
q4=q(max(1,1),4)-q(1,4);%计算关节4变化的范围
q5=q(max(1,1),5)-q(1,5);%计算关节5变化的范围
q6=q(max(1,1),6)-q(1,6);%计算关节6变化的范围
Dq=[q1 q2 q3 q4 q5 q6];%保存关节的范围

guihua_q=q(1,:);%存储规划的关节位置
k=1;%当前位置指针
for i=k:(2135-1)/2%前半段轨迹规划
    for j=k+1:2135/2
        if(abs(q(j,4)-q(k,4))>0.0006*amax*k)
            guihua_q=[guihua_q;q(j,:)];
            k=j;
        end
    end
end
display(size(guihua_q));%观察前半段选取了多少点
l=(2135-1)/2+1;
for i=l:2135-1%后半段轨迹规划
    for j=l+1:2135
        if(abs(q(j,4)-q(l,4))>0.0006*amax*(2135-l))
            guihua_q=[guihua_q;q(j,:)];
            l=j;
        end
    end
end
display(size(guihua_q));%观察后半段选取了多少点
% 2.速度计算与保存
guihua_temp=guihua_q(1,:);
guihua_v=[];%将规划的速度值记录并打印
for i=2:size(guihua_q)
    guihua_v=[guihua_v;guihua_q(i,:)-guihua_temp];
    guihua_temp=guihua_q(i,:);
end
% for i=2:size(v)
%       if(guihua_v(i,1)<vmax && guihua_v(i,2) <vmax && guihua_v(i,3)<vmax&&guihua_v(i,4)<vmax&&guihua_v(i,5)<vmax&&guihua_v(i,6)<vmax) %         temp_v=temp_v+v(i,:);
%       end
%     guihua_v=[guihua_v;temp_v];
% end
   
%绘制关节位置曲线
figure(1)
i=1:6;
plot(guihua_q(:,i));
title('关节位置-时间图')
xlabel('时间t');
ylabel('关节角度');
grid on;
%绘制速度曲线
figure(2)
subplot(3,2,1);
plot(guihua_v(:,1));
title('关节1速度');
xlabel('时间t');
ylabel('关节转速');
grid on;
subplot(3,2,2);
plot(guihua_v(:,2));
title('关节2速度');
xlabel('时间t');
ylabel('关节转速');
grid on;
subplot(3,2,3);
plot(guihua_v(:,3));
title('关节3速度');
xlabel('时间t');
ylabel('关节转速');
grid on;
subplot(3,2,4);
plot(guihua_v(:,4));
title('关节4速度');
xlabel('时间t');
ylabel('关节转速');
grid on;
subplot(3,2,5);
plot(guihua_v(:,5));
title('关节5速度');
xlabel('时间t');
ylabel('关节转速');
grid on;
subplot(3,2,6);
plot(guihua_v(:,6));
title('关节6速度');
xlabel('时间t');
ylabel('关节转速');
grid on;
% 绘制机械臂的运动轨迹
for i=1:size(guihua_q)
    figure(3)
    robot.plot(guihua_q(i,:),'workspace',w,'jaxes','jointdiam',1.3);
    title('机械臂运动');
end