%% �켣�滮
% ���㱣��ԭʼ���ݵ��ٶ���
temp=q(1,:);
v=[];%����v�������洢
for i=2:size(q)
    v=[v;q(i,:)-temp];
    temp=q(i,:);
end
% ���ڸ���Ƕ������һ�����ڵ�����
for i=1:size(q)
    if q(i,6)<0
        q(i,6)=q(i,6)+2*pi;
    end
end

% 1.��������
vmax=0.01;%����ؽڵ�����ٶ�
amax=0.1;%����ؽڵ������ٶ�
% �����ؽڵ�ѡ��
max=size(q);
q1=q(max(1,1),1)-q(1,1);%����ؽ�1�仯�ķ�Χ
q2=q(max(1,1),2)-q(1,2);%����ؽ�2�仯�ķ�Χ
q3=q(max(1,1),3)-q(1,3);%����ؽ�3�仯�ķ�Χ
q4=q(max(1,1),4)-q(1,4);%����ؽ�4�仯�ķ�Χ
q5=q(max(1,1),5)-q(1,5);%����ؽ�5�仯�ķ�Χ
q6=q(max(1,1),6)-q(1,6);%����ؽ�6�仯�ķ�Χ
Dq=[q1 q2 q3 q4 q5 q6];%����ؽڵķ�Χ

guihua_q=q(1,:);%�洢�滮�Ĺؽ�λ��
k=1;%��ǰλ��ָ��
for i=k:(2135-1)/2%ǰ��ι켣�滮
    for j=k+1:2135/2
        if(abs(q(j,4)-q(k,4))>0.0006*amax*k)
            guihua_q=[guihua_q;q(j,:)];
            k=j;
        end
    end
end
display(size(guihua_q));%�۲�ǰ���ѡȡ�˶��ٵ�
l=(2135-1)/2+1;
for i=l:2135-1%���ι켣�滮
    for j=l+1:2135
        if(abs(q(j,4)-q(l,4))>0.0006*amax*(2135-l))
            guihua_q=[guihua_q;q(j,:)];
            l=j;
        end
    end
end
display(size(guihua_q));%�۲����ѡȡ�˶��ٵ�
% 2.�ٶȼ����뱣��
guihua_temp=guihua_q(1,:);
guihua_v=[];%���滮���ٶ�ֵ��¼����ӡ
for i=2:size(guihua_q)
    guihua_v=[guihua_v;guihua_q(i,:)-guihua_temp];
    guihua_temp=guihua_q(i,:);
end
% for i=2:size(v)
%       if(guihua_v(i,1)<vmax && guihua_v(i,2) <vmax && guihua_v(i,3)<vmax&&guihua_v(i,4)<vmax&&guihua_v(i,5)<vmax&&guihua_v(i,6)<vmax) %         temp_v=temp_v+v(i,:);
%       end
%     guihua_v=[guihua_v;temp_v];
% end
   
%���ƹؽ�λ������
figure(1)
i=1:6;
plot(guihua_q(:,i));
title('�ؽ�λ��-ʱ��ͼ')
xlabel('ʱ��t');
ylabel('�ؽڽǶ�');
grid on;
%�����ٶ�����
figure(2)
subplot(3,2,1);
plot(guihua_v(:,1));
title('�ؽ�1�ٶ�');
xlabel('ʱ��t');
ylabel('�ؽ�ת��');
grid on;
subplot(3,2,2);
plot(guihua_v(:,2));
title('�ؽ�2�ٶ�');
xlabel('ʱ��t');
ylabel('�ؽ�ת��');
grid on;
subplot(3,2,3);
plot(guihua_v(:,3));
title('�ؽ�3�ٶ�');
xlabel('ʱ��t');
ylabel('�ؽ�ת��');
grid on;
subplot(3,2,4);
plot(guihua_v(:,4));
title('�ؽ�4�ٶ�');
xlabel('ʱ��t');
ylabel('�ؽ�ת��');
grid on;
subplot(3,2,5);
plot(guihua_v(:,5));
title('�ؽ�5�ٶ�');
xlabel('ʱ��t');
ylabel('�ؽ�ת��');
grid on;
subplot(3,2,6);
plot(guihua_v(:,6));
title('�ؽ�6�ٶ�');
xlabel('ʱ��t');
ylabel('�ؽ�ת��');
grid on;
% ���ƻ�е�۵��˶��켣
for i=1:size(guihua_q)
    figure(3)
    robot.plot(guihua_q(i,:),'workspace',w,'jaxes','jointdiam',1.3);
    title('��е���˶�');
end