clc;
clearvars;
%連桿偏移
d1 = 0;
d2 = 0;
d3 = 0;
d4 = 805;
d5 = 0;
d6 = 110;
%連桿長度
a1 = 0;
a2 = 150;
a3 = 770;
a4 = 150;
a5 = 0;
a6 = 0;
%連桿角度
alpha1 = deg2rad(0);
alpha2 = deg2rad(90);
alpha3 = deg2rad(0);
alpha4 = deg2rad(90);
alpha5 = deg2rad(-90);
alpha6 = deg2rad(90);

%DH參數表  theta  d    a     alpha   
L1 = Link([ 0    d1   a1    alpha1 ],'modified'); 
L1.offset = deg2rad(90);
L2 = Link([ 0    d2   a2    alpha2 ],'modified');
L2.offset = deg2rad(90);
L3 = Link([ 0    d3   a3    alpha3 ],'modified');
L3.offset = deg2rad(0);
L4 = Link([ 0    d4   a4    alpha4 ],'modified');
L4.offset = deg2rad(0);
L5 = Link([ 0    d5   a5    alpha5 ],'modified');
L5.offset = deg2rad(0);
L6 = Link([ 0    d6   a6    alpha6 ],'modified');
L6.offset = deg2rad(0);

%最大運動範圍
L1.qlim = [(-180/180)*pi,(180/180)*pi];
L2.qlim = [(-135/180)*pi,(100/180)*pi];
L3.qlim = [(-80/180)*pi,(190/180)*pi];
L4.qlim = [(-200/180)*pi,(200/180)*pi];
L5.qlim = [(-130/180)*pi,(130/180)*pi];
L6.qlim = [(-360/180)*pi,(360/180)*pi];

%連接連桿
Ra620 = SerialLink([L1 L2 L3 L4 L5 L6], 'name', 'RA620', 'modified');
%列出DH參數表
Ra620.display;
%%
while(true)
    x = input('請輸入1.教導　2.ptp_pos　3.ptp_axis　4.多次ptp_pos　5.多次ptp_axis　6.一次性ptp_axis並製作gif　7.Exit = ');
    
    %教導
    if 1 == x 
        title('Ra620  Simulation','FontSize',12);
        set(gca,'FontName','Times New Roman','FontSize',8)
        joint(: , 1) = linspace(deg2rad(0),deg2rad(0));
        joint(: , 2) = linspace(deg2rad(0),deg2rad(0));
        joint(: , 3) = linspace(deg2rad(0),deg2rad(0));
        joint(: , 4) = linspace(deg2rad(0),deg2rad(0));
        joint(: , 5) = linspace(deg2rad(0),deg2rad(0));
        joint(: , 6) = linspace(deg2rad(0),deg2rad(0));
        Ra620.plot(joint ,'jointdiam',1,'fps',100,'trail','r-')
    
        %展示模型 
        figure(2)
        title('Ra620  Simulation','FontSize',20);
        set(gca,'FontName','Times New Roman','FontSize',15)
        set(gca,'linewidth',1.5)
 
        Ra620.teach('rpy/zyx');
        view(90,0)

    %ptp_pos    
    elseif 2 == x
        P1 = [0, 0, 0, 0; 0, 0, 0, 0; 0, 0, 0, 0; 0, 1065, 920, 0];
        P_1 = transpose(P1);
        R_1 = trotz(-90) * troty(-90) * trotx(0);
        T1 = R_1 + P_1;
        T1 = SE3(T1);
        while 2 == x
            %disp('跳出迴圈請按 Ctrl + C')
            robot_continue = input('是否作動 0.否 1.是 = ');
            if 0 == robot_continue
                fprintf('\n');
                break
            elseif 1 == robot_continue
                world_X = input('請輸入世界座標系 X = ');
                world_Y = input('請輸入世界座標系 Y = ');
                world_Z = input('請輸入世界座標系 Z = ');
                rotation_X = input('請輸入 A 軸 夾角 = ');
                rotation_Y = input('請輸入 B 軸 夾角 = ');
                rotation_Z = input('請輸入 C 軸 夾角 = ');
                P2 = [0, 0, 0, 0; 0, 0, 0, 0; 0, 0, 0, 0; world_X, world_Y, world_Z, 0];
                P_2 = transpose(P2);
                R_2 = trotz(rotation_Z) * troty(rotation_Y) * trotx(rotation_X);
                T2 = R_2 + P_2;
                T2 = SE3(T2);

                T_1 = ctraj(T1,T2,25);
                T_2 = transl(T_1);
                q = Ra620.ikine(T_1);
    
                title('Ra620  Simulation','FontSize',12);
                set(gca,'FontName','Times New Roman','FontSize',8)
                Ra620.plot(q,'jointdiam',1,'fps',100,'trail','b-');
                disp('-----------作動完成-----------')
            
                T1 = T2;
                disp('----------各關節角度----------');
                fprintf('%.3f\t', rad2deg(q(25,:)));
                fprintf('\n')
                fprintf('\n')
            else
                disp('請重新輸入!');
                fprintf('\n');
            end
        end
        
    %ptp_axis    
    elseif 3 == x 
        joint_1_now = 0;
        joint_2_now = 0;
        joint_3_now = 0;
        joint_4_now = 0;
        joint_5_now = 0;
        joint_6_now = 0;
        while 3 == x
            %disp('跳出迴圈請按 Ctrl + C')
            robot_continue = input('是否作動 0.否 1.是 = ');
            if 0 == robot_continue
                fprintf('\n');
                break
            elseif 1 == robot_continue
                joint_1_after = input('請輸入第一軸關節角度 = ');
                joint(: , 1) = linspace(deg2rad(joint_1_now),deg2rad(joint_1_after));
                joint_2_after = input('請輸入第二軸關節角度 = ');
                joint(: , 2) = linspace(deg2rad(joint_2_now),deg2rad(joint_2_after));
                joint_3_after = input('請輸入第三軸關節角度 = ');
                joint(: , 3) = linspace(deg2rad(joint_3_now),deg2rad(joint_3_after));
                joint_4_after = input('請輸入第四軸關節角度 = ');
                joint(: , 4) = linspace(deg2rad(joint_4_now),deg2rad(joint_4_after));
                joint_5_after = input('請輸入第五軸關節角度 = ');
                joint(: , 5) = linspace(deg2rad(joint_5_now),deg2rad(joint_5_after));
                joint_6_after = input('請輸入第六軸關節角度 = ');
                joint(: , 6) = linspace(deg2rad(joint_6_now),deg2rad(joint_6_after));
                title('Ra620  Simulation','FontSize',12);
                set(gca,'FontName','Times New Roman','FontSize',8)
                Ra620.plot(joint ,'jointdiam',1,'fps',100,'trail','r-')
                disp('------------作動完成------------')
    
                joint_1_now = joint_1_after;
                joint_2_now = joint_2_after;
                joint_3_now = joint_3_after;
                joint_4_now = joint_4_after;
                joint_5_now = joint_5_after;
                joint_6_now = joint_6_after;   
        
                thetadeg = [deg2rad(joint_1_after) deg2rad(joint_2_after) deg2rad(joint_3_after) deg2rad(joint_4_after) deg2rad(joint_5_after) deg2rad(joint_6_after)];
                disp('----------齊次轉換矩陣----------');
                Ra620.fkine(thetadeg).display;
                disp('--------------------------------');
                fprintf('\n')
            else
                disp('請重新輸入!');
                fprintf('\n');
            end
        end
        
    %多次ptp_pos    
    elseif 4 == x
        P1 = [0, 0, 0, 0; 0, 0, 0, 0; 0, 0, 0, 0; 0, 1065, 920, 0];
        P_1 = transpose(P1);
        R_1 = trotz(-90) * troty(-90) * trotx(0);
        T1 = R_1 + P_1;
        T1 = SE3(T1);
        while 4 == x
            %disp('跳出迴圈請按 Ctrl + C')
            robot_continue = input('是否作動 0.否 1.是 = ');
            if 0 == robot_continue
                fprintf('\n');
                break
            elseif 1 == robot_continue
                y = input('請輸入需移動位置數 = ');
                fprintf('\n');
                for i = 1:y
                    fprintf('第 %d 移動點設定\n',i);
                    world_X = input('請輸入世界座標系 X = ');
                    world_Y = input('請輸入世界座標系 Y = ');
                    world_Z = input('請輸入世界座標系 Z = ');
                    rotation_X = input('請輸入 A 軸 夾角 = ');
                    rotation_Y = input('請輸入 B 軸 夾角 = ');
                    rotation_Z = input('請輸入 C 軸 夾角 = ');
                    disp('--------------------------------'); 
                    P2 = [0, 0, 0, 0; 0, 0, 0, 0; 0, 0, 0, 0; world_X, world_Y, world_Z, 0];
                    P_2 = transpose(P2);
                    R_2 = trotz(rotation_Z) * troty(rotation_Y) * trotx(rotation_X);
                    T2 = R_2 + P_2;
                    T2 = SE3(T2);
                    eval(['moving_',num2str(i),'=','T2',';']);
                end
                for j = 1:y
                    T2 = eval(['moving_',num2str(j),'(1,1)']);

                    T_1 = ctraj(T1,T2,25);
                    T_2 = transl(T_1);
                    q = Ra620.ikine(T_1);
    
                    title('RA620  Simulation','FontSize',12);
                    set(gca,'FontName','Times New Roman','FontSize',8)
                    Ra620.plot(q,'jointdiam',1,'fps',100,'trail','b-');
                    fprintf('完成第 %d 移動點\n',j);   
            
                    T1 = T2;
                    disp('--------------------------------');
                end   
            else
                disp('請重新輸入!');
                fprintf('\n');
            end    
        end
        
    %多次ptp_axis    
    elseif 5 == x    
         joint_1_now = 0;
         joint_2_now = 0;
         joint_3_now = 0;
         joint_4_now = 0;
         joint_5_now = 0;
         joint_6_now = 0;
         while 5 == x
             %disp('跳出迴圈請按 Ctrl + C')
             robot_continue = input('是否作動 0.否 1.是 = ');
             if 0 == robot_continue
                 fprintf("\n");
                 break
             elseif 1 == robot_continue
                 y = input('請輸入需移動位置數 = ');
                 fprintf('\n');
                 for i = 1:y
                     fprintf('第 %d 移動點設定\n',i);
                     joint_1_after = input('請輸入第一軸關節角度 = ');
                     joint_2_after = input('請輸入第二軸關節角度 = ');
                     joint_3_after = input('請輸入第三軸關節角度 = ');
                     joint_4_after = input('請輸入第四軸關節角度 = ');
                     joint_5_after = input('請輸入第五軸關節角度 = ');
                     joint_6_after = input('請輸入第六軸關節角度 = ');
                     disp('--------------------------------'); 
                     m = [joint_1_after joint_2_after joint_3_after joint_4_after joint_5_after joint_6_after];
                     eval(['moving_',num2str(i),'=','m',';']);
                 end
                 for j = 1:y
                     joint_1_after = eval(['moving_',num2str(j),'(1,1)']);
                     joint_2_after = eval(['moving_',num2str(j),'(1,2)']);
                     joint_3_after = eval(['moving_',num2str(j),'(1,3)']);
                     joint_4_after = eval(['moving_',num2str(j),'(1,4)']);
                     joint_5_after = eval(['moving_',num2str(j),'(1,5)']);
                     joint_6_after = eval(['moving_',num2str(j),'(1,6)']);
                
                     joint(: , 1) = linspace(deg2rad(joint_1_now),deg2rad(joint_1_after));
                     joint(: , 2) = linspace(deg2rad(joint_2_now),deg2rad(joint_2_after));
                     joint(: , 3) = linspace(deg2rad(joint_3_now),deg2rad(joint_3_after));
                     joint(: , 4) = linspace(deg2rad(joint_4_now),deg2rad(joint_4_after));
                     joint(: , 5) = linspace(deg2rad(joint_5_now),deg2rad(joint_5_after));
                     joint(: , 6) = linspace(deg2rad(joint_6_now),deg2rad(joint_6_after));
                
                     title('RA620  Simulation','FontSize',12);
                     set(gca,'FontName','Times New Roman','FontSize',8)
                     Ra620.plot(joint ,'jointdiam',1,'fps',100,'trail','r-')
                     fprintf('完成第 %d 移動點\n',j);             
                
                     joint_1_now = joint_1_after;
                     joint_2_now = joint_2_after;
                     joint_3_now = joint_3_after;
                     joint_4_now = joint_4_after;
                     joint_5_now = joint_5_after;
                     joint_6_now = joint_6_after;   
                     disp('--------------------------------');    
                 end
             else
                 disp('請重新輸入!');
             end
         end   
         
    %一次性ptp_axis並製作gif
    elseif 6 == x
       while 6 == x
           %disp('跳出迴圈請按 Ctrl + C')
           robot_continue = input('是否作動 0.否 1.是 = ');
           if 0 == robot_continue
               fprintf("\n");
               break
           elseif 1 == robot_continue
               joint_1 = input('請輸入第一軸關節角度 = ');
               joint(: , 1) = linspace(deg2rad(0),deg2rad(joint_1));
               joint_2 = input('請輸入第二軸關節角度 = ');
               joint(: , 2) = linspace(deg2rad(0),deg2rad(joint_2));
               joint_3 = input('請輸入第三軸關節角度 = ');
               joint(: , 3) = linspace(deg2rad(0),deg2rad(joint_3));
               joint_4 = input('請輸入第四軸關節角度 = ');
               joint(: , 4) = linspace(deg2rad(0),deg2rad(joint_4));
               joint_5 = input('請輸入第五軸關節角度 = ');
               joint(: , 5) = linspace(deg2rad(0),deg2rad(joint_5));
               joint_6 = input('請輸入第六軸關節角度 = ');
               joint(: , 6) = linspace(deg2rad(0),deg2rad(joint_6));
               title('Ra620  Simulation','FontSize',12);
               set(gca,'FontName','Times New Roman','FontSize',8)
               Ra620.plot(joint ,'jointdiam',1,'fps',100,'trail','r-')

               name = input('錄製名稱：');
               filename = [num2str(name),'.gif'];
               fprintf('gif製作中...');
               for i = 1:length(joint)
                   drawnow;
                   Ra620.plot(joint(i,:));
                   F = getframe(gcf);  
                   I = frame2im(F);
                   [I,map] = rgb2ind(I,256);
                   if i == 1
                       imwrite(I,map,filename,'gif', 'Loopcount',inf,'DelayTime',0.1);
                   else
                       imwrite(I,map,filename,'gif','WriteMode','append','DelayTime',0.1);
                   end
               end
               fprintf("\n");
               disp('錄製結束');
               fprintf("\n");
           end
       end
       
    elseif 7 == x
        close all;
        break;
    else        
        disp('請重新輸入!');
        fprintf('\n');
    end
end
