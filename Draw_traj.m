function Draw_traj(handles,app)
%%%%%%%%%%%%%%%%%%%%%%%%% time %%%%%%%%%%%%%%%%%%%%%%%%%%% 
%master_t{dataset_num} = [start to fly, touch the ground]
%pass 3-11
master_t{3} = [3079355 4379344];
master_t{4} = [2912688 4046022];
master_t{5} = [3112677 4146022];
master_t{6} = [3979355 5079355];
master_t{7} = [4279355 5312677];
master_t{8} = [4879355 6046011];
master_t{9} = [5379355 6412688];
master_t{10} = [5379344 6546011];
master_t{11} = [4746011 5912677];
%no human at master view 12-14
master_t{12} = [4379355 5446011];
master_t{13} = [3012688 3912700];
master_t{14} = [2712688 3779344];
%Bound pass 15-16
master_t{15} = [4579355 4879344; 4946011 5946011];
master_t{16} = [2779355 2946011; 2979344 3979366];

%%
trans_extrinsic = 0;
datanum = get(handles.popupmenu1,'Value') + 2;
[RMSE_RegressionGT, RMSE_Sample] = error_cal_with_draw(datanum, master_t, trans_extrinsic, handles,app);

% mean(error_master)
% mean(error_sub)

%%
% datanum = 3;
% showTomaster(datanum, master_t)
end

%%
function [RMSE_RegressionGT, RMSE_Sample] = error_cal_with_draw(datanum, master_t, trans_extrinsic, handles,app)
meter_disp = 0;
%extrinsic parameter
m2s = [-0.0560914164776252,0.0289597612225271,-0.998005553705723,2463.56384997572;...
        -0.0181056325880368,0.999385381984152,0.0300174007032279,68.9575934605440;...
        0.998261458269386,0.0197532404006365,-0.0555326068859913,2317.39673268936];
s2m = inv([m2s; 0 0 0 1]);
s2m = s2m(1:3,:);

%intrinsic parameter
intr_master = [610.893, 0, 637.595; 0, 610.849, 369.618; 0, 0, 1];
intr_sub = [614.324, 0, 637.612; 0, 614.333, 368.556; 0, 0, 1];

%data load
master = load("traj_csv/" + "master" + string(datanum) + ".csv");
sub = load("traj_csv/" + "sub" + string(datanum) + ".csv");
term_t = master_t{datanum};

term_t = term_t/1000;
master(:,4) = master(:,4)/1000;
sub(:,4) = sub(:,4)/1000;
RMSE_RegressionGT = [];
RMSE_Sample = [];

for i = 1:size(term_t,1) % 바운드 패스는 손을떠나 바닥에 닿기 전, 닿은 후로 분리
    %모든 공 위치 중에서 손을 벗어난 순간을 pick
    idx1 = master(:,4)>term_t(i,1) &  master(:,4)<term_t(i,2);
    idx2 = sub(:,4)>term_t(i,1) &  sub(:,4)<term_t(i,2);

    master_pick = master(idx1,:);
    sub_pick = sub(idx2,:);
    if(isempty(sub_pick) || isempty(master_pick))
        continue;
    end
    if(get(handles.uibuttongroup1,'SelectedObject').String=="Robot view")
        set(handles.text6,'String',sprintf('max = %d',size(sub_pick,1)));
        set(app.uipanel3,'Title','Components - Robot camera');
        set(app.uipanel3_2,'Title','Extrinsic projection from - Side camera');
    end
    if(get(handles.uibuttongroup1,'SelectedObject').String=="Side view")
        set(handles.text6,'String',sprintf('max = %d',size(master_pick,1)));
        set(app.uipanel3,'Title','Components - Side camera');
        set(app.uipanel3_2,'Title','Extrinsic projection from - Robot camera');
    end
    
    if(trans_extrinsic)
        master_pick_trans = (s2m * cart2hom(sub_pick(:,1:3))')';
        master_pick_trans = [master_pick_trans sub_pick(:,4)];
        [coef_x, coef_y, coef_z] = traj_regression(master_pick_trans);
        
          
        sub_pick_trans = (m2s * cart2hom(master_pick(:,1:3))')';
        sub_pick_trans = [sub_pick_trans master_pick(:,4)];
        [coef_x_sub, coef_y_sub, coef_z_sub] = traj_regression(sub_pick_trans);  

    else
        [coef_x, coef_y, coef_z] = traj_regression(master_pick);
        [coef_x_sub, coef_y_sub, coef_z_sub] = traj_regression(sub_pick);     
    end

    % time_steps = master_pick(:,4);
    if(get(handles.uibuttongroup1,'SelectedObject').String=="Side view")
        if(get(handles.checkbox1,'Value'))
            Draw_3Dpoints(handles,master_pick);
        end
        if(get(handles.checkbox2,'Value'))
            err_RegressionGT = Draw_RegressionGT(handles,master_pick);
        end
        if(get(handles.checkbox3,'Value'))
            err_Sample = Draw_Sampled_points_trajectory(handles,master_pick);
        end
        title('Master')
        xlabel('X')
        ylabel('Y')
        zlabel('Z')
        
        text(master_pick(1,1),master_pick(1,3),-master_pick(1,2),"Start",'FontSize',10, 'Color', 'c');
        text(0,0,0,"Side cam",'FontSize',10, 'Color', 'c')

        pose = rigid3d(m2s(:,1:3),[-m2s(1,4),m2s(3,4),-m2s(2,4)]);
        plotCamera('AbsolutePose',pose,'Opacity',0, 'Size',50)
        text(-m2s(1,4),m2s(3,4),-m2s(2,4),"Robot cam",'FontSize',10, 'Color', 'c')
    end
    if(get(handles.uibuttongroup1,'SelectedObject').String=="Robot view")
        if(get(handles.checkbox1,'Value'))
            Draw_3Dpoints(handles,sub_pick);
        end
        if(get(handles.checkbox2,'Value'))
            err_RegressionGT = Draw_RegressionGT(handles,sub_pick);
        end
        if(get(handles.checkbox3,'Value'))
            err_Sample = Draw_Sampled_points_trajectory(handles,sub_pick);
        end
        title('sub')
        xlabel('X')
        ylabel('Y')
        zlabel('Z')
        
        text(sub_pick(1,1),sub_pick(1,3),-sub_pick(1,2),"Start",'FontSize',10, 'Color', 'c');
        text(0,0,0,"Robot cam",'FontSize',10, 'Color', 'c')
        
        pose = rigid3d(s2m(:,1:3),[-s2m(1,4),s2m(3,4),-s2m(2,4)]);
        plotCamera('AbsolutePose',pose,'Opacity',0, 'Size',50)
        text(-s2m(1,4),s2m(3,4),-s2m(2,4),"Side cam",'FontSize',10, 'Color', 'c')
    end
    
    R = [1 0 0; 0 0 -1;0 1 0];
    t = [0 0 0];
    pose = rigid3d(R,t);
    plotCamera('AbsolutePose',pose,'Opacity',0, 'Size',50)
    
    if(get(handles.uibuttongroup1,'SelectedObject').String=="Robot view")
        view(45,45)
    end
    if(get(handles.uibuttongroup1,'SelectedObject').String=="Side view")
        view(-45,45)
    end
            
    %error var save for bound pass
    if(get(handles.checkbox2,'Value'))
        err_RegressionGT = err_RegressionGT.*err_RegressionGT;
        RMSE_RegressionGT = [RMSE_RegressionGT; err_RegressionGT];
    end
    if(get(handles.checkbox3,'Value'))
        err_Sample = err_Sample.*err_Sample;
        RMSE_Sample = [RMSE_Sample; err_Sample];
    end
    
end %bound pass 2 term iteration end

%error calcul & visual
if(get(handles.checkbox2,'Value')) %regressionGT
RMSE_RegressionGT = sqrt(sum(RMSE_RegressionGT(:))/size(RMSE_RegressionGT,1));
    if(get(handles.checkbox4,'Value')) %cm
        set(handles.text10,'String',sprintf('%.2fcm',RMSE_RegressionGT/10));
    else
        set(handles.text10,'String',sprintf('%.2fmm',RMSE_RegressionGT));
    end
else
    set(handles.text10,'String',"-");
end
if(get(handles.checkbox3,'Value')) %sample traj
RMSE_Sample = sqrt(sum(RMSE_Sample(:))/size(RMSE_Sample,1));
    if(get(handles.checkbox4,'Value')) %cm
        set(handles.text11,'String',sprintf('%.2fcm',RMSE_Sample/10));
    else
        set(handles.text11,'String',sprintf('%.2fmm',RMSE_Sample));
    end
else
    set(handles.text11,'String',"-");
    set(handles.text12,'String',"-");
end

if(meter_disp)
    meter = 1;
    meter_time = (meter*1000-coef_z_sub(2))/coef_z_sub(1);
    GT_meter = [coef_x_sub(1)*meter_time + coef_x_sub(2), coef_y_sub(1)*meter_time.*meter_time + coef_y_sub(2)*meter_time + coef_y_sub(3), coef_z_sub(1)*meter_time + coef_z_sub(2)];
    num_over_meter = sum(sub_pick(:,3)>(meter*1000));
    for i=2:num_over_meter
        velo = sub_pick(i,:) - sub_pick(i-1,:);
        velo = velo(1:3)/velo(4);
        sub_meter_time = (meter*1000-sub_pick(i,3))/velo(3);
        sub_meter{i} = [velo(1)*sub_meter_time + sub_pick(i,1), 9.8/2000*sub_meter_time.*sub_meter_time + velo(2)*sub_meter_time + sub_pick(i,2), velo(3)*sub_meter_time + sub_pick(i,3)];
    end
    sub_meter2 = cell2mat(sub_meter');
    err = sub_meter2 - GT_meter;
    err_meter_sub = err.*err;
    err_meter_sub = sqrt(sum(err_meter_sub,2));
    sub_res = [sub_pick(2:num_over_meter,:) sub_meter2 err_meter_sub];
    sub_res = [sub_res; 0 0 0 0 GT_meter 0];

    figure, pcshow(sub_res(:,5:7)-sub_res(end,5:7),[sub_res(1:end-1,3); meter*1000],'markersize',5000)
    colorbar
    xlabel('X')
    ylabel('Y')
    %saveas(gcf,'Barchart.png')
    %black_image = zeros(720,1280,3);
end

end %function end

%%
function showTomaster(datanum, master_t)
m2s = [-0.0560914164776252,0.0289597612225271,-0.998005553705723,2463.56384997572;...
        -0.0181056325880368,0.999385381984152,0.0300174007032279,68.9575934605440;...
        0.998261458269386,0.0197532404006365,-0.0555326068859913,2317.39673268936];
s2m = inv([m2s; 0 0 0 1]);
s2m = s2m(1:3,:);

intr_master = [610.893, 0, 637.595; 0, 610.849, 369.618; 0, 0, 1];
intr_sub = [614.324, 0, 637.612; 0, 614.333, 368.556; 0, 0, 1];

master = load("traj_csv/" + "master" + string(datanum) + ".csv");
sub = load("traj_csv/" + "sub" + string(datanum) + ".csv");
term_t = master_t{datanum};

term_t = term_t/1000;
master(:,4) = master(:,4)/1000;
sub(:,4) = sub(:,4)/1000;
error_master = [];
error_sub = [];

for i = 1:size(term_t,1)
    %regression
    idx1 = master(:,4)>term_t(1,1) &  master(:,4)<term_t(1,2);
    idx2 = sub(:,4)>term_t(1,1) &  sub(:,4)<term_t(1,2);

    master_pick = master(idx1,:);
    sub_pick = sub(idx2,:);

        master_pick_trans = (s2m * cart2hom(sub_pick(:,1:3))')';
        X = cart2hom(sub_pick(:,4));
        X2 = [X(:,1).^2 X];
        y_x_sub = master_pick_trans(:,1);
        y_y_sub = master_pick_trans(:,2);
        y_z_sub = master_pick_trans(:,3);

        X = cart2hom(master_pick(:,4));
        X2 = [X(:,1).^2 X];
        y_x = master_pick(:,1);
        y_y = master_pick(:,2);
        y_z = master_pick(:,3);


    % time_steps = master_pick(:,4);
        pcshow([y_x,y_y,y_z],'r','markersize',50)
        title('Master-red, Sub-green')
        xlabel('X')
        ylabel('Y')
        zlabel('Z')
        hold on
        pcshow([y_x_sub,y_y_sub,y_z_sub],'g','markersize',50)
end

end
%%
function [coef_x, coef_y, coef_z] = traj_regression(pick)
        X = cart2hom(pick(:,4));
        X2 = [X(:,1).^2 X];
        y_x = pick(:,1);
        y_y = pick(:,2);
        y_z = pick(:,3);
        coef_x = inv(X'*X)*X'*y_x;
        coef_y = inv(X2'*X2)*X2'*y_y;
        coef_z = inv(X'*X)*X'*y_z;
end
%%
function [Vx, Vy, Vz] = traj_few_num(pick, start_num, few_num)
assert(few_num > 1, "select bigger than 1")
    [coef_x, coef_y, coef_z] = traj_regression(pick(start_num:few_num,:));
    t = pick(:,4);
    vx = [];
    vy = [];
    vz = [];
        for i=2:few_num
            vx = [vx (pick(i,1)-pick(i-1,1))/(t(i)-t(i-1))];
            vz = [vz (pick(i,3)-pick(i-1,3))/(t(i)-t(i-1))];
        end
        for i=2:few_num
            %2*v0 = 2dy/dt - 9.8/2000*dt^2
            %v0 = dy/dt - 4.9/2000*dt^2 
            dt = t(i)-t(i-1);
            dy = pick(i,2) - pick(i-1,2);
            vy = [vy (dy/dt - 9.8/2000*dt - 9.8/1000*(t(i-1) - t(1)) )];
        end
    Vx = mean(vx);
    Vy = mean(vy);
    Vz = mean(vz);
end
%% Draw
function Draw_3Dpoints(handles,pick)
    c_list = 'rgbwc';
    hold on
    %pcshow(pick(:,1:3),c_list(get(handles.popupmenu4,'Value')),'markersize',50) %image plane
    pcshow([pick(:,1),pick(:,3),-pick(:,2)],c_list(get(handles.popupmenu4,'Value')),'markersize',50)
end
function err = Draw_RegressionGT(handles,pick)
    [coef_x, coef_y, coef_z] = traj_regression(pick);
    time_steps = (pick(1,4) : pick(end,4))';
    GT_pc = [coef_x(1)*time_steps + coef_x(2), coef_y(1)*time_steps.*time_steps + coef_y(2)*time_steps + coef_y(3), coef_z(1)*time_steps + coef_z(2)];
    c_list = 'rgbwc';
    %hold on, pcshow(GT_pc,c_list(get(handles.popupmenu5,'Value'))) % imageplane
    hold on, pcshow([GT_pc(:,1),GT_pc(:,3),-GT_pc(:,2)],c_list(get(handles.popupmenu5,'Value')))
    
    time_steps = pick(:,4);
    GT_pc = [coef_x(1)*time_steps + coef_x(2), coef_y(1)*time_steps.*time_steps + coef_y(2)*time_steps + coef_y(3), coef_z(1)*time_steps + coef_z(2)];
    err = pick(:,1:3) - GT_pc;
end
function err = Draw_Sampled_points_trajectory(handles,pick)
    few_num =  str2num(get(handles.edit1,'String'));
    if(few_num>size(pick,1))
        few_num = size(pick,1);
    elseif(few_num<2)
        few_num = 2;
    else
        few_num = uint16(few_num);
    end
    set(handles.edit1,'String',string(few_num));
    set(handles.text12,'String',sprintf('0 ~ %.1f ms',pick(few_num,4)-pick(1,4)));
    start_num = 1;
    [Vx, Vy, Vz] = traj_few_num(pick, start_num, few_num);
    [coef_x_few, coef_y_few, coef_z_few] = traj_regression(pick(start_num:few_num,:));
    time_steps = (pick(1,4) : pick(end,4))';
    time_steps = time_steps - time_steps(1);
    if(few_num==2)
        few_start_points = pick(1,:);
    else
        few_start_points = [coef_x_few(1)*pick(1,4) + coef_x_few(2),...
                            coef_y_few(1)*pick(1,4).*pick(1,4) + coef_y_few(2)*pick(1,4) + coef_y_few(3),...
                            coef_z_few(1)*pick(1,4) + coef_z_few(2)];
    end
    %GT_pc = [few_coef_x(1)*time_steps + few_coef_x(2), few_coef_y(1)*time_steps.*time_steps + few_coef_y(2)*time_steps + few_coef_y(3), few_coef_z(1)*time_steps + few_coef_z(2)];
    GT_pc = [Vx*time_steps + few_start_points(1), 9.8/2000*time_steps.*time_steps + Vy*time_steps + few_start_points(2), Vz*time_steps + few_start_points(3)];
    c_list = 'rgbwc';
    %hold on, pcshow(GT_pc,c_list(get(handles.popupmenu6,'Value'))) % image plane
    hold on, pcshow([GT_pc(:,1),GT_pc(:,3),-GT_pc(:,2)],c_list(get(handles.popupmenu6,'Value')))
    
    time_steps = pick(:,4);
    time_steps = time_steps - time_steps(1);
    GT_pc = [Vx*time_steps + few_start_points(1), 9.8/2000*time_steps.*time_steps + Vy*time_steps + few_start_points(2), Vz*time_steps + few_start_points(3)];
    err = pick(:,1:3) - GT_pc;
end

%% new extrinsic 3d points
function extrinsic_3D_points()
master_pick = [];
sub_pick = [];

for datanum=3:14
master = load("traj_csv/" + "master" + string(datanum) + ".csv");
sub = load("traj_csv/" + "sub" + string(datanum) + ".csv");
term_t = master_t{datanum};
idx1 = master(:,4)>term_t(1,1) &  master(:,4)<term_t(1,2);
idx2 = sub(:,4)>term_t(1,1) &  sub(:,4)<term_t(1,2);
t_master = master(idx1,:);
t_sub = sub(idx2,:);

[~,loc] = min(abs(t_master(:,4) - t_sub(1,4)));
if(loc>1)
    t_master = t_master(loc:end,:);
end
[~,loc] = min(abs(t_master(:,4) - t_sub(end,4)));
if(loc>1)
    t_master = t_master(1:loc,:);
end

[~,loc] = min(abs(t_sub(:,4) - t_master(1,4)));
if(loc>1)
    t_sub = t_sub(loc:end,:);
end
[~,loc] = min(abs(t_sub(:,4) - t_master(end,4)));
if(loc>1)
    t_sub = t_sub(1:loc,:);
end
if(size(t_sub,1) ~= size(t_sub,1)) 
   assert(0); 
end
master_pick = [master_pick; t_master];
sub_pick = [sub_pick; t_sub];
end
master_pick = master_pick(:,1:3);
sub_pick = sub_pick(:,1:3);

A = cart2hom(master_pick);
b = sub_pick;
m2s = zeros(3,4);
for i=1:3
m2s(i,:) = inv(A' * A) * A' * b(:,i);
end
end
%% new extrinsic regressionGT 미구현
function extrinsic_regressionGT()
master_pick = [];
sub_pick = [];

for datanum=3:14
master = load("traj_csv/" + "master" + string(datanum) + ".csv");
sub = load("traj_csv/" + "sub" + string(datanum) + ".csv");
term_t = master_t{datanum};
idx1 = master(:,4)>term_t(1,1) &  master(:,4)<term_t(1,2);
idx2 = sub(:,4)>term_t(1,1) &  sub(:,4)<term_t(1,2);
t_master = master(idx1,:);
t_sub = sub(idx2,:);

[~,loc] = min(abs(t_master(:,4) - t_sub(1,4)));
if(loc>1)
    t_master = t_master(loc:end,:);
end
[~,loc] = min(abs(t_master(:,4) - t_sub(end,4)));
if(loc>1)
    t_master = t_master(1:loc,:);
end

[~,loc] = min(abs(t_sub(:,4) - t_master(1,4)));
if(loc>1)
    t_sub = t_sub(loc:end,:);
end
[~,loc] = min(abs(t_sub(:,4) - t_master(end,4)));
if(loc>1)
    t_sub = t_sub(1:loc,:);
end
if(size(t_sub,1) ~= size(t_sub,1)) 
   assert(0); 
end
master_pick = [master_pick; t_master];
sub_pick = [sub_pick; t_sub];
end
master_pick = master_pick(:,1:3);
sub_pick = sub_pick(:,1:3);

A = cart2hom(master_pick);
b = sub_pick;
m2s = zeros(3,4);
for i=1:3
m2s(i,:) = inv(A' * A) * A' * b(:,i);
end
end
