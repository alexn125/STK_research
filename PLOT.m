clear all
close all

load('Sim_Data/results.mat')
% load('Sim_Data/navonly_results.mat')
tvec = 0:sim.step:sim.missiontime;

for i = 1:length(tvec)
    conv = (1/(1+quat_truth(4,i)))*[quat_truth(1,i); quat_truth(2,i); quat_truth(3,i)];
    if norm(conv) >= 1.0
        MRP_truth(:,i) = conv*(-1./(conv'*conv));
    else
        MRP_truth(:,i) = conv;
    end
end

pvec = nav.m_history(1:3,:);
bvec = nav.m_history(4:6,:);
tt = nav.t_history;

time_vec = tt(1:2:end);

for i = 0:time_vec(end)
    evec(:,i+1) = subMRP(MRP_truth(:,i+1),pvec(:,2*i+1));
end

stk.mrp_true = MRP_truth;
stk.mrp_nav = pvec;
stk.nav_err = evec;

clear MRP_truth pvec evec

addpath("../deepthought/RigidBodyKinematicsSchaubBook/Matlab/")

load('../deepthought/GNCout/SIM_ALL_STUFF/FULLresults.mat')

len = size(qbn);
MRPtruth = zeros(len(1),3);
MRPtriad = zeros(3,length(time_vec));
addpath("ALGORITHM/transforms/")

for i = 1:len(1)
    conv = (1/(1+qbn(i,4)))*[qbn(i,1); qbn(i,2); qbn(i,3)];
    if norm(conv) >= 1.0
        MRPtruth(i,:) = conv*(-1./(conv'*conv));
    else
        MRPtruth(i,:) = conv;
    end
    if i < len(1)
        conv2 = (1/(1+triadhistory(1,i)))*[triadhistory(2,i); triadhistory(3,i); triadhistory(4,i)];
        if norm(conv2) >= 1.0
            MRPtriad(:,i) = conv2*(-1./(conv2'*conv2));
        else
            MRPtriad(:,i) = conv2;
        end    
    end
end

pvec = nav.m_history(1:3,:);

deep.mrp_true = MRPtruth';
deep.mrp_nav = pvec;
for i = 0:time_vec(end)
    evec(:,i+1) = subMRP(deep.mrp_true(:,i+1),pvec(:,2*i+1));
end
deep.mrp_err = evec;

clear MRPtruth pvec evec

navtable = readtable('../Modelspace_AlexN/results/nav_log.csv');
wt.time = navtable{:,1}';
wt.mrp_nav = navtable{:,2:4}';

guidtalbe = readtable('../Modelspace_AlexN/results/guid_log.csv');

truetable = readtable('../Modelspace_AlexN/results/truth.csv');
truequat = truetable{:,2:5};
for i = 1:length(wt.time)
    conv = (1/(1+truequat(i,1)))*[truequat(i,2); truequat(i,3); truequat(i,4)];
    if norm(conv) >= 1.0
        MRPtruth(i,:) = conv*(-1./(conv'*conv));
    else
        MRPtruth(i,:) = conv;
    end
end    

MRPtruth = MRPtruth';

for i = 0:time_vec(end-1)
    evec(:,i+1) = subMRP(MRPtruth(:,i+1),wt.mrp_nav(:,i+1));
end
wt.nav_err = evec;


figure;
t = tiledlayout(3,1);
nexttile
title(t,"Attitude Estimation Error Comparison, Full GNC")
plot(time_vec,stk.nav_err(1,:),time_vec,deep.mrp_err(1,:),wt.time,wt.nav_err(1,:))
xlabel('Time (s)')
ylabel("MRP 1")
legend('STK','42','WarpTwin')
nexttile
plot(time_vec,stk.nav_err(2,:),time_vec,deep.mrp_err(2,:),wt.time,wt.nav_err(2,:))
xlabel('Time (s)')
ylabel("MRP 2")
legend('STK','42','WarpTwin')
nexttile
plot(time_vec,stk.nav_err(3,:),time_vec,deep.mrp_err(3,:),wt.time,wt.nav_err(3,:))
xlabel('Time (s)')
ylabel("MRP 3")
legend('STK','42','WarpTwin')



% load('../Modelspace_AlexN/results/nav_log.csv')
% for i = 1:length(tvec)
%     conv = (1/(1+quat_truth(4,i)))*[quat_truth(1,i); quat_truth(2,i); quat_truth(3,i)];
%     if norm(conv) >= 1.0readtable('../Modelspace_AlexN/results/nav_log.csv')

%         MRP_truth(:,i) = conv*(-1./(conv'*conv));
%     else
%         MRP_truth(:,i) = conv;
%     end
% end
% 
% pvec = nav.m_history(1:3,:);
% bvec = nav.m_history(4:6,:);
% tt = nav.t_history;
% 
% time_vec = tt(1:2:end);
% 
% Pvec = zeros(6,cnt);
% for i = 1:6
%     Pvec(i,:) = sqrt(nav.P_history(i,i,:));
% end
% 
% % figure;
% % t = tiledlayout(4,1);
% % title(t,'Truth Attitude, Quaternions')
% % nexttile
% % plot(tvec,quat_truth(1,1:end-1))
% % ylabel('q1')
% % nexttile
% % plot(tvec,quat_truth(2,1:end-1))
% % ylabel('q2')
% % nexttile
% % plot(tvec,quat_truth(3,1:end-1))
% % ylabel('q3')
% % nexttile
% % plot(tvec,quat_truth(4,1:end-1))
% % ylabel('Scalar part')
% 
% figure
% t = tiledlayout(3,1);
% title(t,'Truth vs. Estimated Attitude (MRP)')
% nexttile
% plot(tt,pvec(1,:))
% hold on
% plot(tvec,MRP_truth(1,:))
% ylabel('MRP 1')
% xlabel('Time (s)')
% legend('Estimate','Truth','Location','best')
% hold off
% grid on
% nexttile
% plot(tt,pvec(2,:))
% hold on
% plot(tvec,MRP_truth(2,:))
% ylabel('MRP 2')
% xlabel('Time (s)')
% legend('Estimate','Truth','Location','best')
% hold off
% grid on
% nexttile
% plot(tt,pvec(3,:))
% hold on
% plot(tvec,MRP_truth(3,:))
% ylabel('MRP 3')
% xlabel('Time (s)')
% legend('Estimate','Truth','Location','best')
% hold off
% grid on
% 
% % figure
% % t = tiledlayout(3,1);
% % title(t,'Estimated bias')
% % nexttile
% % plot(tt,bvec(1,:))
% % grid on
% % nexttile
% % plot(tt,bvec(2,:))
% % grid on
% % nexttile
% % plot(tt,bvec(3,:))
% % grid on
% 
% evec = zeros(3,length(time_vec));
% 
% for i = 0:time_vec(end)
%     evec(:,i+1) = subMRP(MRP_truth(:,i+1),pvec(:,2*i+1));
% end
% 
% figure
% t = tiledlayout(3,1);
% title(t,'Attitude Error (truth - estimate) and 3 sigma covariance (MRP)')
% nexttile
% plot(time_vec,evec(1,:))
% hold on
% plot(tt,3*Pvec(1,:),'r',tt,-3*Pvec(1,:),'r')
% xlabel("MRP 1")
% ylabel("Time (s)")
% legend('MRP 1','$+3\sigma$','$-3\sigma$','Interpreter','latex','Location','best');
% hold off
% grid on
% nexttile
% plot(time_vec,evec(2,:))
% hold on
% plot(tt,3*Pvec(2,:),'r',tt,-3*Pvec(2,:),'r')
% xlabel("MRP 2")
% ylabel("Time (s)")
% legend('MRP 2','$+3\sigma$','$-3\sigma$','Interpreter','latex','Location','best');
% hold off
% grid on
% nexttile
% plot(time_vec,evec(3,:))
% hold on
% plot(tt,3*Pvec(3,:),'r',tt,-3*Pvec(3,:),'r')
% xlabel("MRP 3")
% ylabel("Time (s)")
% legend('MRP 3','$+3\sigma$','$-3\sigma$','Interpreter','latex','Location','best');
% hold off
% grid on
% 
% figure
% t = tiledlayout(3,1);
% title(t,"Truth angular velocity, rad/s")
% nexttile
% plot(tvec,angvel_truth(1,1:end-1))
% xlabel("\")
% grid on
% nexttile
% plot(tvec,angvel_truth(2,1:end-1))
% grid on
% nexttile
% plot(tvec,angvel_truth(3,1:end-1))
% grid on
