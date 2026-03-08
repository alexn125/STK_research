clear all
close all

% load('Sim_Data/results.mat')
load('Sim_Data/navonly_results.mat')
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

Pvec = zeros(6,cnt);
for i = 1:6
    Pvec(i,:) = sqrt(nav.P_history(i,i,:));
end

figure;
t = tiledlayout(4,1);
title(t,'Truth Attitude, Quaternions')
nexttile
plot(tvec,quat_truth(1,1:end-1))
ylabel('q1')
nexttile
plot(tvec,quat_truth(2,1:end-1))
ylabel('q2')
nexttile
plot(tvec,quat_truth(3,1:end-1))
ylabel('q3')
nexttile
plot(tvec,quat_truth(4,1:end-1))
ylabel('Scalar part')

figure
t = tiledlayout(3,1);
title(t,'Truth vs. Estimated Attitude (MRP)')
nexttile
plot(tt,pvec(1,:))
hold on
plot(tvec,MRP_truth(1,:))
ylabel('MRP 1')
xlabel('Time (s)')
legend('Estimate','Truth','Location','best')
hold off
grid on
nexttile
plot(tt,pvec(2,:))
hold on
plot(tvec,MRP_truth(2,:))
ylabel('MRP 2')
xlabel('Time (s)')
legend('Estimate','Truth','Location','best')
hold off
grid on
nexttile
plot(tt,pvec(3,:))
hold on
plot(tvec,MRP_truth(3,:))
ylabel('MRP 3')
xlabel('Time (s)')
legend('Estimate','Truth','Location','best')
hold off
grid on

figure
t = tiledlayout(3,1);
title(t,'Estimated bias')
nexttile
plot(tt,bvec(1,:))
grid on
nexttile
plot(tt,bvec(2,:))
grid on
nexttile
plot(tt,bvec(3,:))
grid on

evec = zeros(3,length(time_vec));

for i = 0:time_vec(end)
    evec(:,i+1) = MRP_truth(:,i+1) - pvec(:,2*i+1);
end

figure
t = tiledlayout(3,1);
title(t,'Attitude Error (truth - estimate) and 3 sigma covariance (MRP)')
nexttile
plot(time_vec,evec(1,:))
hold on
plot(tt,3*Pvec(1,:),'r',tt,-3*Pvec(1,:),'r')
legend('MRP 1','$+3\sigma$','$-3\sigma$','Interpreter','latex','Location','best');
legend()
hold off
grid on
nexttile
plot(time_vec,evec(2,:))
hold on
plot(tt,3*Pvec(2,:),'r',tt,-3*Pvec(2,:),'r')
legend('MRP 2','$+3\sigma$','$-3\sigma$','Interpreter','latex','Location','best');
hold off
grid on
nexttile
plot(time_vec,evec(3,:))
hold on
plot(tt,3*Pvec(3,:),'r',tt,-3*Pvec(3,:),'r')
legend('MRP 3','$+3\sigma$','$-3\sigma$','Interpreter','latex','Location','best');
hold off
grid on

figure
t = tiledlayout(3,1);
title(t,"Truth angular velocity, rad/s")
nexttile
plot(tvec,angvel_truth(1,1:end-1))
grid on
nexttile
plot(tvec,angvel_truth(2,1:end-1))
grid on
nexttile
plot(tvec,angvel_truth(3,1:end-1))
grid on
