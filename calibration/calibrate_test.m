function [r] = calibrate_test()

    ps = dlmread('sensor_points');
    pv = dlmread('vehicle_points');
    R = dlmread('rotation');
    t = dlmread('translation');

    ps = ps';
    ps_ = R'*ps;
    ps_ = ps_';

    ps_(:,1) = ps_(:,1) + t(1);
    ps_(:,2) = ps_(:,2) + t(2);
    ps_(:,3) = ps_(:,3) + t(3);
        
    r = pv - ps_;

    scatter3(ps_(:,1), ps_(:,2), ps_(:,3));
    hold on;
    scatter3(pv(:,1), pv(:,2), pv(:,3), 'r');

end


