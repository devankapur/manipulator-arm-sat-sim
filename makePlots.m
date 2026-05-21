function plots = makePlots(t, sol, Tc, Marm, armTc, sim_type)

switch sim_type
    case 1
        plot1 = figure(1);

        p1 = subplot(2,4,1);
        p2 = subplot(2,4,2);
        p3 = subplot(2,4,3);
        p4 = subplot(2,4,4);
        p5 = subplot(2,4,5);

        p1.Position = [0.0200, 0.5200, 0.2250, 0.4400];
        p2.Position = [0.2650, 0.5200, 0.2250, 0.4400];
        p3.Position = [0.5150, 0.5200, 0.4725, 0.4400];
        p4.Position = [0.0200, 0.0300, 0.4725, 0.4400];
        p5.Position = [0.5150, 0.0300, 0.4725, 0.4400];

        plot(p1, t, sol(:,1:3));
        plot(p2, t, sol(:,11:13));
        plot(p4, t, sol(:,4:6));
        plot(p3, t, sol(:,7:10));
        plot(p5, t, Tc);

        title(p1, 'Satellite Linear Velocity');
        title(p2, 'Satellite Linear Position');
        title(p3, 'Satellite Quaternion');
        title(p4, 'Satellite Angular Velocity');
        title(p5, 'Satellite Controller Control Torque');

        plots = plot1;
    case 2
        plot1 = figure(1);

        p1 = subplot(2,4,1);
        p2 = subplot(2,4,2);
        p3 = subplot(2,4,3);
        p4 = subplot(2,4,4);
        p5 = subplot(2,4,5);
        p6 = subplot(2,4,6);
        p7 = subplot(2,4,7);
        p8 = subplot(2,4,8);

        p1.Position = [0.0200, 0.5200, 0.2250, 0.4400];
        p2.Position = [0.2650, 0.5200, 0.2250, 0.4400];
        p3.Position = [0.5150, 0.5200, 0.2250, 0.4400];
        p4.Position = [0.7650, 0.5200, 0.2250, 0.4400];
        p5.Position = [0.0200, 0.0300, 0.2250, 0.4400];
        p6.Position = [0.2650, 0.0300, 0.2250, 0.4400];
        p7.Position = [0.5150, 0.0300, 0.2250, 0.4400];
        p8.Position = [0.7650, 0.0300, 0.2250, 0.4400];
        
        plot(p1, t, sol(:,1:3));
        plot(p2, t, sol(:,4:6));
        plot(p3, t, sol(:,7:10));
        plot(p4, t, sol(:,14:16));
        plot(p5, t, sol(:,11:13));
        plot(p6, t, armTc);
        plot(p7, t, Marm);
        plot(p8, t, sol(:,17:19));
        
        title(p1, 'Satellite Linear Velocity');
        title(p2, 'Satellite Angular Velocity');
        title(p3, 'Satellite Quaternion');
        title(p4, 'Arm Joint Angles');
        title(p5, 'Satellite Linear Position');
        title(p6, 'Arm Controller Control Torque');
        title(p7, 'Arm Torque on Satellite');
        title(p8, 'Arm Joint Velocities')
        
        plots = plot1;
    case 3
        plot1 = figure(1);

        p1 = subplot(2,4,1);
        p2 = subplot(2,4,2);
        p3 = subplot(2,4,3);
        p4 = subplot(2,4,4);
        p5 = subplot(2,4,5);
        p6 = subplot(2,4,6);
        p7 = subplot(2,4,7);
        p8 = subplot(2,4,8);

        p1.Position = [0.0200, 0.5200, 0.2250, 0.4400];
        p2.Position = [0.2650, 0.5200, 0.2250, 0.4400];
        p3.Position = [0.5150, 0.5200, 0.2250, 0.4400];
        p4.Position = [0.7650, 0.5200, 0.2250, 0.4400];
        p5.Position = [0.0200, 0.0300, 0.2250, 0.4400];
        p6.Position = [0.2650, 0.0300, 0.2250, 0.4400];
        p7.Position = [0.5150, 0.0300, 0.2250, 0.4400];
        p8.Position = [0.7650, 0.0300, 0.2250, 0.4400];
        
        plot(p1, t, sol(:,1:3));
        plot(p2, t, sol(:,4:6));
        plot(p3, t, sol(:,7:10));
        plot(p4, t, sol(:,17:19));
        plot(p5, t, sol(:,11:13));
        plot(p6, t, Tc);
        plot(p7, t, Marm);
        plot(p8, t, sol(:,20:22));
        
        title(p1, 'Satellite Linear Velocity');
        title(p2, 'Satellite Angular Velocity');
        title(p3, 'Satellite Quaternion');
        title(p4, 'Arm Joint Angles');
        title(p5, 'Satellite Linear Position');
        title(p6, 'Satellite Controller Control Torque');
        title(p7, 'Arm Torque on Satellite');
        title(p8, 'Arm Joint Velocities')
        
        plots = plot1;
    case 4
        plot1 = figure(1);

        p1 = subplot(2,4,1);
        p2 = subplot(2,4,2);
        p3 = subplot(2,4,3);
        p4 = subplot(2,4,4);
        p5 = subplot(2,4,5);
        p6 = subplot(2,4,6);
        p7 = subplot(2,4,7);
        p8 = subplot(2,4,8);

        p1.Position = [0.0200, 0.5200, 0.2250, 0.4400];
        p2.Position = [0.2650, 0.5200, 0.2250, 0.4400];
        p3.Position = [0.5150, 0.5200, 0.2250, 0.4400];
        p4.Position = [0.7650, 0.5200, 0.2250, 0.4400];
        p5.Position = [0.0200, 0.0300, 0.2250, 0.4400];
        p6.Position = [0.2650, 0.0300, 0.2250, 0.4400];
        p7.Position = [0.5150, 0.0300, 0.2250, 0.4400];
        p8.Position = [0.7650, 0.0300, 0.2250, 0.4400];
        
        plot(p1, t, sol(:,1:3));
        plot(p2, t, sol(:,4:6));
        plot(p3, t, sol(:,7:10));
        plot(p4, t, sol(:,17:19));
        plot(p5, t, sol(:,11:13));
        plot(p6, t, armTc);
        plot(p7, t, Marm);
        plot(p8, t, sol(:,20:22));
        
        title(p1, 'Satellite Linear Velocity');
        title(p2, 'Satellite Angular Velocity');
        title(p3, 'Satellite Quaternion');
        title(p4, 'Arm Joint Angles');
        title(p5, 'Satellite Linear Position');
        title(p6, 'Arm Controller Control Torque');
        title(p7, 'Arm Torque on Satellite');
        title(p8, 'Arm Joint Velocities')
        
        plots = plot1;
    otherwise
end

end