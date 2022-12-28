clc
clear
close all
load('data_test_video.mat')
%%
clear Fvid vid
mygreen = [26, 186, 63]/255;
data_vid = data_2;
close all
opt_font= {'interpreter','latex','fontsize',11};
zone_size_xy = 1.2;
z_max= 1.5;
sizeFrame=[1100 1920]/3.25;


h=figure('Position',[50 100, sizeFrame ]);
set(gcf,'color','w');
xlim([-zone_size_xy zone_size_xy]);ylim([-zone_size_xy zone_size_xy]);zlim([-0.001 z_max])
grid on
hold on
view(-10,12)
xlabel('x [m]',opt_font{:})
ylabel('y [m]',opt_font{:})
zlabel('z [m]',opt_font{:})
for i = 1:size(data_vid.ref,1)
    plotref=plot3(data_vid.ref(:,1),data_vid.ref(:,2),data_vid.ref(:,3),'b--');
    hold on
    scatter3(data_vid.full_data(1,1),data_vid.full_data(1,2),data_vid.full_data(1,3),60,'filled');
    set(gcf,'color','w');
    view(-10,12)
    xlim([-zone_size_xy zone_size_xy]);ylim([-zone_size_xy zone_size_xy]);zlim([-0.001 z_max])
    grid on
    plotref=plot3(data_vid.full_data(1:i,1),data_vid.full_data(1:i,2),data_vid.full_data(1:i,3),'color',mygreen,'linewidth',2);
    drone_plot_w(data_vid.full_data(i,1),data_vid.full_data(i,2),...
        data_vid.full_data(i,3),data_vid.phi(i),data_vid.theta(i),...
        data_vid.psi(i)*pi/180+pi/4,0.15);
    xlabel('x [m]',opt_font{:})
    ylabel('y [m]',opt_font{:})
    zlabel('z [m]',opt_font{:})
        hold off

    drawnow
    Fvid(i) = getframe(gcf);
end

vid = VideoWriter('Video_sim_drone','Uncompressed AVI');
vid.FrameRate=13.33;
open(vid)
writeVideo(vid,Fvid);
close(vid)
