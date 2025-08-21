close all

% Turn {SaveVideo} to 1 to save the video as {VideoName}.mp4 
SaveVideo = 0;
VideoName = 'Name';

% Wheel rectangle coordinates 
% lw : wheel length
% ww : wheel width
wx=[-lw/2 -lw/2 lw/2 lw/2 -lw/2];
wy=[-ww/2 ww/2 ww/2 -ww/2 -ww/2];
fwr=zeros(2,5);rwr=zeros(2,5);

% car rectangle coordinates 
% Dr : rear axle to Center Of Mass (COM) distance
% Df : front axle to Center Of Mass (COM) distance
% w : car width 
cx=[-Dr -Dr Df Df -Dr];
cy=[-w/2 w/2 w/2 -w/2 -w/2];
cr=zeros(2,5);

% Create VideoWriter Object to recored and save video
if SaveVideo == 1
    myWriter=VideoWriter([VideoName '.mp4'],'MPEG-4');
    myWriter.FrameRate=10;
    open(myWriter);
end


num=length(Time);

%drawing loop
for i=1:10:num
    cla
    R=[cos(Theta(i)) -sin(Theta(i));
        sin(Theta(i)) cos(Theta(i))];
    
    %  cr : rotated car
    %  rwr : rotated rear wheel 
    cr = R*[cx;cy];
    rwr= R*[wx;wy];
    
    R=[cos(Theta(i)+Psi(i)) -sin(Theta(i)+Psi(i));
        sin(Theta(i)+Psi(i)) cos(Theta(i)+Psi(i))];
    % fwr : rotated front wheel (steering wheel)
    fwr = R*[wx;wy];
    
    % draw target trajectory 
    if exist('Xt','var') == 1
        plot(Xt,Yt,'r')
        hold on
        plot(Xt(i),Yt(i),'go',Xt(i),Yt(i),'rx');
    end
    % draw robot and its trajectory	
    plot(X,Y,'k');
    grid

    hold on
    fill(cr(1,:)+X(i),cr(2,:)+Y(i),'b');
    fill(fwr(1,:)+X(i)+Df*cos(Theta(i)),fwr(2,:)+Y(i)+Df*sin(Theta(i)),'k');
    fill(rwr(1,:)+X(i)-Dr*cos(Theta(i)),rwr(2,:)+Y(i)-Dr*sin(Theta(i)),'k');
    hold off

    % figure display
    window_size=4;
    axis([X(i)-window_size,X(i)+window_size,Y(i)-window_size,Y(i)+window_size]);

    drawnow
    if SaveVideo == 1
        F=getframe;
        writeVideo(myWriter,F);
    end
    
end

if SaveVideo == 1
    close(myWriter);
end

