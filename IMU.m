
PORT = "COM8"; BAUDRATE = 115200; WINDOW = 25;  
% opens serial 
 delete(serialportfind("Port",PORT)); 

s = serialport(PORT, BAUDRATE, "Timeout", 2);
configureTerminator(s,"CR/LF"); flush(s);

% plot
f = figure; 
x = axes(f);
hold(x,'on'); 
grid(x,'on');
xlim(x,[0 WINDOW]); 
ylim(x,[-200 200]);

xlabel(x,'Time (s)');
ylabel(x,'Gyro X (°/s)');
h = plot(x,nan,nan,'LineWidth',2);
title( 'Pronation and Supination')


t0 = tic; bias = 0; stillThr = 11; tt=[]; yy=[]; alpha=0;
while isvalid(f)
L = strtrim(readline(s));                        
    v = sscanf(L,'G[dps]: %f %f %f | A[g]: %f %f %f');
    t  = toc(t0);
    gx = v(1);                 % x-axis                     

    % zero when nearly still so rest 0 dps
    if abs(gx - bias) < stillThr
        bias = (1 - alpha)*bias + alpha*gx;
    end
    gxc = gx - bias;                                

    tt(end+1) = t; yy(end+1) = gxc;
    keep = tt >= t - WINDOW; tt = tt(keep); yy = yy(keep);

    set(h,'XData', tt - tt(1), 'YData', yy);
    
    drawnow limitrate
end
