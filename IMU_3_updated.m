% ==== Pronation–Supination (Gyro X) with Hesitations + Auto-Stop ====
PORT = "COM8"; BAUDRATE = 115200; WINDOW = 25;
N_CYCLES_TARGET = 10;                

% open serial
try, delete(serialportfind("Port",PORT)); end
if exist('s','var') && isa(s,'serialport') && isvalid(s), delete(s); end
s = serialport(PORT, BAUDRATE, "Timeout", 5);
configureTerminator(s,"CR/LF"); flush(s);

% plot
f = figure('Color','w');
ax = axes(f); hold(ax,'on'); grid(ax,'on');
xlim(ax,[0 WINDOW]); ylim(ax,[-250 250]);
xlabel(ax,'Time (s)'); ylabel(ax,'Gyro X (°/s)');
title(ax,'Pronation–Supination');
h = plot(ax, NaN, NaN, 'LineWidth', 2);
tHUD = text(ax, 0.01, 0.98, ...
    'Hesitations: 0 | Cycles: 0/10 | Speed: 0.00 cyc/s', ...
    'Units','normalized','VerticalAlignment','top', ...
    'FontSize',12,'FontWeight','bold');

% state
t0 = tic; tt = []; yy = [];
bias = 0; alpha = 0.01; restThr = 14;   % auto-zero + near-still (dps)
pauseDur = 0.11;                       % stillness duration to count hesitation (s)
hesitations = 0; stillTime = 0; wasMoving = false; countedThisPause = false;
tPrev = toc(t0);

% --- cycle counter settings (robust) ---
deadband = 15;           % dps around zero to avoid chattering (hysteresis)
peakThr  = 50;           % must see a lobe exceed this magnitude before counting crossing
minGap   = 0.25;         % s: refractory gap between counted zero-crossings
lastSign = 0;            % -1, 0, +1 based on deadbanded sign of signal
lastCrossTime = -Inf;
hadPeakPos = false;
hadPeakNeg = false;
halfCycles = 0;          % two half cycles = one full cycle

% --- NEW: speed tracking state ---
fullCycleTimes = [];     % timestamps of each FULL cycle
prevFullCycles = 0;      % to detect new full cycles
meanSpeedHz = 0;         % cycles per second (updated online)

while isvalid(f)
    L = strtrim(readline(s));                  
    v = sscanf(L,'Gx[dps]: %f');
    if numel(v) ~= 1, continue, end

    tNow = toc(t0); dt = max(0, tNow - tPrev); tPrev = tNow;
    gx = v(1);

    % auto-zero so rest ~ 0 dps
    if abs(gx - bias) < restThr
        bias = (1 - alpha)*bias + alpha*gx;
    end
    gxc = gx - bias;

    % ------------------ hesitation logic ------------------
    if abs(gxc) < restThr
        stillTime = stillTime + dt;
        if wasMoving && ~countedThisPause && stillTime >= pauseDur
            hesitations = hesitations + 1;
            countedThisPause = true;    % latch to avoid double counting same pause
        end
        wasMoving = false;
    else
        stillTime = 0;
        countedThisPause = false;
        wasMoving = true;
    end

    % ------------------ cycle counting ----------------------
    % track whether each side reached a meaningful peak before crossing
    if gxc >=  peakThr, hadPeakPos = true; end
    if gxc <= -peakThr, hadPeakNeg = true; end

    % deadbanded sign to avoid flicker around zero
    if     gxc >  deadband, signNow = +1;
    elseif gxc < -deadband, signNow = -1;
    else,  signNow = 0;
    end

    % count a HALF-cycle on valid sign change with amplitude & time guards
    if signNow ~= 0 && signNow ~= lastSign && (tNow - lastCrossTime) >= minGap
        % only count if we actually visited the opposite side with a real peak
        valid = (signNow == +1 && hadPeakNeg) || (signNow == -1 && hadPeakPos);
        if valid
            halfCycles = halfCycles + 1;
            lastCrossTime = tNow;
            if signNow == +1
                hadPeakNeg = false;
            else
                hadPeakPos = false;
            end
        end
        lastSign = signNow;
    end

    fullCycles = floor(halfCycles/2);

    % -------- NEW: speed computation (online) --------
    if fullCycles > prevFullCycles
        fullCycleTimes(end+1) = tNow;   % log time of each new full cycle
        prevFullCycles = fullCycles;
    end

    if numel(fullCycleTimes) >= 2
        duration = fullCycleTimes(end) - fullCycleTimes(1);      % seconds
        meanSpeedHz = (numel(fullCycleTimes)-1) / max(duration, eps);
    else
        meanSpeedHz = 0;
    end
    % ------------------------------------------------

    % plot buffer
    tt(end+1) = tNow; yy(end+1) = gxc;
    keep = tt >= tNow - WINDOW; tt = tt(keep); yy = yy(keep);
    set(h, 'XData', tt - tt(1), 'YData', yy);

    % --- HUD with speed ---
    set(tHUD, 'String', sprintf( ...
        'Hesitations: %d | Cycles: %d/%d | Speed: %.2f cyc/s', ...
        hesitations, fullCycles, N_CYCLES_TARGET, meanSpeedHz));

    drawnow limitrate

    % -------- auto-stop after target full cycles --------
    if fullCycles >= N_CYCLES_TARGET
        try, delete(s); end
        break
    end
end

% ===== Post-test speed evaluation =====
if numel(fullCycleTimes) >= 2
    [meanSpeedHz_final, speedRatingText] = evaluateRotationSpeed(fullCycleTimes);
    fprintf('Mean rotation speed: %.2f cycles/s -> %s\n', ...
            meanSpeedHz_final, speedRatingText);
else
    fprintf('Not enough cycles to evaluate speed.\n');
end

% ========= Local function: speed rating =========
function [meanHz, ratingText] = evaluateRotationSpeed(cycleTimes)


    if numel(cycleTimes) < 2
        meanHz = 0;
        ratingText = "Speed: not enough data";
        return;
    end

    duration = cycleTimes(end) - cycleTimes(1);           % seconds
    meanHz = (numel(cycleTimes)-1) / max(duration, eps);  % cycles per second

  
    % Thresholds are example values, not clinical 
    if meanHz >= 1.5
        ratingText = "0: Normal speed";
    elseif meanHz >= 1.0
        ratingText = "1: Slight slowing";
    elseif meanHz >= 0.5
        ratingText = "2: Mild slowing";
    elseif meanHz >= 0.25
        ratingText = "3: Moderate slowing";
    else
        ratingText = "4: Severe slowing";
    end
end
