function [fc, pm, gm] = loop_margins(Lw, f)
%LOOP_MARGINS Crossover, phase margin and gain margin from a frequency grid.
%   [fc, pm, gm] = loop_margins(Lw, f) takes L(j*2*pi*f) with one loop per
%   column of Lw (nf-by-M) and the grid f in Hz. Returns 1-by-M vectors:
%     fc  last 0 dB crossing (Hz), NaN if none
%     pm  phase margin (deg) at fc
%     gm  gain margin (dB): smallest at a -180 (mod 360) crossing above fc,
%         Inf if there is none (the briefing's "-")
%   Crossings are interpolated on log frequency.

f  = f(:);
lf = log(f);
lm = log(abs(Lw));
ph = unwrap(angle(Lw))*180/pi;
M  = size(Lw, 2);
fc = nan(1, M);
pm = nan(1, M);
gm = inf(1, M);
for k = 1:M
    i = find(lm(1:end-1, k).*lm(2:end, k) <= 0, 1, 'last');
    if isempty(i), continue; end
    a = lm(i, k)/(lm(i, k) - lm(i+1, k));
    fc(k) = exp(lf(i) + a*(lf(i+1) - lf(i)));
    phc   = ph(i, k) + a*(ph(i+1, k) - ph(i, k));
    pm(k) = mod(phc + 360, 360) - 180;

    n = floor((ph(i:end, k) + 180)/360);
    j = find(diff(n) ~= 0) + i - 1;
    for jj = j(:)'
        target = 360*max(n(jj-i+1), n(jj-i+2)) - 180;
        b = (ph(jj, k) - target)/(ph(jj, k) - ph(jj+1, k));
        gm(k) = min(gm(k), -20/log(10)*(lm(jj, k) + b*(lm(jj+1, k) - lm(jj, k))));
    end
end
end
