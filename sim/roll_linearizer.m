function [sl, pt] = roll_linearizer(mdl)
%ROLL_LINEARIZER slLinearizer for roll_axis with small-signal block models.
%   [sl, pt] = roll_linearizer('roll_axis') returns the linearizer and a
%   struct of analysis-point names:
%     pt.U   controller output (Roll PID cell output 1)
%     pt.FB  rate feedback into the PID cell (FB select output)
%     pt.E, pt.P, pt.I, pt.D  rate error and the P, I, D stage outputs
%   The sampled and quantised blocks are replaced by the models the
%   briefing's loop budget uses: quantisers -> 1, firmware ZOH -> a pure
%   delay of Ts_fw/2. Transport delays are kept exact.

if nargin < 1, mdl = 'roll_axis'; end
load_system(mdl);

opt = linearizeOptions('UseExactDelayModel', 'on');
sl  = slLinearizer(mdl, opt);

q  = find_system(mdl, 'LookUnderMasks', 'all', 'BlockType', 'Quantizer');
z  = find_system(mdl, 'LookUnderMasks', 'all', 'BlockType', 'ZeroOrderHold');
Ts = evalin('base', 'Ts_fw');
sl.BlockSubstitutions = [struct('Name', q, 'Value', 1); ...
                         struct('Name', z, 'Value', ss(1, 'InputDelay', Ts/2))];

cellPath = [mdl '/Roll PID cell'];
blocks = {cellPath, [mdl '/FB select'], [cellPath '/U201A rails'], ...
          [cellPath '/U201B rails'], [cellPath '/U201C rails'], [cellPath '/U201D rails']};
addPoint(sl, blocks, ones(1, numel(blocks)));
names = strcat(blocks, '/1');
pt = cell2struct(names(:), {'U'; 'FB'; 'E'; 'P'; 'I'; 'D'});
end
