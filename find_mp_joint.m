%FIND_MP_JOINT  Report which Revolute joint spins the left MP gear (18T body).
model = 'onlyJoints_driven';
if isempty(find_system('SearchDepth',0,'Name',model)), load_system(model); end

gearBody = [model '/Monopole_Drive_Assembly_1/abenics_monopole_gear_18tmod2_v8_v1_1_RIGID'];
h0 = getSimulinkBlockHandle(gearBody);
if h0 <= 0, error('gear body not found: %s', gearBody); end

% Breadth-first walk over CONNECTION LINES (incl. Simscape physical ports)
% until we hit a Revolute joint. Report ALL joints found, nearest first.
visited = h0;  queue = h0;  hops = 0;  qhop = 0;  jointsFound = {};
while ~isempty(queue)
    b = queue(1);  queue(1) = [];  hb = qhop(1); qhop(1) = [];
    st = ''; try, st = get_param(b,'SourceType'); catch, end
    if contains(st,'Revolute')
        jointsFound{end+1} = sprintf('[%d hops] %s', hb, getfullname(b)); %#ok<AGROW>
    end
    lh = get_param(b,'LineHandles');
    lines = [lh.LConn(:); lh.RConn(:); lh.Inport(:); lh.Outport(:)];
    for L = lines(:)'
        if L <= 0, continue; end
        for nb = [get_param(L,'SrcBlockHandle'), get_param(L,'DstBlockHandle')]
            if nb > 0 && ~any(visited == nb)
                visited(end+1) = nb;  queue(end+1) = nb;  qhop(end+1) = hb+1; %#ok<AGROW>
            end
        end
    end
end

if isempty(jointsFound)
    disp('Still no Revolute joint reachable from the 18T MP-gear body.');
else
    disp('Revolute joints reachable from the left 18T MP-gear body:');
    for i = 1:numel(jointsFound), disp(['   ' jointsFound{i}]); end
end
