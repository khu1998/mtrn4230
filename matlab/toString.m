function msgString = toString(positions)
    msgString="";
    shape = size(positions);
    num_entries = shape(2);
    for i = 1:num_entries
        p = positions(i);
        msgString = msgString + sprintf('%d,%s,%0.5f,%0.5f,%0.5f|',p.index,p.type,p.gp3d.x,p.gp3d.y,p.gp3d.z);
    end
end