function msgString = toString(positions)
    msgString="";
    shape = size(positions);
    num_entries = shape(1);
    for i = 1:num_entries
        p = positions(i,:);
        msgString = msgString + sprintf('%d,%s,%0.5f,%0.5f,%0.5f|',p{1},p{2},p{3},p{4},p{5});
    end
end