function path = reverse(path,start,last)
while(start<last)
    temp=[];
    temp(1,1) = path(start,1);
    temp(1,2) = path(start,2);
    path(start,1) = path(last,1);
    path(start,2) = path(last,2);
    path(last,1) = temp(1,1);
    path(last,2) = temp(1,2);
    last = last-1;
    start = start+1;
end