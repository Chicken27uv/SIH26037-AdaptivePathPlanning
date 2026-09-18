function map = M6_CreateMap()

map = binaryOccupancyMap(60,30,2);

[x1,y1] = meshgrid(28:0.5:33,0:0.5:20);
[x2,y2] = meshgrid(28:0.5:33,25:0.5:30);

setOccupancy(map,[x1(:) y1(:)],1)
setOccupancy(map,[x2(:) y2(:)],1)

end