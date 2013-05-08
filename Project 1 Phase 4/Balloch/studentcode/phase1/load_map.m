function map = load_map(filename, xy_res, z_res, margin)
% LOAD_MAP Load a map from disk.
%  MAP = LOAD_MAP(filename, xy_res, z_res, margin).  Creates an occupancy grid
%  map where a node is considered fill if it lies within 'margin' distance of
%  on abstacle.

% map = zeros(0, 0);
global MapData

quadwidth = 0.25; %radius. make =0 for point 

MapData.xy_res=xy_res;
MapData.z_res=z_res;
MapData.margin=margin;

map=[];

fileID=fopen(filename);
format = '%s %f %f %f %f %f %f %f %f %f';
RawMapData=textscan(fileID,format, 'commentStyle', '#');
fclose(fileID);

args=size(RawMapData{1},1);

% boundary first
for i=1:args
    if strcmp(RawMapData{1}{i},'boundary')
        boundidx=i;
    map=zeros( ceil((RawMapData{5}(i)-RawMapData{2}(i))/xy_res),...
               ceil((RawMapData{6}(i)-RawMapData{3}(i))/xy_res) ,...
               ceil((RawMapData{7}(i)-RawMapData{4}(i))/z_res) );
    else
%        throw('Incorrect map file: no boundary');
    end
end


MapData.boundidx=boundidx;
MapData.minx=RawMapData{2}(MapData.boundidx);
MapData.miny=RawMapData{3}(MapData.boundidx);
MapData.minz=RawMapData{4}(MapData.boundidx);
MapData.maxx=RawMapData{5}(MapData.boundidx);
MapData.maxy=RawMapData{6}(MapData.boundidx);
MapData.maxz=RawMapData{7}(MapData.boundidx);


% blocks second, just in case out of order
blockids=[];

for i=1:args
    if strcmp(RawMapData{1}{i},'block')
        minblock(1)=ceil( ( RawMapData{2}(i)-(margin+quadwidth)-RawMapData{2}(boundidx) )/xy_res );
        minblock(2)=ceil( ( RawMapData{3}(i)-(margin+quadwidth)-RawMapData{3}(boundidx) )/xy_res );
        minblock(3)=ceil( ( RawMapData{4}(i)-margin-RawMapData{4}(boundidx) )/z_res );
        maxblockx=ceil( ( RawMapData{5}(i)+(margin+quadwidth) -RawMapData{2}(boundidx) )/xy_res);
        maxblocky=ceil( ( RawMapData{6}(i)+(margin+quadwidth) -RawMapData{3}(boundidx) )/xy_res);
        maxblockz=ceil( (  RawMapData{7}(i)+margin -RawMapData{4}(boundidx) )/z_res );
        
        minblock(minblock <=1)=1;
        maxblockx(maxblockx >= size(map,1))=size(map,1);
        maxblocky(maxblocky >= size(map,2))=size(map,2);
        maxblockz(maxblockz >= size(map,3))=size(map,3);
        map( minblock(1):maxblockx,...
             minblock(2):maxblocky,...
             minblock(3):maxblockz )= 1;

    %drawing data; faces verts
        blockids=[blockids i];
    end
end

%% Data for patching
SZ=size(blockids,2);

MapData.verts=[ 
    [RawMapData{2}(blockids) RawMapData{3}(blockids) RawMapData{4}(blockids)];...
    [RawMapData{2}(blockids) RawMapData{6}(blockids) RawMapData{4}(blockids)];...
    [RawMapData{5}(blockids) RawMapData{6}(blockids) RawMapData{4}(blockids)];...
    [RawMapData{5}(blockids) RawMapData{3}(blockids) RawMapData{4}(blockids)];...
    
    [RawMapData{2}(blockids) RawMapData{3}(blockids) RawMapData{7}(blockids)];...
    [RawMapData{2}(blockids) RawMapData{6}(blockids) RawMapData{7}(blockids)];...
    [RawMapData{5}(blockids) RawMapData{6}(blockids) RawMapData{7}(blockids)];...
    [RawMapData{5}(blockids) RawMapData{3}(blockids) RawMapData{7}(blockids)];...
    ];


blkfcverts=repmat([1:SZ]',1,4);

MapData.faces= [
    bsxfun(@plus, [0 SZ 2*SZ 3*SZ],blkfcverts);... %z(-)
    bsxfun(@plus, [4*SZ 5*SZ 6*SZ 7*SZ],blkfcverts);... %z(+)
    bsxfun(@plus, [0 4*SZ 5*SZ 1*SZ],blkfcverts);... %x(-)
    bsxfun(@plus, [2*SZ 6*SZ 7*SZ 3*SZ],blkfcverts);... %x(+)
    bsxfun(@plus, [SZ 5*SZ 6*SZ 2*SZ],blkfcverts);... %y(-)
    bsxfun(@plus, [3*SZ 7*SZ 4*SZ 0],blkfcverts);... %y(+)
    ]; 

%because colors repeats, when making the patches, just concatonate 6 copies
MapData.colors=[RawMapData{8}(blockids) RawMapData{9}(blockids) RawMapData{10}(blockids)]./255 ;

end
