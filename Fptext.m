function Fg_points = Fptext(In, prm, st, frame)
%FPTEXT Summary of this function goes here
%   Detailed explanation goes here
pnt           = FKloa(st, frame);                       % load points
pnt           = FKcrp(pnt, st, frame);                  % crop points to the inside the local grid and image
Fg_points     = Fvox(prm, st, pnt.pts, pnt.ind, In);                 % remove ground and voxelize points

end

function pts  = FKloa(st, frame)

% simple load
% output: pts.[pts ptn rtn trn] all points 

%% transformation matrixes [rotation 3x3, translation 3x1]hst
% transform     = st.dt.pose(:, :, frame);                                     % transformation matrix in camera coordinate
% pts.rtn       = transform(1:3, 1:3);                                         % rotation    3x3
% pts.trn       = transform(1:3, 4);                                           % translation 3x1
transform = st.dt.pose(frame,:);
pts.rtn = [transform(1:3);transform(5:7);transform(9:11)];
pts.trn = [transform(4);transform(8);transform(12)];
%% velodyne points [x, y, z, r] total number of pointsx4
fid.pts       = fopen(sprintf('%s/%06d.bin', st.dr.pts, frame - 1), 'rb');   % read from directory of points (number of frames in each seq.)
velodyne      = fread(fid.pts, [4 inf], 'single')';                          % velodyne points [x, y, z, r] (total number of pointsx4)
fclose(fid.pts);                                                               % close fid
pts.pts       = velodyne(:, 1:3);                                            % velodyne points [x, y, z] (total number of pointsx3)
pts.ptn       = pts.pts * pts.rtn' + repmat(pts.trn', size(pts.pts, 1), 1);  % transformed velodyne points (Xp = RX + T)
pts.ind       = linspace(1,size(pts.pts,1), size(pts.pts,1))';
end

function hst    = FKcrp(hst, st, frame)

% crop points to the inside the local grid and image
% output: hst.[pts] that is croped integrated points

%% velodyne points [x, y, z, r total number of pointsx4]
ins.grd         = ((hst.pts(:,1) > st.vm.xb) & (hst.pts(:,1) < st.vm.xf) & ...       % filter points to inside/outside the local grid
                  (hst.pts(:,2) > st.vm.yr) & (hst.pts(:,2) < st.vm.yl) & ...
                  (hst.pts(:,3) > st.vm.zd) & (hst.pts(:,3) < st.vm.zu));
hst.pts         = hst.pts(ins.grd, 1:3);           % velodyne points in local grid  
hst.ind         = hst.ind(ins.grd, 1);
%% incorporate image and color data [pts col ref pxs]
pixel           = hst.pts * st.dt.clb;                                               % velodyne points on image plane
pixel(:, 1)     = pixel(:, 1)./pixel(:, 3); pixel(:, 2) = pixel(:, 2)./pixel(:, 3);  % point's x & y are cor. to image's c & nr - r (nr: nnumber of raws)
pixel           = round(pixel(:, 1:2));                                              % interpolate (it is not that much precise, round is enough!)
image           = imread(sprintf('%s/%06d.png', st.dr.img, frame - 1));              % load image (number of frames in each seq.)
ins.img         = (pixel(:, 1) >= 1) & (pixel(:, 1) <= size(image, 2)) & (pixel ...  % index of pixels inside grid and image
                  (:, 2) >= 1) & (pixel(:, 2) <= size(image, 1));
hst.pts         = hst.pts(ins.img, :);                                               % velodyne points in local grid & image
hst.ind         = hst.ind(ins.img, 1);
end

function Fg_points = Fvox(prm, st, input, index, In)

%% remove ground points 
pts.pts        = [];
pts.ind        = [];
for pci        = 1 : st.rd.no                                                                  % index of the first piece : index of the last piece
sp             = st.vm.xb + (pci - 1) * st.rd.pc;                                              % start of the current piece (sp) x   
ep             = sp + st.rd.pc;                                                                % end of the current piece (ep) x
pc             = input((input(:, 1) > sp) & (input(:, 1) < ep), :);                            % take the current piece's points
index_current  = index((input(:, 1) > sp) & (input(:, 1) < ep), :);
pln            = prm(pci, :);                                                                  % take the current estimated piece's parameter
nrm            = cross([0 0 pln(3)] - [1 1 sum(pln)], [0 0 pln(3)] - [0 1 pln(2) + pln(3)]);   % surface normal (the slope of normal line) 
t              = (pc(:, 3) - pln(1) .* pc(:,1) - pln(2) * pc(:, 2) - pln(3)) ...               % t is a variable
                 ./ ( pln(1) .* nrm(1)+ pln(2) .* nrm(2) - nrm(3));
pp             = [pc(:, 1) + nrm(1) .* t, pc(:, 2) + nrm(2) * t, pc(:, 3) + nrm(3) * t];       % projected points on the surface
id             = ((pc(:, 3) - pp(:, 3)) < st.rd.rm) | (abs((pc(:, 3) - pp(:, 3))) < st.rd.rm); % remove points under height                      
pc(id, :)      = []; 
index_current(id, :) = [];
pts.pts        = [pts.pts; pc];                                                                % velodyne points in local grid
pts.ind        = [pts.ind; index_current];
end
%% voxelize points 
pts.idx        = floor([(pts.pts(:,1) - st.vm.xb)/st.vx.x + 1, (pts.pts(:,2) - ...             % quantize and transform point's index 
                 st.vm.yr)/st.vx.y + 1, (pts.pts(:,3) - st.vm.zd) / st.vx.z + 1]);
pts.idx(pts.idx(:, 1) > st.vx.ix, 1) = st.vx.ix;            
pts.idx(pts.idx(:, 2) > st.vx.iy, 2) = st.vx.iy; 
pts.idx(pts.idx(:, 3) > st.vx.iz, 3) = st.vx.iz;
Fg_points.pts = [];
Fg_points.ind = [];
for i = 1 : size(pts.idx,1)
    for j = 1 : size(In.pts.uni)
        if pts.idx(i,1) == In.pts.uni(j,1) && pts.idx(i,2) == In.pts.uni(j,2) && pts.idx(i,3) == In.pts.uni(j,3)
            Fg_points.pts = [Fg_points.pts;pts.pts(i,:)];
            Fg_points.ind = [Fg_points.ind;pts.ind(i,:)];
            break;
        end
    end
end
fprintf('Fg_points size')
size(Fg_points.pts)
end

