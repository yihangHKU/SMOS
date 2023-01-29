function label = Flabel(ind,st,frame)
%FLABEL Summary of this function goes here
%   Detailed explanation goes here
pnt  = FKloa(st, frame);    
file_w = fopen(sprintf('%s/%06d.label', st.dr.pre, frame - 1), 'w');
label = zeros(size(pnt.pts,1), 1);
for i = 1:size(ind,1)
    label(ind(i,1)) = 1; 
end
for i = 1:size(label,1)
    if label(i) == 1
        fwrite(file_w, 251, 'int');
    else
        fwrite(file_w, 9, 'int');
    end      
end

fclose(file_w);
end

function pts    = FKloa(st, frame)

% simple load
% output: pts.[pts ptn rtn trn] all points 

%% transformation matrixes [rotation 3x3, translation 3x1]
% transform       = st.dt.pose(:, :, frame);                                     % transformation matrix in camera coordinate
% pts.rtn         = transform(1:3, 1:3);                                         % rotation    3x3
% pts.trn         = transform(1:3, 4);                                           % translation 3x1
transform = st.dt.pose(frame,:);
pts.rtn = [transform(1:3);transform(5:7);transform(9:11)];
pts.trn = [transform(4);transform(8);transform(12)];
%% velodyne points [x, y, z, r] total number of pointsx4
fid.pts         = fopen(sprintf('%s/%06d.bin', st.dr.pts, frame - 1), 'rb');   % read from directory of points (number of frames in each seq.)
velodyne        = fread(fid.pts, [4 inf], 'single')';                          % velodyne points [x, y, z, r] (total number of pointsx4)
fclose(fid.pts);                                                               % close fid
pts.pts         = velodyne(:, 1:3);                                            % velodyne points [x, y, z] (total number of pointsx3)
pts.ptn         = pts.pts * pts.rtn' + repmat(pts.trn', size(pts.pts, 1), 1);  % transformed velodyne points (Xp = RX + T)


end