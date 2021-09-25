
rosshutdown;

clear all; close all; clc
rosinit

global point;
global I;
global ox_;
global oy_;
global res;
global w;
global h;
global pts_; % orig pts locs
global pts
global orig;

point_sub = rossubscriber('/detected_points', @pointCallback) ;
map_sub   = rossubscriber('/move_base/global_costmap/costmap', @costmapCallback);


figure(1); clf; hold on
imshow(I); hold on
plot(orig(1), orig(2), 'bo')
for idx=1:size(pts_,1)
    hold on
   plot( (pts_(idx,1) - ox_)/res , (pts_(idx,2) - oy_ )/res, 'r+') ; 
end

pts_ = [];



