
map_sub = rossubscriber('/tesse0/projected_map') ;

while 1
    map = receive(map_sub) ;
    I_ = reshape(map.Data, map.Info.Width, map.Info.Height  )' ;
    idx_unknown = find(I_ < 0 );
    idx_obs = find(I_ == 100 ) ;
    I = uint8(I_);
    I(idx_unknown) = 127 ;
    I(idx_obs)    = 255 ;
    BW = edge(I);

    % walls 
    idx = find(I == 255) ;
    tmp = zeros(size(I)) ;
    tmp(idx) = 255 ;

    se = strel('square',6) ;
    J = imdilate(tmp,se);
    Q = BW & ~J ;

    idx_frontier = find(Q);
    [row,col] = ind2sub( size(I), idx_frontier );

    figure(1);clf;
    imshow(I) ;
    hold on
    plot(col,row,'r.')
    
end
