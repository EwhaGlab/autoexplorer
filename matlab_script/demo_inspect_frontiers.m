
cellsize = 20  ;

q = [ (pt(1,:) - origin.X )/0.05 ; (pt(2,:) - origin.Y )/0.05 ]

for idx=1:12
   
    p = q(:,idx) ; 
    roi = I( p(1)-cellsize:p(1)+cellsize, p(2)-cellsize:p(2)+cellsize ) ;
    
    figure(1); clf; hold on
    imshow(I) ; hold on; plot( p(1), p(2), 'r+' )

    figure(2); clf;
    imshow(roi)
    mean(roi(:))
    pause
    
end