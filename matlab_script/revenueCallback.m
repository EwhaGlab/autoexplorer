function revenueCallback( src, message )
    
    global I;
    global point ;
    global ox_ ;
    global oy_ ;
    global res ;
    global w;
    global h;
    global orig ;

    for idx=1:size(1,pt_)
        px = (pts_(idx,1) - ox_)/res ;
        py = (pts_(idx,2) - oy_)/res ;
        
        roi = I( px-[-10:10], py-[-10:10]) ;
        
        figure(2 + idx)
        imshow(roi) ;
        
    end
    

end